import os
import numpy as np
import cv2
import json
import matplotlib.pyplot as plt
from depth_image_processing import *
from obstacle import Obstacle
from ros_publish import send_obstacle_data
from localization_listener import update_position

saved_dir = 'src/NASA-RMC-2019/obstacle_detection/data/saved_frames/'
localization_dir = 'src/NASA-RMC-2019/obstacle_detection/data/localization_data/'
global_map_dir = 'src/NASA-RMC-2019/obstacle_detection/data/obstacle_maps/'

try:
	os.mkdir(localization_dir)
except Exception as e:
	pass
	
try:
	os.mkdir(saved_dir)
except Exception as e:
	pass
	
try:
	os.mkdir(global_map_dir)
except Exception as e:
	pass

for f in os.listdir(saved_dir):
	os.remove(saved_dir + f)

for f in os.listdir(localization_dir):
	os.remove(localization_dir + f)
	
for f in os.listdir(global_map_dir):
	os.remove(global_map_dir + f)


def get_ground_plane_roi(depth_frame, depth_cutoff, y_cutoff, roi_x, roi_y):
	"""
	Crops the depth frame and applies a circular mask to define the region of interest for ground plane estimation

	Args:
		depth_frame (:obj:`numpy.float32`): [424, 512] depth frame
		depth_cutoff (:obj:`float`) : furthest depth to keep, Kinect loses accuracy beyond 4.5m
		y_cutoff (:obj:`int`) : minimum height to keep, everything else set to zero
		roi_x (:obj:`int`) : x coordinate of center of circular mask
		roi_y (:obj:`int`) : y coordinate of center of circular mask

	Returns:
		Cropped depth frame with only region of interest (preferably a rough estimate of the ground plane)
	"""
	h, w = depth_frame.shape[:2]

	depth_frame[depth_frame > depth_cutoff] = 0
	depth_frame[:y_cutoff, ...] = 0

	ground_plane_roi = depth_frame.copy()
	ground_plane_roi[:y_cutoff + 100, ...] = 0
	mask = create_circular_mask(h, w, center=[roi_x, roi_y], radius=100)
	ground_plane_roi[mask] = 0
	return ground_plane_roi


def orient_point_cloud_to_ground_plane(xyz_arr, roi_point_cloud, thetas, phis, num_points, num_planes, memory):
	"""
	Computes the estimated ground plane and orients the point cloud coordinate system to have that plane be the XZ plane

	Args:
		xyz_arr (:obj:`numpy.float32`) : 3D point cloud of the entire depth frame represented as an [N, 3] Numpy array
		roi_point_cloud (:obj:`numpy.float32`) : 3D point cloud of cropped ROI represented as an [N, 3] Numpy array which is used to estimate the ground plane
		thetas (:obj:`list` of `float`) : list of Kinect elevation values currently in memory
		phis (:obj:`list` of `float`) : list of Kinect azimuth values currently in memory
		num_points (:obj:`int`) : number of points to consider in estimating each ground plane
		num_planes (:obj:`int`) : number of ground plane estimates to average over
		memory (:obj:`int`) : number of orientations to hold in memory at a time

	Returns:
		Re-oriented point cloud, ROI point cloud, and center of the ground plane estimate

	Examples:
		.. highlight:: python
		.. code-block:: python

			xyz_arr, roi_point_cloud, center = orient_point_cloud_to_ground_plane(xyz_arr, roi_point_cloud, thetas, phis, num_points, num_planes, memory)

	"""
	center, plane, phi, theta = get_orientation(roi_point_cloud, num_points, num_planes)
	thetas = np.append(thetas, theta)
	phis = np.append(phis, phi)
	# print(center, plane, np.median(theta), np.median(phis))
	CameraPosition['elevation'] = np.median(thetas)
	CameraPosition['azimuth'] = np.median(phis)
	if thetas.size > memory:
		thetas = thetas[1:]
	if phis.size > memory:
		phis = phis[1:]
	update_position()
	f = open(localization_dir + '%d.json' % len(os.listdir(localization_dir)),'w')
	f.write(json.dumps(CameraPosition))
	f.close()
	center = apply_camera_orientation(center, CameraPosition)
	plane = apply_camera_orientation(plane, CameraPosition)
	xyz_arr = apply_camera_matrix_orientation(xyz_arr, CameraPosition)
	return xyz_arr, roi_point_cloud, center


def project_plane_to_2d(xyz_arr, img, center, dist_thresh):
	"""
	Removes all points above some threshold of the estimated ground plane and projects the remaining points back to 2D

	Args:
		xyz_arr (:obj:`numpy.float32`): 3D point cloud represented as an [N, 3] Numpy array
		img (:obj:`numpy.uint8`) : image representaion of depth frame
		center (:obj:`numpy.float32`) : XYZ coordinate of center of ground plane
		dist_thresh (:obj:`float`) : maximum distance from the ground plane in meters

	Returns:
		Image representation of depth frame containing only the potential obstacle points
	"""
	plane_img = np.zeros(img.size)
	plane_img[xyz_arr[:, 2] > dist_thresh + center[2]] = 1

	plane_img = np.uint8(np.reshape(plane_img, (424, 512)) * 255)  # reshape to match depth data and convert to uint8
	plane_img = np.uint8(
		(np.ones((424, 512)) * 255) - plane_img)  # invert img so pixel value corresponds to NOT ground plane
	ret, plane_img = cv2.threshold(plane_img, 0, 255,
								   cv2.THRESH_BINARY)  # filter points that are probaly not ground plane
	plane_img = cv2.subtract(img, plane_img)
	return plane_img


def get_dimensions_from_contour(img, cntr, kernel):
	"""
	Estimated obstacle dimensions from a contour

	Args:
		img (:obj:`numpy.uint8`) : image representaion of depth frame
		cntr (:obj:`cv2.OutputArrayOfArrays`) : OpenCV contour object
		kernel (:obj:`numpy.uint8`) : kernel to use for mask erosion

	Returns:
		Mask for obstacle image, bounding box of obstacle, x coordinate of bottom left corner of bounding box,
		y coording of bottom left corner of bounding box, length of bounding box, and height of bounding box

	Examples:
		.. highlight:: python
		.. code-block:: python

			mask, box, x, y, obj_length, obj_height = get_dimensions_from_contour(img, cntr, kernel)

	"""
	mask = np.zeros_like(img)  # mask will contain the fitted and adjusted ellipse of a single obstacle
	ellipse = cv2.fitEllipse(cntr)
	x, y, obj_length, obj_height = cv2.boundingRect(cntr)
	rect = cv2.minAreaRect(cntr)

	equi_diameter = obj_length  # bounding rectangle gives a better approximation of diameter

	box = cv2.boxPoints(rect)
	box = np.int0(box)
	mask = cv2.ellipse(mask, ellipse, (255, 255, 255), -1)  # draw the fitted ellipse
	rows = mask.shape[0]
	cols = mask.shape[1]
	M = np.float32([[1, 0, 0], [0, 1, equi_diameter / 4]])  # shift mask down to match obstacle, not edge
	mask = cv2.warpAffine(mask, M, (cols, rows))
	mask = cv2.erode(mask, kernel, iterations=3)  # erode the mask to remove background points
	return mask, box, x, y, obj_length, obj_height


def process_obstacle(color, cx, cy, box, x, y, obj_length, obj_height, obj_depth,
					 equi_diameter, obstacle_list, obstacle_lifetime, obstacle_id, visualize, send_data):
	"""
	Consider each possible obstacle and decide if it meets the criteria for detection and data publishing.
	Optionally displays visualizations of the algorithm in real-time and publishes obstacle data using ROS.

	Args:
		color (:obj:`numpy.uint8`) : ``uint8`` RGB image representation of depth frame used to visualize detected obstacles
		cx (:obj:`int`) : estimated x coordinate of the center of obstacle in depth frame
		cy (:obj:`int`) : estimated y coordinate of the center of obstacle in depth frame
		box (:obj:`cv2.RotatedRect`) : bounding box of obstacle
		x (:obj:`int`) : x coordinate of bottom left corner of bounding box
		y (:obj:`int`) : y coordinate of bottom left corner of bounding box
		obj_length (:obj:`int`) : length of bounding box in pixels
		obj_height (:obj:`int`) : height of bounding box in pixels
		obj_depth (:obj:`float`) : estimated depth (z coordinate) of bounding box area in original depth frame in millimeters
		equi_diameter (:obj:`int`) : estimated obstacle diameter in pixels
		obstacle_list (:obj:`list` of :obj:`Obstacle`) : list of currently detected obstacles in memory
		obstacle_lifetime (`int`) : default lifetime for newly detected obstacles
		obstacle_id (`int`) : current maximum obstacle ID reached, ensures unique IDs
		visualize (`bool`) : boolean value specifying if obstacle detection should be visualized by displaying the result
		of drawing the bounding box and measured values on each obstacle
		send_data (`bool`) : boolean value specifying if detected obstacles should be published to the ``obstacleDetection`` topic
	"""
	coords = depth_to_point_cloud_pos(cx, cy, obj_depth)  # convert obstacle depth to XYZ coordinate
	mm_diameter = equi_diameter * (1.0 / CameraParams['fx']) * obj_depth  # convert pixel diameter to mm

	print(coords)

	if 100 < mm_diameter < 400:
		new_obstacle = True
		current_obstacle = None
		for obstacle in obstacle_list:
			x_match = abs(obstacle.x - coords[0]) < 0.3
			y_match = abs(obstacle.y - coords[1]) < 0.3
			z_match = abs(obstacle.z - obj_depth) < 0.5
			diameter_match = abs(obstacle.diameter - mm_diameter) / 1000. < 0.5
			if x_match and y_match:
				obstacle.x = coords[0]
				obstacle.y = coords[1]
				obstacle.z = coords[2]
				obstacle.diameter = mm_diameter / 1000.
				new_obstacle = False
				obstacle.lifetime = obstacle_lifetime
				if send_data:
					send_obstacle_data(obstacle)
				current_obstacle = Obstacle(obstacle.id,
											obstacle.x,
											obstacle.y,
											obstacle.z,
											obstacle.diameter,
											obstacle_lifetime)
				if obstacle.lifetime == 0:
					obstacle_list.remove(obstacle)
				break
		if new_obstacle:
			current_obstacle = Obstacle(obstacle_id,
										coords[0],
										coords[1],
										coords[2],
										mm_diameter / 1000.,
										obstacle_lifetime)
			obstacle_id += 1
			if send_data:
				send_obstacle_data(current_obstacle)
			obstacle_list.append(current_obstacle)

		if visualize:
			# begin visualization
			cv2.drawContours(color, [box], 0, (0, 0, 255), 1)
			cv2.rectangle(color, (x, y), (x + obj_length, y + obj_height), (0, 255, 0), 2)
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(color, 'id = %d' % current_obstacle.id, (cx, cy + 15), font, 0.4, (255, 0, 255),
						1, cv2.LINE_AA)
			cv2.putText(color, "x = %.2f" % coords[0], (cx, cy + 30), font, 0.4, (0, 0, 255), 1,
						cv2.LINE_AA)
			cv2.putText(color, "y = %.2f" % coords[1], (cx, cy + 45), font, 0.4, (0, 255, 0), 1,
						cv2.LINE_AA)
			cv2.putText(color, "z = %.2f" % (obj_depth / 1000), (cx, cy + 60), font, 0.4, (255, 0, 127),
						1, cv2.LINE_AA)
			cv2.putText(color, "diameter = %.2f" % (mm_diameter / 1000), (cx, cy + 75), font, 0.4,
						(255, 127, 0), 1,
						cv2.LINE_AA)
	return obstacle_id


def remove_dead_obstacles(obstacle_list):
	"""
	Checks the lifetimes of all obstacles currently in memory to see if any have died.  If any obstacles have reached
	a lifetime of zero (meaning they have been absent from ``obstacle_lifetime`` frames) they are removed from the
	obstacle list

	Args:
		obstacle_list (:obj:`list` of :obj:`Obstacle`) : list of currently detected obstacles in memory
	"""
	for obstacle in obstacle_list:
		obstacle.lifetime -= 1
		if obstacle.lifetime == 0:
			obstacle_list.remove(obstacle)
		print(obstacle)
		

def plot_global_map(obstacle_list):
	fig = plt.figure(figsize=(8, 8))
	ax = plt.subplot(111)
	for obstacle in obstacle_list:
		ax.scatter(obstacle.x, obstacle.z, s=obstacle.diameter * 1000, alpha=obstacle.lifetime / 5., label=obstacle.id)
	ax.set_ylim(-1.5, 1.5)
	ax.set_xlim(-1.5, 1.5)
	fig.savefig(global_map_dir + '%d.png' % len(os.listdir(global_map_dir)))
	plt.close()


def get_obstacles_with_plane(depth_frame,
							 color_frame,
							 obstacle_list,
							 thetas,
							 phis,
							 obstacle_id,
							 num_planes=1,
							 dist_thresh=0.2,
							 memory=200,
							 obstacle_lifetime=5,
							 depth_cutoff=4500.,
							 y_cutoff=424//2,
							 roi_x=512//2,
							 roi_y=320,
							 send_data=False,
							 visualize=False,
							 save_frames=False):
	"""
	Main obstacle detection function.  Performs full cycle of algorithm on a single depth frame, updates the obstacle
	list, and optionally visualizes the detected obstacles in real-time, publishes the obstacle data to the ``obstacleDetection``
	topic, or save the visualizations for viewing (generally used when running the algorithm on the robot during testing)

	Args:
		depth_frame (:obj:`numpy.float32`) : [424, 512] depth frame of meter values
		color_frame: (:obj:`numpy.uint8`) : [424, 512] RGB image representation of Kinect color data overlayed on the depth frame
		obstacle_list (:obj:`list` of :obj:`Obstacle`) : list of currently detected obstacles in memory
		thetas (:obj:`list` of :obj:`float`) : list of Kinect elevation values currently in memory
		phis (:obj:`list` of :obj:`float`) : list of Kinect azimuth values currently in memory
		obstacle_id (:obj:`int`) : current maximum obstacle ID reached, ensures unique IDs
		num_planes (:obj:`int`) : number of ground plane estimates to average over
		dist_thresh (:obj:`float`) : maximum distance from the ground plane in meters
		memory (:obj:`int`) : memory: number of orientations to hold in memory at a time
		obstacle_lifetime (:obj:`int`) : default lifetime for newly detected obstacles
		depth_cutoff (:obj:`float`) : furthest depth to keep, Kinect loses accuracy beyond 4.5m
		y_cutoff (:obj:`int`) : minimum height to keep, everything else set to zero
		roi_x (:obj:`int`) : x coordinate of center of circular mask
		roi_y (:obj:`int`) : y coordinate of center of circular mask
		send_data (`bool`) : boolean value specifying if detected obstacles should be published to the ``obstacleDetection`` topic
		visualize (:obj:`bool`) : boolean value specifying if obstacle detection should be visualized by displaying the result
		of drawing the bounding box and measured values on each obstacle
		save_frames (:obj:`bool`) : boolean value specifying if detected obstacle visualizations should be saved to the disk

	Returns:
		[424, 512] Numpy array showing the locations of all detected obstacles (used for validation with labeled data)

	"""
	obstacles = np.zeros(depth_frame.shape)  # empty image that will store the locations of detected obstacles
	img = np.uint8(depth_frame / 4500. * 255)  # image representation of depth frame
	# crop the depth frame and apply a circular mask
	ground_plane_roi = get_ground_plane_roi(depth_frame, depth_cutoff, y_cutoff, roi_x, roi_y)

	xyz_arr = depth_matrix_to_point_cloud(depth_frame)  # convert depth data to point cloud
	roi_point_cloud = depth_matrix_to_point_cloud(ground_plane_roi)  # convert ROI to point cloud
	num_points = np.count_nonzero(roi_point_cloud[..., 0]) // 100  # use 1% of ROI points for each plane fit
	try:
		# fit a plane to the ROI point cloud and re-orient both point clouds to use that as the ground plane
		xyz_arr, roi_point_cloud, center = orient_point_cloud_to_ground_plane(xyz_arr,
																			  roi_point_cloud,
																			  thetas,
																			  phis,
																			  num_points,
																			  num_planes,
																			  memory)

		# project the remaining points back to 2D
		plane_img = project_plane_to_2d(xyz_arr, img, center, dist_thresh)

		# noise removal
		kernel = np.ones((3, 3), np.uint8)
		opening = cv2.morphologyEx(plane_img, cv2.MORPH_OPEN, kernel, iterations=3)  # erosion followed by dilation

		# erosiong
		opening = cv2.erode(opening, kernel=kernel, iterations=1)

		color = color_frame.copy()  # BGR image to draw labels on

		# begin contour detection
		_, contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		color = cv2.drawContours(color, contours, -1, (0, 255, 0), 1)
		for cntr in contours:
			try:
				# calculate diameter of equivalent circle
				# this measurement is only used for checking if countours fit our bounds
				area = cv2.contourArea(cntr)
				equi_diameter = np.sqrt(4 * area / np.pi)

				# Hardcoded Diameter Range in pixels
				LOW_DIAMETER_BOUND = 20
				HIGH_DIAMETER_BOUND = 150

				HIGH_DISTANCE_BOUND = 4500
				# Original tolerances were 20 and 150

				if (equi_diameter > LOW_DIAMETER_BOUND and equi_diameter < HIGH_DIAMETER_BOUND):
					mask, box, x, y, obj_length, obj_height = get_dimensions_from_contour(img, cntr, kernel)
					img_fg = cv2.bitwise_and(depth_frame, depth_frame,
											 mask=mask)  # use the mask to isolate original depth values
					img_fg = cv2.medianBlur(img_fg, 5)  # median blur to further remove noise
					obstacles = cv2.add(np.float32(img_fg), np.float32(obstacles))

					mean_val = np.median(img_fg[img_fg.nonzero()])  # compute the non-zero average of obstacle depth values

					moment = cv2.moments(cntr)  # get the centroid of the obstacle using its moment
					cx = int(moment['m10'] / moment['m00'])
					cy = int(moment['m01'] / moment['m00'])

					if mean_val < HIGH_DISTANCE_BOUND:  # kinect loses accuracy beyond 4.5m
						obstacle_id = process_obstacle(color, cx, cy, box, x, y, obj_length, obj_height, mean_val,
										 equi_diameter, obstacle_list, obstacle_lifetime, obstacle_id, visualize, send_data)
			except cv2.error as e:
				print(e)
				pass

		if save_frames:
			try:
				os.mkdir(saved_dir)
			except Exception as e:
				pass
			rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
			plt.imsave(saved_dir + '%d.png' % len(os.listdir(saved_dir)), rgb)

		if visualize:
			print('next frame...')
			#cv2.imshow('detected_obstacles', color)
			#cv2.imshow('plane', plane_img)
			#cv2.imshow('deph frame', depth_frame)
			#cv2.imshow('roi', np.uint8(ground_plane_roi / 4500. * 255.))

		remove_dead_obstacles(obstacle_list)

	except AssertionError as e:
		print('Not enough points to estimate the ground plane, trying again...')
		pass

	return obstacles, obstacle_id
