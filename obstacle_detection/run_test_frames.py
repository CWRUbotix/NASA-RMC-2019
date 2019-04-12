import os
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from pyqtgraph import mkQApp, GraphicsWindow
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph import Vector
import pyqtgraph.opengl as gl
from obstacle_detection.include.depth_image_processing import *
from obstacle_detection.include.obstacle import Obstacle

frames_dir = 'src/test_frames_4_1'
saved_dir = 'saved_frames/'
num_points = 500
num_planes = 5
memory = 200  # number of frames to keep in memory to compute median ground plane
obstacle_lifetime = 5  # number of absent frames before forgetting
dist_thresh = 0.2
depth_cutoff = 4500.
y_cutoff = 424 // 2
roi_x = 512 // 2
roi_y = 320
visualize = True
show_point_cloud = True
save_frames = True
qapp = mkQApp()


def set_point_cloud_position(w, pos=None, distance=None, elevation=None, azimuth=None):
    if pos is not None:
        w.opts['center'] = pos
    if distance is not None:
        w.opts['distance'] = distance
    if elevation is not None:
        w.opts['elevation'] = elevation
    if azimuth is not None:
        w.opts['azimuth'] = azimuth
    w.update()


def run_obstacle_detection():
    global qapp
    if show_point_cloud:
        # QT app
        gl_widget = gl.GLViewWidget()
        gl_widget.show()
        gl_grid = gl.GLGridItem()
        gl_widget.addItem(gl_grid)
        # initialize some points data
        pos = np.zeros((1, 3))

        sp2 = gl.GLScatterPlotItem(pos=pos)
        sp2.setGLOptions('opaque')  # Ensures not to allow vertexes located behind other vertexes to be seen.

        gl_widget.addItem(sp2)

    if save_frames:
        try:
            os.mkdir(saved_dir)
        except FileExistsError as e:
            pass
        for f in os.listdir(saved_dir):
            os.remove(saved_dir + f)

    thetas = np.array([])
    phis = np.array([])
    obstacle_list = []
    obstacle_id = 0

    for i in range(0, len(os.listdir(frames_dir))):
        #print(i)
        depth_frame = np.load(frames_dir + '/%d.npy' % i)
        h, w = depth_frame.shape[:2]

        depth_frame[depth_frame > depth_cutoff] = 0

        depth_frame[:y_cutoff, ...] = 0

        obstacles = np.zeros(depth_frame.shape)  # empty image that will store the locations of detected obstacles
        img = np.uint8(depth_frame / 4500. * 255.)

        ground_plane_roi = depth_frame.copy()
        ground_plane_roi[:y_cutoff + 100, ...] = 0
        mask = create_circular_mask(h, w, center=[roi_x, roi_y], radius=100)
        ground_plane_roi[mask] = 0

        xyz_arr = depth_matrix_to_point_cloud(depth_frame)  # convert depth data to XYZ coordinates
        roi_point_cloud = depth_matrix_to_point_cloud(ground_plane_roi)
        num_points = np.count_nonzero(roi_point_cloud[..., 0]) // 100
        center, plane, phi, theta = get_orientation(roi_point_cloud, num_points, 1)
        thetas = np.append(thetas, theta)
        phis = np.append(phis, phi)
        #print(center, plane, np.median(theta), np.median(phis))
        CameraPosition['elevation'] = np.median(thetas)
        CameraPosition['azimuth'] = np.median(phis)
        if thetas.size > memory:
            thetas = thetas[1:]
        if phis.size > memory:
            phis = phis[1:]
        center = apply_camera_orientation(center, CameraPosition)
        plane = apply_camera_orientation(plane, CameraPosition)
        xyz_arr = apply_camera_matrix_orientation(xyz_arr, CameraPosition)
        plane_img = np.zeros(img.size)
        plane_img[xyz_arr[:, 2] > dist_thresh + center[2]] = 1

        plane_img = np.uint8(np.reshape(plane_img, (424, 512)) * 255)  # reshape to match depth data and convert to uint8
        plane_img = np.uint8(
            (np.ones((424, 512)) * 255) - plane_img)  # invert img so pixel value corresponds to NOT ground plane
        ret, plane_img = cv2.threshold(plane_img, 0, 255,
                                       cv2.THRESH_BINARY)  # filter points that are probaly not ground plane
        plane_img = cv2.subtract(img, plane_img)

        # noise removal
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(plane_img, cv2.MORPH_OPEN, kernel, iterations=3)  # erosion followed by dilation

        # erosiong
        opening = cv2.erode(opening, kernel=kernel, iterations=1)

        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # BGR image to draw labels on

        # begin contour detection
        contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                    img_fg = cv2.bitwise_and(depth_frame, depth_frame,
                                             mask=mask)  # use the mask to isolate original depth values
                    img_fg = cv2.medianBlur(img_fg, 5)  # median blur to further remove noise
                    obstacles = cv2.add(np.float32(img_fg), np.float32(obstacles))

                    mean_val = np.median(img_fg[img_fg.nonzero()])  # compute the non-zero average of obstacle depth values

                    moment = cv2.moments(cntr)  # get the centroid of the obstacle using its moment
                    cx = int(moment['m10'] / moment['m00'])
                    cy = int(moment['m01'] / moment['m00'])

                    if mean_val < HIGH_DISTANCE_BOUND:  # kinect loses accuracy beyond 4.5m
                        coords = depth_to_point_cloud_pos(cx, cy, mean_val)  # convert obstacle depth to XYZ coordinate
                        mm_diameter = equi_diameter * (1.0 / CameraParams['fx']) * mean_val  # convert pixel diameter to mm

                        if 100 < mm_diameter < 400:
                            new_obstacle = True
                            current_obstacle = None
                            for obstacle in obstacle_list:
                                x_match = abs(obstacle.x - coords[0]) < 0.3
                                y_match = abs(obstacle.y - coords[1]) < 0.3
                                z_match = abs(obstacle.z - mean_val) < 0.5
                                diameter_match = abs(obstacle.diameter - mm_diameter) / 1000. < 0.5
                                if x_match and y_match:
                                    obstacle.x = coords[0]
                                    obstacle.y = coords[1]
                                    obstacle.z = coords[2]
                                    obstacle.diameter = mm_diameter / 1000
                                    new_obstacle = False
                                    obstacle.lifetime = obstacle_lifetime
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
                                                            mm_diameter,
                                                            obstacle_lifetime)
                                obstacle_id += 1
                                obstacle_list.append(current_obstacle)

                            if visualize:
                                # begin visualization
                                cv2.drawContours(color, [box], 0, (0, 0, 255), 1)
                                cv2.rectangle(color, (x, y), (x + obj_length, y + obj_height), (0, 255, 0), 2)
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                cv2.putText(color, 'id = %d' % current_obstacle.id, (cx, cy + 15), font, 0.4, (255, 0, 255), 1, cv2.LINE_AA)
                                cv2.putText(color, "x = %.2f" % coords[0], (cx, cy + 30), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
                                cv2.putText(color, "y = %.2f" % coords[1], (cx, cy + 45), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
                                cv2.putText(color, "z = %.2f" % (mean_val / 1000), (cx, cy + 60), font, 0.4, (255, 0, 127), 1, cv2.LINE_AA)
                                cv2.putText(color, "diameter = %.2f" % (mm_diameter / 1000), (cx, cy + 75), font, 0.4, (255, 127, 0), 1,
                                            cv2.LINE_AA)
            except cv2.error as e:
                print(e)
                pass

            if save_frames:
                plt.imsave(saved_dir + '%d.png' % i, color)

        if show_point_cloud:
            # Calculate a dynamic vertex size based on window dimensions and camera's position - To become the "size" input for the scatterplot's setData() function.
            v_rate = 5.0  # Rate that vertex sizes will increase as zoom level increases (adjust this to any desired value).
            v_scale = np.float32(v_rate) / gl_widget.opts['distance']  # Vertex size increases as the camera is "zoomed" towards center of view.
            v_offset = (gl_widget.geometry().width() / 2000) ** 2  # Vertex size is offset based on actual width of the viewport.
            v_size = v_scale + v_offset
            set_point_cloud_position(gl_widget, pos=Vector(center[1] + 2, center[2], center[0] - 1))
            point_cloud_color = color.copy()
            _, plane_thresh = cv2.threshold(plane_img, 1, 2, cv2.THRESH_BINARY)
            point_cloud_color[:, :, 1] *= plane_thresh
            colors = np.divide(point_cloud_color, 255)  # values must be between 0.0 - 1.0

            colors = colors.reshape(colors.shape[0] * colors.shape[1], 3)
            #colors = colors[..., ::-1]  # BGR to RGB
            sp2.setData(pos=xyz_arr, size=v_size, color=colors)

        if visualize:
            cv2.imshow('detected_obstacles', color)
            cv2.imshow('plane', plane_img)
            cv2.imshow('roi', np.uint8(ground_plane_roi / 4500. * 255.))

        for obstacle in obstacle_list:
            obstacle.lifetime -= 1
            if obstacle.lifetime == 0:
                obstacle_list.remove(obstacle)
            print(obstacle)
        print('\n')

        key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break


if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        t = QtCore.QTimer()
        t.timeout.connect(run_obstacle_detection)
        t.start(50)

        sys.exit(qapp.exec_())


