"""
This module contains functions which process the depth data from a Kinect v2 sensor and prepares the data for obstacle detection

Attributes:
-----------

Attributes:

    CameraParams : Parameters of Kinect v2 sensor used for conversion between depth values and cartesian coordinates:

        +---------+---------+---------+---------+-----------+----------+-----------+
        | cx      | cy      | fx      | fy      | k1        | k2       | k3        |
        +=========+=========+=========+=========+===========+==========+===========+
        | 254.878 | 205.395 | 365.456 | 365.456 | 0.0905474 | -0.26819 | 0.0950862 |
        +---------+---------+---------+---------+-----------+----------+-----------+

    CameraPosition : Parameters describing the 3D position and orientation of the Kinect. Used for properly orienting the point cloud.
        Initially all values are 0, but are updated as the obstacle detection node estumates the ground plane

Functions:
----------

"""
import numpy as np
import math
import cv2
import os, os.path
import sys
import time
import pandas as pd
import random

#camera information based on the Kinect v2 hardware
CameraParams = {
  "cx":254.878,
  "cy":205.395,
  "fx":365.456,
  "fy":365.456,
  "k1":0.0905474,
  "k2":-0.26819,
  "k3":0.0950862,
  "p1":0.0,
  "p2":0.0,
}

# Kinect's physical orientation in the real world.
CameraPosition = {
    "x": 0, # actual position in meters of kinect sensor relative to the viewport's center.
    "y": 0, # actual position in meters of kinect sensor relative to the viewport's center.
    "z": 0, # height in meters of actual kinect sensor from the floor.
    "roll": 0, # angle in degrees of sensor's roll (used for INU input - trig function for this is commented out by default).
    "azimuth": 0, # sensor's yaw angle in degrees.
    "elevation": -30, # sensor's pitch angle in degrees.
}

# params for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.01,
                      minDistance=7,
                      blockSize=10)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


def compute_optical_flow(prev, frame, resize_factor=10, cropping=500):
    new_size = 4500 // resize_factor - cropping // resize_factor
    hsv = np.zeros((new_size, new_size, 3), dtype=np.uint8)
    hsv[..., 1] = 255
    if prev is None:
        xyz_arr = depth_matrix_to_point_cloud(frame)
        return 0, 0, project_point_cloud_onto_plane(xyz_arr), hsv
    else:
        xyz_arr = depth_matrix_to_point_cloud(frame)
        proj_frame = project_point_cloud_onto_plane(xyz_arr)

    try:
        flow = cv2.calcOpticalFlowFarneback(prev, frame, None, 0.5, 3, 20, 3, 5, 1.2, 0)
    except cv2.error as e:
        return 0, 0, prev, hsv

    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    hsv[..., 0] = ang * 180 / np.pi
    hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    rgb = cv2.cvtColor(np.uint8(hsv), cv2.COLOR_HSV2BGR)
    mag[mag == np.inf] = 0

    motion_mag = np.median(mag[mag != 0])
    motion_ang = np.median(ang[ang != 0])

    x_motion = math.cos(motion_ang) * motion_mag * resize_factor
    y_motion = math.sin(motion_ang) * motion_mag * resize_factor

    print('dX: %.3f, dY: %.3f, Mag: %.3f, Ang: %.3f' % (x_motion, y_motion, motion_mag, motion_ang * 180 / math.pi))
    return x_motion, y_motion, proj_frame, rgb


def project_point_cloud_onto_plane(xyz_arr, resize_factor=10, cropping=500):
    normal = np.array([[1, 0, 0], [0, 1, 0]])
    proj = np.matmul(xyz_arr, normal.T)
    proj_img = np.zeros((4500, 4500))
    indices = np.int32(proj * 1000)
    indices[..., 1] += 4500 // 2
    indices = np.clip(indices, 0, 4499)
    proj_img[indices[..., 0], indices[..., 1]] = 255
    proj_img = proj_img[cropping:4500 - cropping, cropping:4500 - cropping]
    new_size = 4500 // resize_factor - cropping // resize_factor
    proj_img = cv2.resize(proj_img, (new_size, new_size), interpolation=cv2.INTER_AREA)
    proj_img = cv2.dilate(proj_img, np.ones((3, 3)), iterations=2)
    proj_img = cv2.blur(proj_img, (5, 5))
    return np.uint8(proj_img)


def depth_matrix_to_point_cloud(z, scale=1000):
    """
    Transforms an entire depth frame into a 3D point cloud using the Kinect parameters defined in ``CameraParams``

    Args:
        z (:obj:`numpy.float32`) : A depth frame, generally of millimeter values
        scale (:obj:`int`) : the conversion scale to use.  If the depth frame has millimeter units, then by default they will be converted to meters

    Returns:
        A point cloud represented as an [N, 3] Numpy array where each entry on axis 0 represent an XYZ coordinate

    Examples:
        .. highlight:: python
        .. code-block:: python

            xyz_arr = depth_matrix_to_point_cloud(depth_frame)
    """
    C, R = np.indices(z.shape)

    R = np.subtract(R, CameraParams['cx'])
    R = np.multiply(R, z)
    R = np.divide(R, CameraParams['fx'] * scale)

    C = np.subtract(C, CameraParams['cy'])
    C = np.multiply(C, z)
    C = np.divide(C, CameraParams['fy'] * scale)

    return np.column_stack((z.ravel() / scale, R.ravel(), -C.ravel()))


def depth_to_point_cloud_pos(x_d, y_d, z, scale=1000):
    """
    Transforms a single depth value to an XYZ coordinate using the Kinect parameters defined in ``CameraParams``

    Args:
        x_d (:obj:`int`) : the row index of the depth value in the depth frame
        y_d (:obj:`int`) : the column index of the depth value in the depth frame
        z (:obj:`float`) : the depth value at index [x_d, y_d]
        scale (:obj:`int`) : the conversion scale to use.  If the depth frame has millimeter units, then by default they will be converted to meters

    Returns:
        A tuple (X, Y, Z) representing the cartesian coordinates of the given point in the point cloud

    Examples:
        .. highlight:: python
        .. code-block:: python

            x, y, z = (row, col, depth)
    """
    x = (x_d - CameraParams['cx']) * z / CameraParams['fx']
    y = (y_d - CameraParams['cy']) * z / CameraParams['fy']

    return x / scale, y / scale, z / scale


def apply_camera_orientation(pt, CameraPosition=CameraPosition):
    """
    Transforms a single XYZ coordinate according to the position of the Kinect

    Args:
        pt (:obj:`numpy.float32`) : an XYZ coordinate of a point in the point cloud
        CameraPosition (:obj:`dict`) : the current position and orientation of the Kinect sensor.  Either hardcoded in the ``CameraPosition`` dictionary, or computed from the best-fit ground plane

    Returns:
        An updated XYZ coordinate where the input point has been transformed to correspond with the current camera position and orientation
    """
    # This runs slowly in Python as it is required to be called within a loop, but it is a more intuitive example than it's vertorized alternative (Purly for example)
    # use trig to rotate a vertex around a gimbal.
    def rotatePoints(ax1, ax2, deg):
        # math to rotate vertexes around a center point on a plane.
        hyp = np.sqrt(pt[ax1] ** 2 + pt[ax2] ** 2) # Get the length of the hypotenuse of the real-world coordinate from center of rotation, this is the radius!
        d_tan = np.arctan2(pt[ax2], pt[ax1]) # Calculate the vertexes current angle (returns radians that go from -180 to 180)

        cur_angle = np.degrees(d_tan) % 360 # Convert radians to degrees and use modulo to adjust range from 0 to 360.
        new_angle = np.radians((cur_angle + deg) % 360) # The new angle (in radians) of the vertexes after being rotated by the value of deg.

        pt[ax1] = hyp * np.cos(new_angle) # Calculate the rotated coordinate for this axis.
        pt[ax2] = hyp * np.sin(new_angle) # Calculate the rotated coordinate for this axis.

    #rotatePoints(0, 2, CameraPosition['roll']) #rotate on the Y&Z plane # Disabled because most tripods don't roll. If an Inertial Nav Unit is available this could be used)
    rotatePoints(1, 2, CameraPosition['elevation']) #rotate on the X&Z plane
    rotatePoints(0, 1, CameraPosition['azimuth']) #rotate on the X&Y plane

    # Apply offsets for height and linear position of the sensor (from viewport's center)
    pt[:] += np.float_([CameraPosition['x'], CameraPosition['y'], CameraPosition['z']])
    return pt


def apply_camera_matrix_orientation(pt, CameraPosition):
    """
    Transforms the entire 3D point cloud according to the position of the Kinect.  Basically an efficient vectorized version of ``apply_camera_orientation``

    Args:
        pt (:obj:`numpy.float32`) : 3D point cloud represented by an [N, 3] Numpy array
        CameraPosition (:obj:`dict`) : the current position and orientation of the Kinect sensor.  Either hardcoded in the ``CameraPosition`` dictionary, or computed from the best-fit ground plane

    Returns:
        Updated point cloud represented as an [N, 3] numpy array in correct orientation to the Kinect
    """
    # Kinect Sensor Orientation Compensation
    # bacically this is a vectorized version of applyCameraOrientation()
    # uses same trig to rotate a vertex around a gimbal.
    def rotatePoints(ax1, ax2, deg):
        # math to rotate vertexes around a center point on a plane.
        hyp = np.sqrt(pt[:, ax1] ** 2 + pt[:, ax2] ** 2) # Get the length of the hypotenuse of the real-world coordinate from center of rotation, this is the radius!
        d_tan = np.arctan2(pt[:, ax2], pt[:, ax1]) # Calculate the vertexes current angle (returns radians that go from -180 to 180)

        cur_angle = np.degrees(d_tan) % 360 # Convert radians to degrees and use modulo to adjust range from 0 to 360.
        new_angle = np.radians((cur_angle + deg) % 360) # The new angle (in radians) of the vertexes after being rotated by the value of deg.

        pt[:, ax1] = hyp * np.cos(new_angle) # Calculate the rotated coordinate for this axis.
        pt[:, ax2] = hyp * np.sin(new_angle) # Calculate the rotated coordinate for this axis.

    #rotatePoints(1, 2, CameraPosition['roll']) #rotate on the Y&Z plane # Disabled because most tripods don't roll. If an Inertial Nav Unit is available this could be used)
    rotatePoints(0, 2, CameraPosition['elevation']) #rotate on the X&Z plane
    rotatePoints(0, 1, CameraPosition['azimuth']) #rotate on the X&Y

    # Apply offsets for height and linear position of the sensor (from viewport's center)
    pt[:] += np.float_([CameraPosition['x'], CameraPosition['y'], CameraPosition['z']])
    return pt


def create_circular_mask(h, w, center=None, radius=None):
    """
    Creates a circular mask for defining the ROI for plane fitting

    Args:
        h (:obj:`int`) : height of frame (depth frame is 424)
        w (:obj:`int`) : width of frame (depth frame is 512)
        center (:obj:`int`) : (x, y) coordinates of the center of the circle.  If no value is given, center is assumed to be the center of the frame
        radius (:obj:`int`) : radius of the circle in pixels

    Returns:
        A [h, w] boolean Numpy array defining the circular mask
    """
    if center is None: # use the middle of the image
        center = [int(w/2), int(h/2)]
    if radius is None: # use the smallest distance between the center and image walls
        radius = min(center[0], center[1], w-center[0], h-center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

    mask = dist_from_center <= radius
    return mask


def plane_fit(points):
    """
    Fits a plane to the input point cloud by minimizing the orthogonal distance between each point and the plane using the method of least-squares

    Args:
        points (:obj:`numpy.float32`) : point cloud represented as an [N, 3] Numpy array

    Returns:
        A tuple defining the center of the plane

    Examples:
        .. highlight:: python
        .. code-block:: python

            center, plane = plane_fit(points)

    """
    from numpy.linalg import svd
    points = np.reshape(points, (np.shape(points)[0], -1)) # Collapse trialing dimensions
    assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
    ctr = points.mean(axis=1)
    x = points - ctr[:,np.newaxis]
    M = np.dot(x, x.T) # Could also use np.cov(x) here.
    return ctr, svd(M)[0][:,-1]


def get_orientation(xyz_arr, num_points, n_iter):
    """
    Estimates the best fit ground plane by iteratively fitting a plane to a set of random points using the ``plane_fit`` function

    Args:
        xyz_arr (:obj:`numpy.float32`) : 3D point cloud represented as an [N, 3] Numpy array
        num_points (:obj:`int`) : number of points to consider for each plane
        n_iter (:obj:`int`) : number of planes to fit before taking the average as the best estimate

    Returns:
        Tuple containing the center, normal, pitch, and azimuth of the Kinect sensor as estimated by the ground plane

    Examples:
        .. highlight:: python
        .. code-block:: python

            center, plane, phi, theta = get_orientation(xyz_arr, num_points)
    """
    pitch = []
    azimuth = []
    planes = []
    centers = []
    for _ in range(0, n_iter):
        indices = np.random.choice(np.arange(0, len(xyz_arr)), size=len(xyz_arr), replace=False)
        rand_points = xyz_arr[random.randrange(0, len(xyz_arr))]
        i = 0
        while len(rand_points) < num_points:
            if not xyz_arr[indices[i]].all() == 0:
                rand_points = np.vstack((rand_points, xyz_arr[indices[i]]))
            i += 1
        rand_points = np.array(rand_points).T
        ctr, P = plane_fit(rand_points)
        r = math.sqrt(P[0]**2 + P[1]**2 + P[2]**2)
        theta = math.acos(P[2]/r) * 180 / math.pi
        phi = math.atan2(P[1], P[0]) * 180 / math.pi
        pitch.append(phi)
        azimuth.append(theta)
        planes.append(P)
        centers.append(ctr)
    return np.mean(centers, axis=0), np.mean(planes, axis=0), np.mean(pitch), np.mean(azimuth)
