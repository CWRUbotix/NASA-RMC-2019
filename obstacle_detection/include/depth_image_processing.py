import numpy as np
import math
import cv2
import os, os.path
import sys
import time
import pandas as pd
import random

path = 'C:/Users/jumpr_000/Desktop/ransac_obstacle_detection/saved'
frames_dir = "kinect_data/" #the directory holding the saved depth images in .npy format

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


def depthMatrixToPointCloudPos(z, scale=1000):
    """
    xyz_arr = depthMatrixToPointCloudPos(depth_frame)

    Given a depth image, converts each
    point to an XYZ coordinate and returns
    an array of these points.
    The scale is a conversion factor.
    Default converts from milimeter
    depth data to meters.
    """
    C, R = np.indices(z.shape)

    R = np.subtract(R, CameraParams['cx'])
    R = np.multiply(R, z)
    R = np.divide(R, CameraParams['fx'] * scale)

    C = np.subtract(C, CameraParams['cy'])
    C = np.multiply(C, z)
    C = np.divide(C, CameraParams['fy'] * scale)

    return np.column_stack((z.ravel() / scale, R.ravel(), -C.ravel()))


def depthToPointCloudPos(x_d, y_d, z, scale=1000):
    """
    x, y, z = (row, col, depth)
    
    Given a point from a depth image
    and its position in the image,
    returns the x, y, and z coordinates
    relative to the camera location.
    The scale is a conversion factor.
    Default converts from milimeter
    depth data to meters.
    """
    x = (x_d - CameraParams['cx']) * z / CameraParams['fx']
    y = (y_d - CameraParams['cy']) * z / CameraParams['fy']

    return x / scale, y / scale, z / scale


def applyCameraOrientation(pt):
    # Kinect Sensor Orientation Compensation
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


def applyCameraMatrixOrientation(pt):
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


def plane_fit(points):
    """
    p, n = plane_fit(points)

    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fit an d-dimensional plane to the points.
    Return a point, p, on the plane (the point-cloud centroid),
    and the normal, n.
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
    center, plane, theta = get_orientation(xyz_arr, num_points)

    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fits a plane to the array n_iter times and
    returns the average of the planes centers,
    norms, and pitch degrees.
    """
    pitch = []
    planes = []
    centers = []
    for _ in range(0, n_iter):
        rand_points = []
        while len(rand_points) < num_points:
            index = random.randrange(0,len(xyz_arr))
            if not xyz_arr[index].all() == np.zeros(3).all():
                rand_points.append(xyz_arr[index])
        rand_points = np.array(rand_points).T
        ctr, P = plane_fit(rand_points)
        r = math.sqrt(P[0]**2 + P[1]**2 + P[2]**2)
        theta = math.acos(P[2]/r) * 180 / math.pi
        phi = math.atan(P[1]/P[0]) * 180 / math.pi
        pitch.append(theta)
        planes.append(P)
        centers.append(ctr)
    return np.mean(centers, axis = 0), np.mean(planes, axis = 0), np.mean(pitch)
