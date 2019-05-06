#!/usr/bin/env python
import os
import sys
import numpy as np
import cv2
import time
import math
import rospy
import pandas as pd
#from optical_flow import *
from obstacle_detection import get_obstacles_with_plane, plot_global_map
from depth_image_processing import *
from ros_publish import setup_obstacle_node
from localization_listener import update_position
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

try:
	from pylibfreenect2 import OpenCLPacketPipeline

	pipeline = OpenCLPacketPipeline()
except:

	from pylibfreenect2 import CpuPacketPipeline

	pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(None)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
	print("No device connected!")
	sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
							device.getColorCameraParams())

h, w = 512, 424
FOVX = 1.232202  # horizontal FOV in radians
focal_x = device.getIrCameraParams().fx  # focal length x
focal_y = device.getIrCameraParams().fy  # focal length y
principal_x = device.getIrCameraParams().cx  # principal point x
principal_y = device.getIrCameraParams().cy  # principal point y
undistorted = Frame(h, w, 4)
registered = Frame(h, w, 4)

thetas = np.array([])
phis = np.array([])
obstacle_list = []
obstacle_id = 0

visualize = False
save_test_data = False

frames_dir = 'src/NASA-RMC-2019/obstacle_detection/data/test_frames/'
if save_test_data:
	try:
		os.mkdir(frames_dir)
	except Exception as e:
		pass
	for f in os.listdir(frames_dir):
		os.remove(frames_dir + f)

print(os.getcwd())

frame_i = 0  # current frame used for saving data
frame_limit = -1  # number of frames to process, -1 will run indefinitely
prev_frame = None  # previous frame used for optical flow

setup_obstacle_node()

# uncomment to send global obstacle positions based on loclalization data
#update_position()

while not rospy.is_shutdown():
	frames = listener.waitForNewFrame()
	depth_frame = frames["depth"]
	color = frames["color"]
	registration.apply(color, depth_frame, undistorted, registered)
	color_frame = registered.asarray(np.uint8)

	if save_test_data:
		np.save(frames_dir + str(frame_i) + ".npy", depth_frame.asarray(np.float32))

	frame_i += 1
	if frame_i == frame_limit:
		break

	img = depth_frame.asarray(np.float32)

	output, obstacle_id = get_obstacles_with_plane(img,
									  prev_frame,
									  color_frame,
									  obstacle_list,
									  thetas,
									  phis,
									  obstacle_id,
									  send_data=True,
									  visualize=visualize,
									  save_frames=True)

	if visualize:
		plot_global_map(obstacle_list)
		key = cv2.waitKey(delay=1)
		if key == ord('q'):
			break
	listener.release(frames)

listener.release(frames)
device.stop()
device.close()

sys.exit(0)
