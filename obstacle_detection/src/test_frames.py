# coding: utf-8

import numpy as np
import math
import cv2
import pika
import sys
import copy
import time
import os
import glob
import messages_pb2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel


try:
		from pylibfreenect2 import OpenCLPacketPipeline
		pipeline = OpenCLPacketPipeline()
except:
		try:
	   		from pylibfreenect2 import OpenGLPacketPipeline
			pipeline = OpenGLPacketPipeline()
		except:
	   		from pylibfreenect2 import CpuPacketPipeline
			pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)


num_frames = 500
frame_i = 0
save_frames = True
for f in files:
	os.remove(f)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

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

h,w = 512, 424
FOVX = 1.232202 #horizontal FOV in radians
focal_x = device.getIrCameraParams().fx #focal length x
focal_y = device.getIrCameraParams().fy #focal length y
principal_x = device.getIrCameraParams().cx #principal point x
principal_y = device.getIrCameraParams().cy #principal point y
undistorted = Frame(h, w, 4)
registered = Frame(h, w, 4)

time.sleep(30)

while frame_i < num_frames:
	
	frames = listener.waitForNewFrame()
	depth_frame = frames["depth"]
	color = frames["color"]
	registration.apply(color, depth_frame, undistorted, registered)
	#convert image
	color = registered.asarray(np.uint8)
	color = cv2.flip(color,1)
	img = depth_frame.asarray(np.float32) / 4500.
	imgray = np.uint8(depth_frame.asarray(np.float32)/255.0)
	#flip images
	img = cv2.flip(img,1)
	imgray = cv2.flip(imgray,1)
	np.save("test_frames/" + str(frame_i)+".npy",depth_frame.asarray(np.float32))
	frame_i += 1


device.stop()
device.close()

sys.exit(0)
