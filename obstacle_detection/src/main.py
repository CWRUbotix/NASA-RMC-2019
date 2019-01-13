#!/usr/bin/env python
import os
import numpy as np
import cv2
import time
import math
import pandas as pd
from obstacle_detection import get_obstacles_with_plane
from depth_image_processing import *

img = np.load('test_frame.npy')
while True:
	output = get_obstacles_with_plane(img, num_planes=46,
                                  num_points=45,
                                  dist_thresh=0.1,
                                  visualize=False,
                                  send_data=True)