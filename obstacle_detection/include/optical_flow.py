#!/usr/bin/env python
import cv2
import numpy as np

# params for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


def compute_optical_flow(prev, frame, p0):
    # calculate optical flow
    if p0 is None:
        p0 = cv2.goodFeaturesToTrack(prev, mask=None, **feature_params)
    p1, st, err = cv2.calcOpticalFlowPyrLK(np.uint8(prev), np.uint8(frame), p0, None, **lk_params)
    # Select good points
    good_new = p1[st == 1]
    good_old = p0[st == 1]
    x_motion = 0
    y_motion = 0
    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        x_dif = a - c
        y_dif = b - d
        x_motion += x_dif
        y_motion += y_dif
        mask = cv2.line(mask // 1.0001, (a, b), (c, d), color[i].tolist(), 2)
    x_motion /= len(good_new)
    y_motion /= len(good_new)
    print('dX: %.2f, dY: %.2f' % (x_motion, y_motion))
    return p1, x_motion, y_motion
