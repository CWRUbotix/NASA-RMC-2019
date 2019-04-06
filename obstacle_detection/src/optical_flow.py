import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

path = 'frames/'
directory = os.fsencode(path)
frame_i = 1

# params for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))
# Take first frame and find corners in it
prev = np.load(path + '0.npy')
img = cv2.cvtColor(prev / 4500., cv2.COLOR_GRAY2BGR)
p0 = cv2.goodFeaturesToTrack(prev, mask=None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(img)
x = np.array([])
y = np.array([])
x_v = np.array([])
y_v = np.array([])
x_pos = 0
y_pos = 0

while True:
    try:
        frame = np.load(path + str(frame_i) + '.npy')
    except Exception as e:
        print('No more frames')
        break
    # calculate optical flow
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
    x_pos += x_motion
    y_pos += y_motion
    x = np.append(x, x_pos)
    y = np.append(y, y_pos)
    x_v = np.append(x_v, x_motion)
    y_v = np.append(y_v, y_motion)
    print('X:', x_pos, 'Y:', y_pos)
    img = cv2.cvtColor(frame / 4500., cv2.COLOR_GRAY2BGR)
    img = cv2.add(img, mask)
    plt.subplot(121)
    plt.plot(x, y, color='blue', linewidth=2, alpha=0.5)
    plt.title('relative XY offset')
    plt.subplot(122)
    plt.imshow(img)
    plt.title('depth image')
    plt.savefig('saved/' + str(frame_i) + '.png')
    # Now update the previous frame and previous points
    prev = frame.copy()
    p0 = good_new.reshape(-1, 1, 2)
    frame_i += 1


frames = np.arange(0, len(x))

plt.subplot(221)
plt.plot(x)
plt.title('X Position')
plt.subplot(222)
plt.plot(y)
plt.title('Y Position')
plt.subplot(223)
plt.plot(x_v)
plt.title('X Velocity')
plt.subplot(224)
plt.plot(y_v)
plt.title('Y Velocity')
plt.show()