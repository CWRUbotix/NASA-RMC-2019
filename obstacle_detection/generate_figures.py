import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.mplot3d import Axes3D
from obstacle_detection.include.depth_image_processing import depthMatrixToPointCloudPos, get_orientation, applyCameraOrientation, applyCameraMatrixOrientation, CameraPosition, depthToPointCloudPos, CameraParams

depth_frame = np.load('src/test_frame.npy')
depth_frame = np.flip(depth_frame, 1)
depth_frame[depth_frame > 1500.] = 0

fig, ax = plt.subplots()
divider = make_axes_locatable(ax)
cax = divider.append_axes('right', size='5%', pad=0.25)
img = ax.imshow(depth_frame, cmap='OrRd')
ax.set_xticks([])
ax.set_yticks([])
colorbar = fig.colorbar(img, cax=cax, orientation='vertical')
colorbar.ax.get_yaxis().labelpad = 15
colorbar.ax.set_ylabel('millimeters', rotation=270)
plt.show()

img = np.uint8(depth_frame) #some opencv functions require a byte image

xyz_arr = depthMatrixToPointCloudPos(depth_frame) #convert depth data to XYZ coordinates
print(xyz_arr.shape)
center, plane, theta = get_orientation(xyz_arr, xyz_arr.shape[0] // 32, 1)
print(center, plane, theta)
CameraPosition['elevation'] = -theta
center = applyCameraOrientation(center, CameraPosition)
plane = applyCameraOrientation(plane, CameraPosition)
xyz_arr = applyCameraMatrixOrientation(xyz_arr, CameraPosition)

plane_img = np.zeros(len(xyz_arr))
plane_img[xyz_arr[:,2] > 0.1 - center[2]] = 1

plane_img = np.uint8(np.reshape(plane_img,(424,512)) * 255) #reshape to match depth data and convert to uint8
plane_img = np.uint8((np.ones((424,512)) * 255) - plane_img) #invert img so pixel value corresponds to NOT ground plane
ret, thresh = cv2.threshold(plane_img,0,255,cv2.THRESH_BINARY) #filter points that are probaly not ground plane
plane_img = cv2.subtract(img, thresh)

ret, thresh = cv2.threshold(plane_img,0,255,cv2.THRESH_BINARY) #filter points that are probaly not ground plane

fig, ax = plt.subplots()
im = ax.imshow(thresh, cmap='OrRd')
ax.set_xticks([])
ax.set_yticks([])
plt.show()

#noise removal
kernel = np.ones((3,3),np.uint8)
opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN, kernel, iterations = 3) #erosion followed by dilation

color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) #BGR image to draw labels on

print(opening.shape)

#begin contour detection
contours, hierarchy = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
color = cv2.drawContours(color, contours, -1, (0,255,0), 1)
for cntr in contours:
    #try:
        #calculate diamter of equivalent circle
        #this measurement is only used for checking if countours fit our bounds
        area = cv2.contourArea(cntr)
        equi_diameter = np.sqrt(4*area/np.pi)

        #Hardcoded Diameter Range in pixels
        LOW_DIAMETER_BOUND = 20
        HIGH_DIAMETER_BOUND = 150

        HIGH_DISTANCE_BOUND = 4500
        #Original tolerances were 20 and 150

        if(equi_diameter>LOW_DIAMETER_BOUND and equi_diameter<HIGH_DIAMETER_BOUND):
            mask = np.zeros_like(img) #mask will contain the fitted and adjusted ellipse of a single obstacle
            ellipse = cv2.fitEllipse(cntr)
            x,y,obj_length,obj_height = cv2.boundingRect(cntr)
            rect = cv2.minAreaRect(cntr)

            equi_diameter = obj_length #bounding rectangle gives a better approximation of diameter

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            mask = cv2.ellipse(mask,ellipse,(255,255,255),-1) #draw the fitted ellipse
            rows = mask.shape[0]
            cols = mask.shape[1]
            M = np.float32([[1,0,0],[0,1,equi_diameter/4]]) #shift mask down to match obstacle, not edge
            mask = cv2.warpAffine(mask,M,(cols,rows))
            mask = cv2.erode(mask, kernel, iterations=3) #erode the mask to remove background points
            img_fg = cv2.bitwise_and(depth_frame,depth_frame,mask = mask) #use the mask to isolate original depth values
            img_fg = cv2.medianBlur(img_fg,5) #median blur to further remove noise

            mean_val = np.median(img_fg[img_fg.nonzero()]) #compute the non-zero average of obstacle depth values
            min_val, distance_to_object, min_loc, max_loc = cv2.minMaxLoc(img_fg)

            moment = cv2.moments(cntr) #get the centroid of the obstacle using its moment
            cx = int(moment['m10']/moment['m00'])
            cy = int(moment['m01']/moment['m00'])

            if mean_val < HIGH_DISTANCE_BOUND: #kinect loses accuracy beyond 4.5m
                coords = depthToPointCloudPos(cx, cy, mean_val) #convert obstacle depth to XYZ coordinate

                mm_diameter = (equi_diameter) * (1.0 / CameraParams['fx']) * mean_val #convert pixel diameter to mm
                fig, ax = plt.subplots()
                divider = make_axes_locatable(ax)
                cax = divider.append_axes('right', size='5%', pad=0.25)
                img = ax.imshow(depth_frame, cmap='OrRd')
                ax.set_xticks([])
                ax.set_yticks([])
                colorbar = fig.colorbar(img, cax=cax, orientation='vertical')
                colorbar.ax.get_yaxis().labelpad = 15
                colorbar.ax.set_ylabel('millimeters', rotation=270)

                # Create a Rectangle patch
                rect = patches.Rectangle((x, y), obj_length, obj_height, linewidth=1, edgecolor='g', facecolor='none')

                # Add the patch to the Axes
                ax.add_patch(rect)

                ax.text(0.95, 0.05, 'x: %.2f mm' % (coords[0] * 1000),
                        verticalalignment='bottom', horizontalalignment='right',
                        transform=ax.transAxes,
                        color='red', fontsize=12)
                ax.text(0.95, 0.1, 'y: %.2f mm' % (coords[1] * 1000),
                        verticalalignment='bottom', horizontalalignment='right',
                        transform=ax.transAxes,
                        color='green', fontsize=12)
                ax.text(0.95, 0.15, 'z: %.2f mm' % mean_val,
                        verticalalignment='bottom', horizontalalignment='right',
                        transform=ax.transAxes,
                        color='blue', fontsize=12)
                ax.text(0.95, 0.2, 'diameter: %.2f mm' % mm_diameter,
                        verticalalignment='bottom', horizontalalignment='right',
                        transform=ax.transAxes,
                        color='black', fontsize=12)

                plt.show()



# plot raw data
plot_all_points = False
plt.figure()
ax = plt.subplot(111, projection='3d')
sample_points = np.random.choice(np.arange(0, xyz_arr.shape[0]), size=xyz_arr.shape[0] // 16, replace=False)
xs = xyz_arr[..., 1]
ys = xyz_arr[..., 2]
zs = xyz_arr[..., 0]

if not plot_all_points:
    xs = xs[sample_points]
    ys = ys[sample_points]
    zs = zs[sample_points]
ax.scatter(zs, xs, ys, color='b', s=0.5, alpha=0.5)

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X, Y = np.meshgrid(np.arange(xlim[0] * 2, xlim[1] * 2),
                   np.arange(ylim[0] * 2, ylim[1] * 2))

# calculate corresponding z
d = -center.dot(plane)
z = (-plane[1] * X - plane[2] * Y - d) * 1. / plane[0]

ax.plot_surface(z, X, Y, alpha=0.5)

ax.plot_wireframe(z, X, Y, color='k', linewidth=0.5)

ax.set_ylim(ylim)
ax.set_xlim(xlim)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()