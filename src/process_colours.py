#!/usr/bin/env python3

import argparse
import cv2
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

cli = argparse.ArgumentParser(description="Obtain pixel colours from an image")

cli.add_argument("Path", metavar="path", type=str, help="Path to the image file")

args = cli.parse_args()

img = cv2.imread(args.Path)
img2 = cv2.imread(args.Path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

print("Processing the image, a plot will appear shortly...")
hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

###3d plot
# h, s, v = cv2.split(hsv_img)
# fig = plt.figure()
# axis = fig.add_subplot(1, 1, 1, projection="3d")

# pixel_colors = hsv_img.reshape((np.shape(hsv_img)[0]*np.shape(hsv_img)[1], 3))
# norm = colors.Normalize(vmin=-1.,vmax=1.)
# norm.autoscale(pixel_colors)
# pixel_colors = norm(pixel_colors).tolist()

# axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
# axis.set_xlabel("Hue")
# axis.set_ylabel("Saturation")
# axis.set_zlabel("Value")
# plt.show()

# #red
# low_threshold = (-2, 190, 50)
# upper_threshold = (6, 260, 255)

# #blue 
# low_threshold = (102, 240, 80)
# upper_threshold = (107, 260, 200)

# #green
# low_threshold = (75, 150, 40)
# upper_threshold = (92, 260, 150)

# #yellow
low_threshold = (18, 170, 50)
upper_threshold = (30, 260, 200)

mask = cv2.inRange(hsv_img, low_threshold, upper_threshold)
result = cv2.bitwise_and(img, img, mask=mask)

lo_square = np.full((10, 10, 3), low_threshold, dtype=np.uint8) / 255.0
do_square = np.full((10, 10, 3), upper_threshold, dtype=np.uint8) / 255.0

# plt.subplot(1, 2, 1)
# plt.imshow(hsv_to_rgb(do_square))
# plt.subplot(1, 2, 2)
# plt.imshow(hsv_to_rgb(lo_square))
# plt.show()

plt.subplot(1, 2, 1)
plt.imshow(mask, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(result)
plt.show()

contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#cnts = imutils.grab_contours(contours)
#c = max(cnts, key=cv2.contourArea)
#print(contours[1])

#get biggest found area
biggest_area = 0
cnt_area_dict = []
for cnt in contours:
    area = cv2.contourArea(cnt)
    if(area >= biggest_area):
        cnt_area_dict = [area, cnt]
        biggest_area = area

if(cnt_area_dict[0] >= 50):
    #print(cnt_area_dict[1])
    approx = cv2.approxPolyDP(cnt_area_dict[1], 0.015 * cv2.arcLength(cnt_area_dict[1], True), True)
    print(approx)
    if(len(approx) <= 20):
        x,y,w,h = cv2.boundingRect(approx)
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        x,y,w,h = cv2.boundingRect(cnt_area_dict[1])
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        #cv2.drawContours(img, [approx], -1, (0, 0, 255), 2)

plt.imshow(img) 
plt.show()


#plt.imshow(img)
#plt.show(img.any())
#plt.pause(10)
#plt.close()

print("done")

#plt.waitforbuttonpress(0) 