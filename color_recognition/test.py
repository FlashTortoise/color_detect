# -*- coding: utf-8 -*-
# color.py
# read pictures
import numpy as np
import cv2
import matplotlib.pyplot as plt
import color_detect as ct

im = cv2.imread('D:/TDPS/data/color_path1/60.jpg')
# from RGB to HSV
im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

# define the boundaries of red, blue and green
boundaries_red = [([125, 43, 46], [153, 255, 255])]
boundaries_blue = [([100, 43, 46], [124, 255, 255])]
boundaries_green = [([35, 43, 46], [77, 255, 255])]


# find the red object
'''for (lower, upper) in boundaries_red:
    lower = np.array(lower)
    upper = np.array(upper)
    mask_red = cv2.inRange(im_hsv, lower, upper)
    im_object_red = cv2.bitwise_and(im_hsv, im_hsv, mask=mask_red)
# find the blue object
for (lower, upper) in boundaries_blue:
    lower = np.array(lower)
    upper = np.array(upper)
    mask_blue = cv2.inRange(im_hsv, lower, upper)
    im_object_blue = cv2.bitwise_and(im_hsv, im_hsv, mask=mask_blue)
# find the green object
for (lower, upper) in boundaries_green:
    lower = np.array(lower)
    upper = np.array(upper)
    mask_green = cv2.inRange(im_hsv, lower, upper)
    im_object_green = cv2.bitwise_and(im_hsv, im_hsv, mask=mask_green)
# find the mask sum of red, blue and green
mask_1 = cv2.add(mask_red, mask_blue)
mask = cv2.add(mask_1, mask_green)
# find the object sum of red, blue and green
im_object_1 = cv2.add(im_object_red, im_object_blue)
im_object = cv2.add(im_object_1, im_object_green)'''

mask, im_object, contours = ct.multi_color_track(
    im_hsv, boundaries_red, boundaries_green, boundaries_blue)
# find coutours
'''im_gray = cv2.cvtColor(im_object, cv2.COLOR_BGR2GRAY)
ret, binary = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY)
_, contours, _ = cv2.findContours(
    binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)'''
# draw contours
#im_contours = cv2.drawContours(im_object, contours[0], -1, (0, 255, 0), 3)
# plt.imshow(im_contours)
# plt.show()
# find the center
# first compute moments


'''def color_location(countours):
    area_list = np.zeros(len(contours))
    i = 0

    for cnt in contours:
        area_list[i] = cv2.contourArea(cnt)
        i = i + 1

    max_ind = np.argmax(area_list)
    max_area = area_list[max_ind]

    # computer the center
    M = cv2.moments(countours[max_ind])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    max_location = (cx, cy)

    return(max_location, max_ind, max_area, countours[max_ind])'''
location, _, area, contours_find = ct.color_location(contours)
print area

im_object = cv2.cvtColor(im_object, cv2.COLOR_HSV2BGR)
im_contours = cv2.drawContours(
    im_object, [contours[0]], -1, (0, 0, 255), thickness=cv2.FILLED)

# cv2.imshow('object', im)
cv2.circle(im_contours, location, 63, (255, 0, 0))
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image', im_contours)

while cv2.waitKey(0) != 27:
    pass
cv2.destroyAllWindows()
# plt.plot(cx,cy,'b.')
# plt.show()
