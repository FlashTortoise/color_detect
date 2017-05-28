import numpy as np
import cv2
# import matplotlib.pyplot as plt


def color_track(im_hsv, boundaries):
        # im_hsv is the image which has been transformed into hsv format
        # find the specified color object
    for (lower, upper) in boundaries:
        lower = np.array(lower)
        upper = np.array(upper)
        mask = cv2.inRange(im_hsv, lower, upper)
        im_object = cv2.bitwise_and(im_hsv, im_hsv, mask=mask)
    # find coutours
    im_gray = cv2.cvtColor(im_object, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return (mask, im_object, contours)


def multi_color_track(im_hsv, boundaries_red, boundaries_green, boundaries_blue):
    # find three color in the img
    (mask_red, im_object_red, _) = color_track(im_hsv, boundaries_red)
    (mask_green, im_object_green, _) = color_track(im_hsv, boundaries_green)
    (mask_blue, im_object_blue, _) = color_track(im_hsv, boundaries_blue)
    mask = cv2.add(
        cv2.add(mask_red, mask_blue), mask_green)

    im_object = cv2.add(
        cv2.add(im_object_red, im_object_blue), im_object_green)

    im_gray = cv2.cvtColor(im_object, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(im_gray, 100, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return(mask, im_object, contours)

def find_max_area(contours):

    area_list = np.zeros(len(contours))
    i = 0
    if len(contours) > 0:
        for cnt in contours:
            area_list[i] = cv2.contourArea(cnt)
            i = i + 1

        max_ind = np.argmax(area_list)
        max_area = area_list[max_ind]

        # computer the center
        M = cv2.moments(contours[max_ind])
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        max_location = (cx, cy)
        return (max_location, max_ind, max_area, contours[max_ind])
    else:
        return((320,240),0,0,contours)



def color_location(im_hsv, contours):

    max_cx = np.shape(im_hsv)[1]
    max_cy = np.shape(im_hsv)[0]

    max_location, max_ind,max_area, _ =find_max_area(contours)
    cx=max_location[0]
    cy=max_location[1]
    cx_ratio = float(cx)/max_cx
    cy_ratio = float(cy)/max_cy
    if max_area !=0:
        while ( (cx_ratio < 0.1) or (cx_ratio > 0.9) or (cy_ratio < 0.1) ) and max_area >50:
            contours = np.delete(contours,max_ind)
            max_location, max_ind, max_area, _ = find_max_area(contours)
            cx = max_location[0]
            cy = max_location[1]
            cx_ratio = float(cx) / max_cx
            cy_ratio = float(cy) / max_cy
            return(max_location,  max_area, contours[max_ind])
    else:
        return (max_location, max_area,contours)


def get_color(contours, im_hsv, boundaries_red, boundaries_green, boundaries_blue):
    # get the color boundary of the location pixel
    #location_color = im_hsv[location[1], location[0]]

    h=np.delete(im_hsv,[1,2],2)
    mask=np.zeros(h.shape,np.uint8)
    cv2.drawContours(mask,[contours],0,255,-1)
    location_color=cv2.mean(im_hsv,mask=mask.astype(np.uint8))

    for (lower, upper) in boundaries_red:
        lower = np.array(lower)
        upper = np.array(upper)
        if (location_color[0:3] > lower).all() and (location_color[0:3] < upper).all():
            return boundaries_red

    for (lower, upper) in boundaries_green:
        lower = np.array(lower)
        upper = np.array(upper)
        if (location_color[0:3] > lower).all() and (location_color[0:3] < upper).all():
            return boundaries_green

    for (lower, upper) in boundaries_blue:
        lower = np.array(lower)
        upper = np.array(upper)
        if (location_color [0:3]> lower).all() and (location_color[0:3] < upper).all():
            return boundaries_blue

    print 'error: the detected color is not in the boundary '
    return [(0,0,0),(0,0,0)]


def stop_check(area):

    global area_list
    if 'area_list' not in globals():
        area_list = list()
    area_list.append(area)
    if len(area_list)>5:
        grad1 = np.gradient(area_list, 1)

        if grad1[len(grad1) - 1] < 0:
            area_max = max(area_list)
            threshold_area = area_max/10
            if area_list[len(grad1) - 1] < threshold_area:
                return 1
            else:
                return 0
        else:
            return 0
    elif len(area_list)>100:
        area_list=[]
        return 0

    else:
        return 0
