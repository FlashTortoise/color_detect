# -*- coding: utf-8 -*-
import glob
import os
from collections import namedtuple

import cv2
import numpy as np
import tortoise as t
from tortoise import config

#import color_detect as ct

SIGHT_PIXEL_COUNT = (config.EYE_SIGHT_HEIGHT * config.EYE_SIGHT_WIDTH)
EXPECTED_BLOCK_AREA = 20000
HALF_WIDTH = config.EYE_SIGHT_WIDTH / 2

# def imgs(path):
#     path = os.path.expanduser(path)
#
#     for pic_path in glob.iglob(path):
#         yield cv2.imread(pic_path)
#
# t.update_config(
#      EYE_SIMULATOR_ACTIVE=True,
#      EYE_SIMULATOR_DATASET=imgs(r"D:/TDPS/bridge detect/align_data/*.JPG")
#  )
#
# # configure the walk period time
t.update_config(TORTOISE_WALK_PERIOD=0.1)

eye = t.peripheral.eye

class GoOnBridgeTask(t.Task):

    def __init__(self):
        super(GoOnBridgeTask, self).__init__()

        self.find_task = FindBridgeTask()

    def step(self):
        self.find_task.step()

        print 'speed: %5.3f direction: %8.1f' % (
            self.find_task.speed, self.find_task.direction)


class FindBridgeTask(t.Task):

    def __init__(self):
        super(FindBridgeTask, self).__init__()

        self.found = False
        self.speed = 0
        self.direction = 0
        self.stop = 0 # judge if the car should stop
        self.if_align = False #小车是否和桥对齐
        self.align_indicator_mat = []
        self.align_index = 0

    def step(self):

        image = eye.see() # get the img from camera
        im_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(im_gray,200,255) # 截取灰度图中特别亮的部分
        im_object = cv2.bitwise_and(image,image,mask=mask) # 截取灰度图中特别亮的部分
        location,max_contours=bridge_location(im_gray)
        degree, im_object= bridge_degree_alian(im_object, 1)
        location = float(location[0]) / np.shape(im_object)[1]
        # print 'degree is',degree
        # print 'location is ', location
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', im_object)

        while cv2.waitKey(0) != 27:
            pass
        cv2.destroyAllWindows()

        if self.if_align == False:
            self.if_align = self.align_check(location,degree)
            if (degree > -2 and degree < 2) and (location >0.4 and location < 0.6):
                self.speed = 0.1
                self.direction = 0
                t.peripheral.wheels.set_diff(self.speed, -self.direction)
                return
            elif degree < -2:
                self.speed = 0.1
                self.direction = 0.3
                t.peripheral.wheels.set_diff(self.speed, -self.direction)
                return
            elif degree > 2:
                self.speed = 0.1
                self.direction = -0.3
                t.peripheral.wheels.set_diff(self.speed, -self.direction)
                return

        elif  self.if_align == True:
            print 'bridge is aligned'
            self.speed = 0.5
            self.direction = float(location - 0.5)/0.5
            t.peripheral.wheels.set_diff(self.speed, -self.direction)
            return

    def align_check(self, location, degree):

        align_indicator = (degree > -2 and degree < 2) and (location >0.4 and location < 0.6)
        if align_indicator == 1:
            print 'align'
        if self.align_index == 0:
            self.align_indicator_mat = []

        if self.align_index >= 5:

            a_01 = self.align_indicator_mat[0] == self.align_indicator_mat[1]
            a_23 = self.align_indicator_mat[2] == self.align_indicator_mat[3]
            a_02 = self.align_indicator_mat[0] == self.align_indicator_mat[2]
            a_04 = self.align_indicator_mat[0] == self.align_indicator_mat[4]
            self.align_index = 0

            if a_01 == 1 and a_23 == 1 and a_02 == 1 and a_04 == 1:
                return True
            else:
                return False

        else:
            self.align_indicator_mat.append(align_indicator)
            self.align_index = self.align_index + 1
            return False



def bridge_degree_alian(image, plot_switch):
# image 是图像，plot_switch 用来指示是否需要在原图上画出检测结果
# 通过检测横向的线实现对齐，返回小车与桥的角度差


    edges = cv2.Canny(image, 50, 500) # Canny滤波
    lines = cv2.HoughLinesP(edges.astype(np.uint8), 1, np.pi / 180, threshold=10, minLineLength=45, maxLineGap=3)
    # 霍夫线变换
    alian_line_angle_mat=[] #声明一个矩阵，用来装每条线的角度
 
    if len(lines) != 0:
        for line in lines: #用for循环检查找到的每条线，看其是否满足条件
            pt1 = (line[0][0], line[0][1])
            pt2 = (line[0][2], line[0][3])
            # 上面是霍夫线变换找到的线对应的两个顶点
            if pt1[0]!=pt2[0]:
                rho = np.tanh(float(pt2[1] - pt1[1]) / (pt1[0] - pt2[0]))
            else:
                rho=np.pi/2
            # rho 是每条线的角度
            if (rho < np.pi/6 and rho>=0 ) or (rho> -np.pi/6 and rho<=0): # 对角度进行筛选

                degree = float(rho) / np.pi * 180 # 把弧度制转换成角度制
                alian_line_angle_mat.append(degree) # 如果角度满足要求，就把角度存进数组里

                if plot_switch == 1: # 画图开关，如果开，在原图上划线
                    #cv2.circle(image, pt1, 10, (0, 255, 0), -1)
                    #cv2.circle(image, pt2, 10, (0, 0, 255), -1)
                    cv2.line(image, pt1, pt2, (255, 0, 0), 2)

       
        if len(alian_line_angle_mat) != 0:
            # 如果角度数组里有值，则求平均值
            alian_line_degree = np.average(np.array(alian_line_angle_mat))
        else:
            # 如果角度数组里没值，则设角度为0
            print 'line detect failed'
            alian_line_degree = 0

        return alian_line_degree, image
    else:
        print 'line detect failed'
        return 0 ,image


def find_max_area(contours):
# 这个函数的作用是输入检测到的所有的轮廓
# 输出其中面积最大的轮廓对应轮廓的中心，轮廓的下标，最大的面积，和 轮廓的数组

    area_list = np.zeros(len(contours))
    i = 0

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



def color_location(im_hsv, contours):
# 这个函数也是用来检测面积最大的轮廓的，和 find_max_area函数相比加了去除边角轮廓的功能
# 以去除天空的影响
    max_cx = np.shape(im_hsv)[1]
    max_cy = np.shape(im_hsv)[0]

    max_location, max_ind,max_area, _ =find_max_area(contours)
    cx=max_location[0]
    cy=max_location[1]
    cx_ratio = float(cx)/max_cx
    cy_ratio = float(cy)/max_cy
    while ( (cx_ratio < 0.1) or (cx_ratio > 0.9) or (cy_ratio < 0.1) ) and max_area >50:
        contours = np.delete(contours,max_ind)
        max_location, max_ind, max_area, _ = find_max_area(contours)
        cx = max_location[0]
        cy = max_location[1]
        cx_ratio = float(cx) / max_cx
        cy_ratio = float(cy) / max_cy
    return(max_location,  max_area, contours[max_ind])


def bridge_location(im_gray):
    # 返回桥的面积，以及桥对应的轮廓
    _, binary = cv2.threshold(im_gray, 200, 255, cv2.THRESH_BINARY)

    _, contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    location, _, max_contours = color_location(im_gray, contours)
    return location,max_contours


if __name__ == '__main__':
    tttt = t.Tortoise()
    tttt.task = GoOnBridgeTask()
    tttt.walk()