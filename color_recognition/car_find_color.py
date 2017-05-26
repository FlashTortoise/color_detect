# -*- coding: utf-8 -*-
import glob
import os
from collections import namedtuple

import cv2
import numpy as np
import tortoise as t
from tortoise import config
from tortoise import p, Task, Tortoise
import color_detect as ct

SIGHT_PIXEL_COUNT = (config.EYE_SIGHT_HEIGHT * config.EYE_SIGHT_WIDTH) # 像素的个数
EXPECTED_BLOCK_AREA = 20000
HALF_WIDTH = config.EYE_SIGHT_WIDTH / 2
#
# def imgs(path):
#     path = os.path.expanduser(path)
#
#     for pic_path in glob.iglob(path):
#         yield cv2.imread(pic_path)
#
# t.update_config(
#      EYE_SIMULATOR_ACTIVE=True,
#      EYE_SIMULATOR_DATASET=imgs(r"D:\TDPS\data\color_test/*.JPG")
#  )

# configure the walk period time
t.update_config(TORTOISE_WALK_PERIOD=0.1)

eye = t.peripheral.eye


class ColorTracingTask(t.Task):

    def __init__(self):
        super(ColorTracingTask, self).__init__()

        self.find_first_task = FindBlockTask() #找到第一个色块的任务
        self.find_second_task = FindBlockTask() #找到第二个色块的任务
        self.turning = Turning() #定角度转弯的任务
        self.turn_over = False  #显示定角度转弯的任务是否完成

    def step(self):
        if self.find_first_task.stop == 0: #如果第一个任务没有停止，则一直运行第一个任务
            self.find_first_task.step()
            print 'speed: %5.3f direction: %8.1f' % (
                self.find_first_task.speed, self.find_first_task.direction)
        else:#如果第一个任务停止了，则运行第二个任务
            print 'second task'
            self.find_second_task.color = self.find_first_task.color
            #把第一个任务找到的色块的颜色赋给第二个任务
            self.find_second_task.if_color_found = True
            # 把第二个任务中'是否找到色块颜色'标志设为True
            boundaries_red = [([125, 80, 46], [153, 255, 255])]
            boundaries_blue = [([100, 100, 46], [124, 255, 255])]
            boundaries_green = [([20, 60, 46], [46, 255, 255])]
            if self.find_second_task.color == boundaries_red:
                # 如果色块为红色
                # 逆时针旋转45度
                self.turning.want_degree = 45
                if self.turn_over == False:
                    while self.turn_over == False:
                        self.turn_over = self.turning.step()

                self.find_second_task.step()#运行找色块任务


            if self.find_second_task.color == boundaries_blue:
                # 如果色块为蓝色
                # 顺时针旋转135度
                self.turning.want_degree = -135
                if self.turn_over == False:
                    while self.turn_over == False:
                        self.turn_over = self.turning.step()
                self.find_second_task.step()#运行找色块任务

            if self.find_second_task.color == boundaries_green:
                # 如果色块为绿色
                # 逆时针旋转90度
                self.turning.want_degree = 90
                if self.turn_over == False:
                    while self.turn_over == False:
                        self.turn_over = self.turning.step()
                    return

                self.find_second_task.step()#运行找色块任务
            print 'speed: %5.3f direction: %8.1f' % (
                self.find_second_task.speed, self.find_second_task.direction)



class FindBlockTask(t.Task):

    def __init__(self):
        super(FindBlockTask, self).__init__()

        self.found = False
        self.speed = 0
        self.direction = 0
        self.stop = 0 # judge if the car should stop


        self.rec = t.get_recorder('block_following')
        self.color_index = 0 # indicate the number of color in the boundary mat
        # color index is going to denote the times that the color is recorded
        self.if_color_found = False # indicate whether the color is found
        self.color = ([0, 0, 0], [0, 0, 0]) # initiate the found color
        self.boundary_mat=[] # used to save the color boundary

    def step(self):


        im = eye.see() # get the img from camera
        im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV) # convert BGR to HSV
        boundaries_red = [([125, 80, 46], [153, 255, 255])]
        boundaries_blue = [([100, 100, 46], [124, 255, 255])]
        boundaries_green = [([20, 60, 46], [46, 255, 255])]

        if self. if_color_found == False:
            # if the color not found, track three colors in the img
            _, im_object, contours = ct.multi_color_track(
                im_hsv, boundaries_red, boundaries_green, boundaries_blue)

        else:
            # if the color is found, only track one color in the img
            _, im_object, contours = ct.color_track(im_hsv, self.color)
            print 'color found'

        location, area, contours_find = ct.color_location(im_hsv, contours)
        print len(contours_find)
        color_boundary = ct.get_color(contours_find, im_hsv, boundaries_red, boundaries_green, boundaries_blue)
        print 'area is',area
        # find the maximum area in the contours and return the location
        # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # cv2.imshow('image', im_object)
        #
        # while cv2.waitKey(0) != 27:
        #     pass
        # cv2.destroyAllWindows()
        # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # cv2.imshow('image', im)
        # while cv2.waitKey(0) != 27:
        #     pass
        # cv2.destroyAllWindows()
        if self.stop == 1:
            print 'toroise stop'
            self.speed = 0
            self.direction = 0
            t.peripheral.wheels.set_diff(self.speed, -self.direction)
            return


        else:
            if self.found == False:

                if area<100:
                    print 'area too small, color not found'
                    self.set_not_found()
                    t.peripheral.wheels.set_diff(self.speed, -self.direction)
                    return
                else:
                    self.found = True

            else:
                self.stop = ct.stop_check(area)
                print self.color
                # used to find the color block and judge whether the color block is found
                if self.if_color_found == False:
                    self.color = self. color_check(color_boundary)



                if self.stop == 1:
                    # case: the car should stop since it arrive the block
                    print 'tortoise stop'
                    self.speed = 0
                    self.direction = 0
                    t.peripheral.wheels.set_diff(self.speed, self.direction)
                    return

                else:
                    t.peripheral.wheels.set_diff(self.speed, self.direction)
                    if location[0] < config.EYE_SIGHT_WIDTH * 0.3:

                        # case: the car should turn big left
                        print 'turn big left'
                        self.speed = 0
                        self.direction = -0.5
                        return

                    elif location[1] > config.EYE_SIGHT_WIDTH * 0.7:
                        # case: the car should turn right
                        print 'trun big right '
                        self.speed = 0
                        self.direction = 0.5
                        return

                    else:

                        self. speed = 0.3
                        self.direction = float(
                            (location[0] - HALF_WIDTH))*0.5 / HALF_WIDTH*0.7
                        print  float((location[0] - HALF_WIDTH)) / HALF_WIDTH
                        return




    def set_not_found(self):
        self.found = False
        self.direction = 0
        self.speed = 0.3

    def color_check(self, area):

        if self.color_index == 0:
            self.boundary_mat = []

        if self.color_index >= 5:

            a_01 = self.boundary_mat[0] == self.boundary_mat[1]
            a_23 = self.boundary_mat[2] == self.boundary_mat[3]
            a_02 = self.boundary_mat[0] == self.boundary_mat[2]
            a_04 = self.boundary_mat[0] == self.boundary_mat[4]
            self.color_index = 0

            if a_01 == 1 and a_23 == 1 and a_02 == 1 and a_04 == 1:
                self.if_color_found = True
                return self.boundary_mat[0]
            else:
                self.if_color_found = False
                return self.boundary_mat[0]

        else:
            self.boundary_mat.append(area)
            self.color_index = self.color_index + 1
            self.if_color_found = False
            return self.boundary_mat[0]

class PIDController(object):
    def __init__(self, kp, ki, kd):
        self._kp, self._ki, self._kd = kp, ki, kd

        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd
        self._a2 = self._kd

        self.lx = 0
        self.llx = 0
        self.ly = 0

    def run(self, x):
        y = self.ly + self._a0 * x + self._a1 * self.lx + self._a2 * self.llx
        self.ly, self.lx, self.llx = y, x, self.lx
        return y

    @property
    def kp(self):
        return self._kp

    @property
    def ki(self):
        return self._ki

    @property
    def kd(self):
        return self._kd

    @kp.setter
    def kp(self, value):
        self._kp = value
        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd

    @ki.setter
    def ki(self, value):
        self._ki = value
        self._a0 = self._kp + self._ki + self._kd

    @kd.setter
    def kd(self, value):
        self._kd = value
        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd
        self._a2 = self._kd


def constrain(a, l, u):
    if a > u:
        return u
    elif a < l:
        return l
    else:
        return a

class Turning(Task):
    def __init__(self):
        super(Turning, self).__init__()
        self.c = PIDController(0.04, 0, 0)
        self.target_yaw = 0
        self.want_degree = 0

    def step(self):
        if self.target_yaw==0:
            self.target_yaw = p.yaw.get()+self.want_degree

        deg = p.yaw.get()
        diff = constrain(self.c.run(self.target_yaw - deg), 0.4, -0.4)

        print deg, diff
        p.wheels.set_diff(speed=0, diff=diff)
        print 'deg is',deg
        print 'target_yaw is ', self.target_yaw

        return abs(self.target_yaw - deg)<5


if __name__ == '__main__':
    tttt = t.Tortoise()
    tttt.task = ColorTracingTask()
    tttt.walk()