#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import LaserScan   # 激光雷达消息类型

import numpy as np                      # Python数值计算库
import cv2
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import math
"""
创建一个订阅者节点
"""
class LaserScanSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.listener_callback, 5)       # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
    
    def listener_callback(self, laser):
        i = 0
        j = 0
        pp = np.zeros((30,2))
        pr = np.zeros((30,2))
        
        for i in range(1, len(laser.ranges)):
            if laser.ranges[i-1] - laser.ranges[i] >= 2.0:
                if laser.ranges[i] < 2.5:
                    pp[j][0] = laser.ranges[i]
                    pp[j][1] = i*laser.angle_increment + laser.angle_min #获得2.5米范围内各锥桶的极坐标
                    j += 1
        for i in range(j):
            pr[i][0] = -pp[i][0] * math.sin(pp[i][1])
            pr[i][1] = pp[i][0] * math.cos(pp[i][1]) #获得2.5米范围内各锥桶的直角坐标(激光雷达为原点)
            
        img = np.zeros((50,50,3),np.uint8)
        
        pb = pr
        
        for i in range(j):
            pb[i][0] = 10.0 * pr[i][0] + 25.0
            pb[i][1] = 10.0 * pr[i][1] + 25.0

        #将该矩阵，第一个像素点的值设置为白色
        for i in range(30):
            img[int(pb[i][0]), int(pb[i][1])] = (255, 255, 255)

        #将Numpy矩阵转换成OpenCV图像
        img_cv = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #设置卷积核
        kernel = np.ones((3,3),np.uint8)
        #膨胀
        img_cv = cv2.dilate(img_cv,kernel,iterations = 1)
        #放大图像
        img_cv = cv2.resize(img_cv, (500,500), interpolation = cv2.INTER_LINEAR)
        #显示OpenCV图像
        cv2.imshow('image', img_cv)
        cv2.waitKey(30)

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = LaserScanSubscriber("scan")                      # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口