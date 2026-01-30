#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from geometry_msgs.msg import Twist     # 阿克曼底盘消息类型
import numpy as np                      # Python数值计算库

"""
创建一个发布者节点
"""
class AckermanPublisher(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.pub = self.create_publisher(Twist, 'teleop_cmd_vel', 10)       # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.1, self.timer_callback)    # 创建定时器以0.01秒为间隔执行回调函数
    
    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 1500.0
        twist.angular.z = 90.0
        self.get_logger().info('Publishing: speed %d' % twist.angular.x)
        self.pub.publish(twist)

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = AckermanPublisher("ackerman")                    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口