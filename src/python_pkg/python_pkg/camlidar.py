#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化订阅者节点 - 用于显示perception_node检测到的锥桶
- 订阅/target话题
- 使用OpenCV可视化锥桶位置和目标点
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import Float32MultiArray  # 目标点消息类型

import numpy as np                      # Python数值计算库
import cv2                              # OpenCV图像处理库
import math                             # 数学函数库

class CamLidarVisualizer(Node):
    """可视化订阅者节点 - 显示锥桶和目标点"""
    
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.get_logger().info("CamLidar visualizer initialized")  # 日志输出
        
        # 创建订阅者，订阅/target话题
        self.sub = self.create_subscription(
            Float32MultiArray, '/target', self.listener_callback, 10)
        
        # 初始化可视化参数
        self.img_size = 500                # 图像大小
        self.scale_factor = 100.0          # 缩放因子（米到像素的转换）
        self.center_offset = self.img_size // 2  # 中心点偏移
        
        # 创建背景图像
        self.background = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        
        # 绘制网格线
        self.draw_grid()
    
    def draw_grid(self):
        """绘制背景网格"""
        # 绘制中心十字线
        cv2.line(self.background, (0, self.center_offset), 
                (self.img_size, self.center_offset), (50, 50, 50), 1)
        cv2.line(self.background, (self.center_offset, 0), 
                (self.center_offset, self.img_size), (50, 50, 50), 1)
        
        # 绘制同心圆（每米一个圆）
        for r in range(1, 6):
            radius = int(r * self.scale_factor)
            cv2.circle(self.background, (self.center_offset, self.center_offset), 
                      radius, (30, 30, 30), 1)
    
    def world_to_image(self, x, y):
        """将世界坐标转换为图像坐标"""
        img_x = int(self.center_offset - y * self.scale_factor)  # 注意坐标系转换
        img_y = int(self.center_offset - x * self.scale_factor)
        return img_x, img_y
    
    def listener_callback(self, msg):
        """订阅回调函数"""
        # 创建可视化图像（复制背景）
        img = self.background.copy()
        
        # 解析消息数据
        data = msg.data
        if len(data) < 2:
            self.get_logger().warn("接收到的数据不完整")
            return
        
        # 获取目标点
        target_x = data[0]
        target_y = data[1]
        
        # 获取内外道锥桶数量和坐标
        if len(data) >= 3:
            cone_count = int(data[2])
            
            # 绘制内道锥桶（红色）
            inner_count = int((len(data) - 3) // 2)
            for i in range(min(inner_count, cone_count)):
                idx = 3 + i * 2
                if idx + 1 < len(data):
                    x, y = data[idx], data[idx + 1]
                    img_x, img_y = self.world_to_image(x, y)
                    cv2.circle(img, (img_x, img_y), 5, (0, 0, 255), -1)  # 红色实心圆
            
            # 绘制外道锥桶（蓝色）
            outer_start = 3 + cone_count * 2
            if outer_start < len(data):
                outer_count = (len(data) - outer_start) // 2
                for i in range(outer_count):
                    idx = outer_start + i * 2
                    if idx + 1 < len(data):
                        x, y = data[idx], data[idx + 1]
                        img_x, img_y = self.world_to_image(x, y)
                        cv2.circle(img, (img_x, img_y), 5, (255, 0, 0), -1)  # 蓝色实心圆
        
        # 绘制目标点（绿色）
        if target_x > -1000 and target_y > -1000:  # 有效目标点
            target_img_x, target_img_y = self.world_to_image(target_x, target_y)
            cv2.circle(img, (target_img_x, target_img_y), 7, (0, 255, 0), -1)  # 绿色实心圆
            cv2.putText(img, f"Target: ({target_x:.2f}, {target_y:.2f})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # 绘制车辆位置（白色）
        cv2.circle(img, (self.center_offset, self.center_offset), 8, (255, 255, 255), 2)
        
        # 显示图像
        cv2.imshow('CamLidar Visualization', img)
        cv2.waitKey(30)  # 30ms刷新

def main(args=None):
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = CamLidarVisualizer("camlidar_node")              # 创建节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    
    # 清理资源
    cv2.destroyAllWindows()
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()