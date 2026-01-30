#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库

def record_video(output_file, duration, fps = 10, width = 640, height = 480):
    """
    录制视频并保存为MP4格式

    参数:
        output_file (str): 输出视频文件名（例如 "output.mp4"）
        duration (int): 录制时长（秒）
        fps (int): 视频帧率，默认为10
        width (int): 视频宽度，默认为640
        height (int): 视频高度，默认为480
    """
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 设置视频编码器和输出文件
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4视频编码器
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    print(f"开始录制视频，时长为{duration}秒，保存为{output_file}")
    start_time = cv2.getTickCount()  # 获取开始时间

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break
        
        # 写入帧到视频文件
        out.write(frame)
        
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("视频录制完成")

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    record_video('output.mp4',3000)
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
