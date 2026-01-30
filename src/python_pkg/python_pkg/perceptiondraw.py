#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯感知节点 - 基于NCSC2024源程序main.py
- 激光雷达锥桶检测与聚类
- 内外道分离与目标点计算
- 发布目标点话题供控制节点使用
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
from sklearn.cluster import KMeans
import math
import copy
import time

import cv2
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类

class PerceptionNode(Node):
    """纯感知节点 - 锥桶检测与目标点计算"""
    
    def __init__(self):
        super().__init__('perception_node')
        
        # --- 声明参数（基于源程序main.py）---
        # 激光雷达滤波参数
        self.declare_parameter('min_range', 0.3)          # 最小有效极径
        self.declare_parameter('max_range', 2.5)          # 最大有效极径
        self.declare_parameter('left_width', 2.5)         # 矩形滤波左侧宽度
        self.declare_parameter('right_width', 2.5)        # 矩形滤波右侧宽度
        self.declare_parameter('forward_length', 2.8)     # 矩形滤波前方长度
        self.declare_parameter('back_length', 0.0)        # 矩形滤波后方长度
        
        # KMeans聚类参数
        self.declare_parameter('n_clusters', 15)          # 聚类个数
        self.declare_parameter('dedup_radius', 0.25)      # 中心点去重半径
        
        # 内外道分离参数
        self.declare_parameter('inner_dist_sq', 1.96)     # 内道扩展距离平方（1.4^2）
        self.declare_parameter('cut_length_y', 0.7)       # 削除圆的直边到圆心Y距离
        self.declare_parameter('cut_length_x', -0.7)      # 削除圆的直边到圆心X距离
        
        # 圆形滤波参数
        self.declare_parameter('max_radius', 6.0)         # 圆形滤波最大半径
        self.declare_parameter('max_y_filter', 0.5)       # 外道Y坐标过滤阈值
        
        # 目标点选择参数
        self.declare_parameter('target_jump', 4)          # 目标点选择跳数（第4个点）
        
        # 获取参数
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.left_width = self.get_parameter('left_width').value
        self.right_width = self.get_parameter('right_width').value
        self.forward_length = self.get_parameter('forward_length').value
        self.back_length = self.get_parameter('back_length').value
        
        self.n_clusters = self.get_parameter('n_clusters').value
        self.dedup_radius = self.get_parameter('dedup_radius').value
        
        self.inner_dist_sq = self.get_parameter('inner_dist_sq').value
        self.cut_length_y = self.get_parameter('cut_length_y').value
        self.cut_length_x = self.get_parameter('cut_length_x').value
        
        self.max_radius = self.get_parameter('max_radius').value
        self.max_y_filter = self.get_parameter('max_y_filter').value
        self.target_jump = self.get_parameter('target_jump').value
        
        # --- 状态变量 ---
        self.target_x = -10000.0
        self.target_y = -10000.0
        self.inner_x = []
        self.inner_y = []
        self.outer_x = []
        self.outer_y = []
        self.cone_count = 0
        # 目标像素Y趋势记录（用于绘制变化趋势图）
        self.target_pixel_y_history = []
        self.trend_max_len = 200  # 趋势显示的最大点数
        
        # --- QoS配置 ---
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- ROS2 订阅与发布 ---
        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 发布目标点（基于源程序main.py的Float32MultiArray格式）
        self.target_pub = self.create_publisher(Float32MultiArray, '/target', 10)
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("纯感知节点已启动（基于NCSC2024源程序）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"激光雷达滤波: 距离[{self.min_range}, {self.max_range}]m")
        self.get_logger().info(f"矩形滤波: 左{self.left_width}m, 右{self.right_width}m, 前{self.forward_length}m")
        self.get_logger().info(f"KMeans聚类: {self.n_clusters}个簇, 去重半径{self.dedup_radius}m")
        self.get_logger().info(f"内外道分离: 内道扩展距离2={self.inner_dist_sq}")
        self.get_logger().info(f"圆形滤波: 最大半径={self.max_radius}m, Y过滤={self.max_y_filter}m")
        self.get_logger().info(f"目标点选择: 第{self.target_jump}个最近点")
        self.get_logger().info("=" * 60)
    
    def correct_center(self, centers):
        """
        中心点去重（基于源程序main.py的correctCenter函数）
        输入中心点列表，输出去重后的中心点列表
        """
        dedup = []
        not_processed = [i for i in range(len(centers))]
        
        for i in range(len(centers)):
            if i in not_processed:
                not_processed.remove(i)
                dedup_x = []
                dedup_y = []
                
                for j in range(len(centers)):
                    if j in not_processed:
                        if math.dist(centers[i], centers[j]) > self.dedup_radius:
                            continue
                        not_processed.remove(j)
                        dedup_x.append(centers[j][0])
                        dedup_y.append(centers[j][1])
                
                x = round((sum(dedup_x) + centers[i][0]) / (len(dedup_x) + 1), 3)
                y = round((sum(dedup_y) + centers[i][1]) / (len(dedup_y) + 1), 3)
                if [x, y] not in dedup:
                    dedup.append([x, y])
        
        return dedup
    
    def euclidean_dist_sq(self, x1, y1, x2, y2):
        """计算欧氏距离平方（基于源程序main.py的euciDistSq2函数）"""
        return (x1 - x2) ** 2 + (y1 - y2) ** 2
    
    def fit_circle(self, x, y):
        """
        拟合圆形（基于源程序main.py的Fit_Circle函数）
        返回圆心(x, y)和半径
        """
        if len(x) < 3:
            return 0, 0, 0
        
        C = []
        A = []
        
        for i in range(len(x)):
            A.append([x[i], y[i], 1])
            C.append(x[i]**2 + y[i]**2)
        
        A = np.array(A)
        C = np.array(C)
        
        try:
            B = np.dot(np.dot(np.linalg.inv(np.dot(A.T, A)), A.T), C)
            center_x = B[0] / 2
            center_y = B[1] / 2
            radius = math.sqrt(B[2] + B[0]**2/4 + B[1]**2/4)
            return center_x, center_y, radius
        except:
            return 0, 0, 0
    
    def fit_circle_and_filter(self, inner_x, inner_y, out_x, out_y):
        """
        根据内道拟合的圆形来滤除错误外道（基于源程序main.py的fitCircleAndFilter函数）
        """
        if len(inner_x) <= 2:
            # 内道点小于等于两个则不做圆形滤波，仅过滤y过大的外道点
            threshold = 1
            indices = [i for i in range(len(out_x)) if out_y[i] <= threshold]
            real_out_x = [out_x[i] for i in indices]
            real_out_y = [out_y[i] for i in indices]
            return real_out_x, real_out_y
        
        # 根据圆形滤波过滤
        center_x, center_y, radius = self.fit_circle(inner_x, inner_y)
        
        if radius > self.max_radius:
            # 假如拟合半径大于最大半径，则认为是直道，仅对y过大的外道点滤波
            indices = [i for i in range(len(out_x)) if out_y[i] <= self.max_y_filter]
            real_out_x = [out_x[i] for i in indices]
            real_out_y = [out_y[i] for i in indices]
            return real_out_x, real_out_y
        
        # 计算原点与拟合圆的半径平方
        radius_sq = self.euclidean_dist_sq(center_x, center_y, 0, 0)
        flag = 1 if radius_sq >= radius**2 else 0  # 原点在拟合圆外/内
        
        real_out_x = []
        real_out_y = []
        
        for i in range(len(out_x)):
            current_r_sq = self.euclidean_dist_sq(out_x[i], out_y[i], center_x, center_y)
            extra_radius = 1
            
            if current_r_sq >= radius_sq and flag == 1:
                real_out_x.append(out_x[i])
                real_out_y.append(out_y[i])
            elif current_r_sq <= (radius_sq + extra_radius) and flag == 0:
                real_out_x.append(out_x[i])
                real_out_y.append(out_y[i])
        
        return real_out_x, real_out_y
    
    def gen_three_equinoxes(self, inner_x, inner_y, outer_x, outer_y):
        """
        生成三等分点（基于源程序main.py的gen3Equinoxes函数）
        为每个内道点找到相配对的外道点(最近欧氏距离),从而得到三等分点
        """
        res_x = []
        res_y = []
        ratio = 0.5
        
        if len(inner_x) == 0 or len(outer_x) == 0:
            return [], []
        
        for i in range(len(inner_x)):
            min_dist_sq = float('inf')
            min_index = -1
            
            for j in range(len(outer_x)):
                dist_sq = self.euclidean_dist_sq(inner_x[i], inner_y[i], outer_x[j], outer_y[j])
                if min_dist_sq > dist_sq:
                    min_index = j
                    min_dist_sq = dist_sq
            
            if min_index != -1:
                res_x.append(inner_x[i] + ratio * (outer_x[min_index] - inner_x[i]))
                res_y.append(inner_y[i] + ratio * (outer_y[min_index] - inner_y[i]))
        
        return res_x, res_y
    
    def get_target_point(self, target_group_x, target_group_y):
        """
        根据三等分点选择目标点（基于源程序main.py的getTargetPoint函数）
        """
        if len(target_group_x) == 0 or len(target_group_y) == 0:
            return -10000, -10000
        
        # 筛选出所有在前方的目标点
        indices = [i for i in range(len(target_group_x)) if target_group_x[i] >= 0]
        
        if len(indices) == 0:
            return -10000, -10000
        
        # 计算距离并排序
        dist = []
        for i in indices:
            dist.append(self.euclidean_dist_sq(target_group_x[i], target_group_y[i], 0, 0))
        
        # 快速排序
        sorted_indices = self.quick_sort(dist, list(range(len(dist))))
        
        # 得到目标点，如果点数小于跳数，则选择最后一个
        if len(sorted_indices) < self.target_jump:
            index = indices[sorted_indices[-1]]
        else:
            index = indices[sorted_indices[self.target_jump - 1]]
        
        return target_group_x[index], target_group_y[index]
    
    def quick_sort(self, arr, indices):
        """快速排序（基于源程序main.py的quick_sort函数）"""
        if len(indices) <= 1:
            return indices
        
        pivot = arr[indices[len(indices) // 2]]
        left = [i for i in indices if arr[i] < pivot]
        middle = [i for i in indices if arr[i] == pivot]
        right = [i for i in indices if arr[i] > pivot]
        
        return self.quick_sort(arr, left) + middle + self.quick_sort(arr, right)
    
    def separate_inner_outer_lanes(self, dedup_points):
        """
        内外道分离（基于源程序main.py的算法）
        先筛选出y>0即小车左侧的点，然后对左侧的点进行扩张，当扩张到长度不变时停止，其他点即为外道
        """
        inner_x = []
        inner_y = []
        outer_x = []
        outer_y = []
        
        if len(dedup_points) < 2:
            return inner_x, inner_y, outer_x, outer_y
        
        not_processed = [i for i in range(len(dedup_points))]
        inner_p = []  # 内圈点
        
        minimum_length_sq2 = 10000
        minimum_point = ()
        minimum_i = None
        
        # 先选出小车所有左侧且在一定范围内最近的点作为必定的内道点(y>0，-x1<x<x1)
        for i in range(len(dedup_points)):
            x1 = 1
            y1 = 0
            y2 = 2
            p = dedup_points[i]
            length_sq2 = self.euclidean_dist_sq(p[0], p[1], 0, 0)
            if (p[1] > y1 and p[0] > -x1 and p[0] < x1 and p[1] < y2 and 
                length_sq2 < minimum_length_sq2):
                minimum_point = p
                minimum_i = i
                minimum_length_sq2 = length_sq2
        
        if minimum_i is not None and minimum_point != ():
            inner_p.append(minimum_point)
            not_processed.remove(minimum_i)
        
        prev_length = 0
        current_length = len(inner_p)
        
        # 长度有改变，表示需要继续扩展
        while prev_length != current_length:
            prev_length = current_length
            ntp = copy.deepcopy(not_processed)
            inp = copy.deepcopy(inner_p)
            
            for i in not_processed:
                p = dedup_points[i]
                x1 = p[0]
                y1 = p[1]
                
                for j in range(len(inner_p)):
                    p1 = inner_p[j]
                    if (self.euclidean_dist_sq(x1, y1, p1[0], p1[1]) <= self.inner_dist_sq and 
                        not (p1[1] - y1 > self.cut_length_y and p1[0] - x1 > self.cut_length_x) and 
                        x1 - p1[0] > 0):
                        ntp.remove(i)
                        inp.append(p)
                        break
            
            not_processed = ntp
            inner_p = inp
            current_length = len(inner_p)
        
        # 提取内道点坐标
        for i in range(len(inner_p)):
            p = inner_p[i]
            inner_x.append(p[0])
            inner_y.append(p[1])
        
        # 剩余点作为外道
        for i in not_processed:
            p = dedup_points[i]
            outer_x.append(p[0])
            outer_y.append(p[1])
        
        return inner_x, inner_y, outer_x, outer_y
    
    def scan_callback(self, msg: LaserScan):
        """
        激光雷达回调 - 基于源程序main.py的callBack函数
        """
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            self.target_x = -20000.0
            self.target_y = -20000.0
            self.publish_target()
            return
        
        # === 数据预处理（基于源程序main.py）===
        decimal = 2  # 保留小数位
        min_R = self.min_range
        max_R = self.max_range
        left_width = self.left_width
        right_width = self.right_width
        forward_length = self.forward_length
        back_length = self.back_length
        
        # 极坐标转笛卡尔坐标
        i = np.arange(len(ranges))
        angles = msg.angle_min + i * msg.angle_increment
        valid_i = (ranges > min_R) & (ranges < max_R)
        valid_angles = angles[valid_i]
        valid_ranges = ranges[valid_i]
        
        x = np.round(np.round(valid_ranges, decimal) * np.cos(valid_angles), decimal)
        y = np.round(np.round(valid_ranges, decimal) * np.sin(valid_angles), decimal)
        
        # 矩形滤波
        valid_i = (x > -back_length) & (x < forward_length) & (y > -left_width) & (y < right_width)
        valid_x = x[valid_i]
        valid_y = y[valid_i]
        points = np.column_stack((valid_x, valid_y))
        
        # === KMeans聚类（基于源程序main.py）===
        if self.n_clusters > len(points):
            self.target_x = -10000.0
            self.target_y = -10000.0
            self.inner_x = []
            self.inner_y = []
            self.outer_x = []
            self.outer_y = []
            self.cone_count = 0
        else:
            cluster = KMeans(n_clusters=self.n_clusters, random_state=0, n_init="auto", init="k-means++")
            cluster.fit(points)
            dedup_points = self.correct_center(cluster.cluster_centers_.tolist())
            
            # 记录锥桶点数量
            self.cone_count = len(dedup_points)
            
            if len(dedup_points) >= 2:
                # === 内外道分离（基于源程序main.py）===
                self.inner_x, self.inner_y, self.outer_x, self.outer_y = self.separate_inner_outer_lanes(dedup_points)
                
                # 检查是否需要圆形滤波
                flag_filter = 0
                for y_val in self.outer_y:
                    if y_val > 1:
                        flag_filter = 1
                        break
                
                if len(self.inner_x) <= 2:
                    # 内道点小于等于两个则不做圆形滤波，仅仅过滤y过大的外道点
                    threshold = 1
                    indices = [i for i in range(len(self.outer_x)) if self.outer_y[i] <= threshold]
                    real_out_x = [self.outer_x[i] for i in indices]
                    real_out_y = [self.outer_y[i] for i in indices]
                elif flag_filter == 0:
                    # 仅对于出现异常外道点时过滤
                    real_out_x = self.outer_x
                    real_out_y = self.outer_y
                else:
                    # 根据圆形滤波过滤
                    real_out_x, real_out_y = self.fit_circle_and_filter(self.inner_x, self.inner_y, self.outer_x, self.outer_y)
                
                # === 生成三等分点（基于源程序main.py）===
                target_group_x, target_group_y = self.gen_three_equinoxes(self.inner_x, self.inner_y, real_out_x, real_out_y)
                
                # === 选择目标点（基于源程序main.py）===
                self.target_x, self.target_y = self.get_target_point(target_group_x, target_group_y)
                
                # 创建画布（plot_space）并转换颜色空间
                plot_space = np.ones((140, 250, 3), dtype=np.uint8) * 255  # 白色背景
                canvas = cv2.cvtColor(plot_space, cv2.COLOR_BGR2RGB)  # 转换为RGB格式
          
                # 1. 绘制内道锥桶（红色）
                for x, y in zip(self.inner_y, self.inner_x):
                    # 坐标转换：将锥桶坐标映射到画布像素（根据画布尺寸140x250调整缩放系数）
                    # 假设原始坐标y轴朝前，x轴左右，这里将坐标原点移至画布底部中心
                    pixel_x = int(- x * 50 + 125)  # 缩放50倍，x轴中心在125像素处
                    pixel_y = int(140 - y * 50)  # 画布y轴向下为正，反转原始y轴（原始y越大越靠上）
                    
                    # 检查坐标是否在画布范围内，避免绘图越界
                    if 0 <= pixel_x < 250 and 0 <= pixel_y < 140:
                        cv2.circle(canvas, (pixel_x, pixel_y), 5, (0, 0, 255), -1)  # 红色填充圆
                        cv2.putText(canvas, f"({x:.1f},{y:.1f})", (pixel_x+5, pixel_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)  # 标注坐标
                
                # 2. 绘制外道锥桶（蓝色）
                for x, y in zip(real_out_y, real_out_x):
                    pixel_x = int(- x * 50 + 125)
                    pixel_y = int(140 - y * 50)
                    if 0 <= pixel_x < 250 and 0 <= pixel_y < 140:
                        cv2.circle(canvas, (pixel_x, pixel_y), 5, (255, 0, 0), -1)  # 蓝色填充圆
                        cv2.putText(canvas, f"({x:.1f},{y:.1f})", (pixel_x+5, pixel_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
                
                # 3. 绘制目标点（绿色）
                if hasattr(self, 'target_x') and hasattr(self, 'target_y'):
                    target_pixel_x = int(self.target_y * (- 50) + 125)
                    target_pixel_y = int(140 - self.target_x * 50)
                    if 0 <= target_pixel_x < 250 and 0 <= target_pixel_y < 140:
                        cv2.circle(canvas, (target_pixel_x, target_pixel_y), 7, (0, 255, 0), -1)  # 绿色填充圆
                        cv2.putText(canvas, "Target", (target_pixel_x+5, target_pixel_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                        # 记录并绘制target_pixel_y的变化趋势
                        try:
                            self.target_pixel_y_history.append(int(target_pixel_y))
                            if len(self.target_pixel_y_history) > self.trend_max_len:
                                self.target_pixel_y_history = self.target_pixel_y_history[-self.trend_max_len:]
                            self.draw_target_trend()
                        except Exception as e:
                            self.get_logger().warn(f"绘制target_pixel_y趋势时出错: {e}")
                
                # 4. 绘制辅助信息（如锥桶数量、坐标系说明）
                cv2.putText(canvas, f"Cones: {self.cone_count} (Inner: {len(self.inner_x)}, Outer: {len(real_out_x)})", 
                            (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                cv2.putText(canvas, "Origin (0,0)", (120, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (128, 128, 128), 1)
                cv2.line(canvas, (125, 140), (125, 10), (128, 128, 128), 1)  # 中轴线（y轴）
                cv2.line(canvas, (10, 140), (240, 140), (128, 128, 128), 1)  # 底线（x轴）
                
                # 5. 实时显示图像（适配ROS2循环回调机制）
                # 注意：cv2.imshow需在主线程或配合cv2.waitKey使用，避免窗口无响应
                cv2.imshow("Cone Detection Visualization", canvas)
                cv2.waitKey(10)  # 等待1ms，允许窗口刷新（必须添加，否则图像无法显示）
            else:
                self.target_x = -10000.0
                self.target_y = -10000.0
                self.inner_x = []
                self.inner_y = []
                self.outer_x = []
                self.outer_y = []
        
        # 发布目标点
        self.publish_target()
        
        # 调试信息
        if self.target_x != -10000:
            self.get_logger().info(
                f'目标点: X={self.target_x:.2f}m, Y={self.target_y:.2f}m | '
                f'锥桶: {self.cone_count}个 | 内道: {len(self.inner_x)}个 | 外道: {len(self.outer_x)}个',
                throttle_duration_sec=0.5
            )
    
    def publish_target(self):
        """发布目标点（基于源程序main.py的发布格式）"""
        msg = Float32MultiArray()
        # 确保值在float32范围内，使用-10000代替-20000作为无效标识
        target_x = float(self.target_x) if abs(self.target_x) < 10000 else -10000.0
        target_y = float(self.target_y) if abs(self.target_y) < 10000 else -10000.0
        msg.data = [target_x, target_y]
        self.target_pub.publish(msg)

    def draw_target_trend(self):
        """绘制target_pixel_y的变化趋势图（OpenCV窗口）"""
        if not self.target_pixel_y_history:
            return
        # 趋势画布尺寸（与主可视化画布一致，便于认知）
        width, height = 250, 140
        img = np.ones((height, width, 3), dtype=np.uint8) * 255

        # 绘制边界和坐标轴
        cv2.rectangle(img, (10, 10), (width - 10, height - 10), (200, 200, 200), 1)
        cv2.line(img, (10, height - 10), (width - 10, height - 10), (180, 180, 180), 1)  # X轴
        cv2.line(img, (10, 10), (10, height - 10), (180, 180, 180), 1)  # Y轴

        # 计算折线点坐标
        n = len(self.target_pixel_y_history)
        if n >= 2:
            xs = [int(10 + i * (width - 20) / (n - 1)) for i in range(n)]
            ys = [int(min(max(y, 10), height - 10)) for y in self.target_pixel_y_history]
            pts = np.array([[xs[i], ys[i]] for i in range(n)], dtype=np.int32)
            cv2.polylines(img, [pts], isClosed=False, color=(0, 180, 0), thickness=2)
            cv2.circle(img, (xs[-1], ys[-1]), 3, (0, 0, 255), -1)

        # 文本标注
        latest = self.target_pixel_y_history[-1]
        cv2.putText(img, f"target_pixel_y: {latest}", (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.putText(img, "Trend (recent)", (14, height - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        cv2.imshow("Target Pixel Y Trend", img)
        cv2.waitKey(1)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

