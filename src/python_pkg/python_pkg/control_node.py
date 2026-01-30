#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制节点 - 基于NCSC2024源程序racecar_teleop_test.py
- 订阅感知节点的目标点话题
- 执行PID控制逻辑
- 发布车辆驱动指令
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool
import math
import numpy as np
import time


class ControlNode(Node):
    """控制节点 - 基于源程序racecar_teleop_test.py的Route类"""
    
    def __init__(self):
        super().__init__('control_node')
        
        # --- 声明参数（基于源程序racecar_teleop_test.py）---
        # 速度控制参数（提高基础速度，保持安全范围）
        self.declare_parameter('base_speed', 50)           # 基础速度档位 (1550)
        self.declare_parameter('circle1_speed', 50)        # 第一圈速度 (1550)
        self.declare_parameter('circle2_speed', 60)        # 第二圈速度 (1560)
        self.declare_parameter('park_speed', 00)           # 停车速度 (1500)
        
        # PID参数（基于源程序的pidList）- 增大P值提高转向灵敏度
        self.declare_parameter('pid_params_20', [-90.0, 0.0, 0.1])   # 低速PID
        self.declare_parameter('pid_params_25', [-100.0, 0.0, 0.12])  # 停车速度PID (1550)
        self.declare_parameter('pid_params_30', [-110.0, 0.0, 0.15])  # 第一圈速度PID (1550)
        self.declare_parameter('pid_params_35', [-120.0, 0.0, 0.18])  # 第二圈速度PID (1560)
        self.declare_parameter('pid_params_45', [-80.0, 0.0, 0.1])
        self.declare_parameter('pid_params_50', [-90.0, 0.0, 0.2])
        self.declare_parameter('pid_params_65', [-90.0, 0.0, 0.1])
        self.declare_parameter('pid_params_70', [-140.0, 0.0, 0.5])
        self.declare_parameter('pid_params_75', [-120.0, 0.0, 20.0])
        self.declare_parameter('pid_params_80', [-145.0, 0.0, 10.0])
        self.declare_parameter('pid_params_85', [-185.0, 0.0, 15.0])
        
        # 动态PID参数
        self.declare_parameter('pid_k_factor', 0.3)        # 直道PID中P的比例因子
        self.declare_parameter('max_error', 100.0)         # 最大误差（用于积分分离）
        self.declare_parameter('alpha_factor', -0.05)      # 积分分离Alpha因子
        self.declare_parameter('sum_err_threshold', 100.0) # 积分项上限
        self.declare_parameter('min_dist_threshold', 0.3)  # 到达目标点的最小距离
        
        # 转向参数（基于源程序的turnTheta）- 增大转向角度避免撞锥桶
        self.declare_parameter('turn_theta_20', [35, 35])  # 低速转向
        self.declare_parameter('turn_theta_25', [38, 38])  # 停车速度转向 (1525)
        self.declare_parameter('turn_theta_30', [40, 40])  # 第一圈转向 (1530)
        self.declare_parameter('turn_theta_35', [42, 42])  # 第二圈转向 (1535)
        self.declare_parameter('turn_theta_45', [30, 30])
        self.declare_parameter('turn_theta_50', [35, 35])
        self.declare_parameter('turn_theta_65', [36, 36])
        self.declare_parameter('turn_theta_70', [45, 45])
        self.declare_parameter('turn_theta_75', [45, 47])
        self.declare_parameter('turn_theta_80', [55, 60])
        self.declare_parameter('turn_theta_85', [100, 100])
        
        # 获取参数
        self.base_speed = self.get_parameter('base_speed').value
        self.circle1_speed = self.get_parameter('circle1_speed').value
        self.circle2_speed = self.get_parameter('circle2_speed').value
        self.park_speed = self.get_parameter('park_speed').value
        
        # PID参数字典
        self.pid_params = {
            20: self.get_parameter('pid_params_20').value,
            25: self.get_parameter('pid_params_25').value,
            30: self.get_parameter('pid_params_30').value,
            35: self.get_parameter('pid_params_35').value,
            45: self.get_parameter('pid_params_45').value,
            50: self.get_parameter('pid_params_50').value,
            65: self.get_parameter('pid_params_65').value,
            70: self.get_parameter('pid_params_70').value,
            75: self.get_parameter('pid_params_75').value,
            80: self.get_parameter('pid_params_80').value,
            85: self.get_parameter('pid_params_85').value
        }
        
        # 动态PID参数
        self.pid_k_factor = self.get_parameter('pid_k_factor').value
        self.max_error = self.get_parameter('max_error').value
        self.alpha_factor = self.get_parameter('alpha_factor').value
        self.sum_err_threshold = self.get_parameter('sum_err_threshold').value
        self.min_dist_threshold = self.get_parameter('min_dist_threshold').value
        
        # 转向参数字典
        self.turn_theta = {
            20: self.get_parameter('turn_theta_20').value,
            25: self.get_parameter('turn_theta_25').value,
            30: self.get_parameter('turn_theta_30').value,
            35: self.get_parameter('turn_theta_35').value,
            45: self.get_parameter('turn_theta_45').value,
            50: self.get_parameter('turn_theta_50').value,
            65: self.get_parameter('turn_theta_65').value,
            70: self.get_parameter('turn_theta_70').value,
            75: self.get_parameter('turn_theta_75').value,
            80: self.get_parameter('turn_theta_80').value,
            85: self.get_parameter('turn_theta_85').value
        }
        
        # --- 状态变量（基于源程序Route类）---
        self.route_point = ()          # 目标点
        self.absolute_point = ()        # 小车绝对坐标
        self.relative_point = ()        # 相对坐标系下的xy坐标
        self.angle = 0.0                # 小车姿态角度，范围[-pi,pi]
        self.last_error = 0.0           # 上次的误差，用于pid微分
        self.sum_error = 0.0            # 误差积分，用于pid积分
        self.origin_p = self.pid_params[self.circle1_speed][0]  # 保存原始P值
        
        # 运行状态
        self.circle = 1                 # 圈数：1为第一圈，2为第二圈
        self.start = False              # 是否开始运行
        self.stop = False               # 是否停车
        self.red = 0                    # 红灯检测
        self.yellow = 0                 # 黄线检测
        self.park = False               # 是否进入停车模式
        self.cooldown = 0               # 红灯冷却时间
        self.A_B = 2                    # 1为A，2为B。预设为B
        
        # 控制参数
        self.v = self.circle1_speed     # 当前速度档位
        self.last_twist = None          # 上次的控制指令
        
        # --- QoS配置 ---
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- ROS2 订阅与发布 ---
        # 订阅目标点（来自感知节点）
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            '/target',
            self.target_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 订阅里程计（用于位置和姿态）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',  # 使用实际的里程计话题
            self.odom_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 发布车辆控制指令（使用teleop_cmd_vel话题，与ROS2驱动器匹配）
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        
        # 发布导航开始信号
        self.navi_pub = self.create_publisher(Bool, '/navi', 10)
        
        # 发布开始信号
        self.start_pub = self.create_publisher(Bool, '/start', 10)
        
        # 定时器（20Hz控制频率）
        self.timer = self.create_timer(0.05, self.control_callback)
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("控制节点已启动（基于NCSC2024源程序）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"速度档位: 第一圈={self.circle1_speed}, 第二圈={self.circle2_speed}, 停车={self.park_speed}")
        self.get_logger().info(f"PID参数: {self.pid_params}")
        self.get_logger().info(f"动态PID: K因子={self.pid_k_factor}, 积分分离阈值={self.max_error}")
        self.get_logger().info(f"转向参数: {self.turn_theta}")
        self.get_logger().info("=" * 60)
    
    def pid_control(self, error, last_error, sum_error, v):
        """
        PID控制（基于源程序racecar_teleop_test.py的PID函数）
        采用积分分离法，对积分系数进行抛物线控制(误差小时积分项控制作用大)
        """
        p, i, d = self.pid_params[v]
        
        # 积分分离法（基于源程序）
        if abs(error) < self.max_error:
            i = (error - self.max_error) ** 2 / self.max_error ** 2 * self.alpha_factor
        
        output_theta = p * error + i * sum_error + d * (error - last_error)
        return output_theta
    
    def drive(self, v, theta, last=False):
        """
        驱动小车（基于源程序racecar_teleop_test.py的drive函数）
        输入小车的v和舵机角度，驱动小车
        """
        if last:
            try:
                self.cmd_vel_pub.publish(self.last_twist)
            except:
                pass
            return
        
        start_speed = 1500
        start_theta = 75
        
        twist = Twist()
        twist.linear.x = float(start_speed + v)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(start_theta + theta)
        twist.angular.z = max(min(twist.angular.z, 180.0), 0.0)
        
        # 调试输出
        if self.absolute_point != ():
            self.get_logger().info(
                f'位置: ({self.absolute_point[0]:.2f}, {self.absolute_point[1]:.2f}) | '
                f'转向: {twist.angular.z:.2f}',
                throttle_duration_sec=0.1
            )
        
        self.cmd_vel_pub.publish(twist)
        self.last_twist = twist
    
    def update_origin_p(self):
        """更新原始P值（基于源程序）"""
        self.origin_p = self.pid_params[self.v][0]
    
    def detect_cooldown(self):
        """检测冷却时间（基于源程序）"""
        if self.cooldown > 0:
            self.red = 0
            self.cooldown -= 1
    
    def set_car_point_and_angle(self, msg):
        """刷新小车绝对坐标与角度（基于源程序）"""
        self.absolute_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # 从四元数提取yaw角度
        orientation = msg.pose.pose.orientation
        # 使用简化的四元数到欧拉角转换
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.angle = np.round(math.atan2(siny_cosp, cosy_cosp), 6)
    
    def set_goal_point(self, p):
        """设置目标点（基于源程序）"""
        self.route_point = (p[0], p[1])
    
    def set_relative_point(self, p):
        """设置相对点（基于源程序）"""
        self.relative_point = p
    
    def delete_goal_point(self):
        """删除目标点（基于源程序）"""
        if self.route_point != ():
            if (abs(self.absolute_point[0] - self.route_point[0]) + 
                abs(self.absolute_point[1] - self.route_point[1])) < self.min_dist_threshold:
                self.route_point = ()
    
    def get_route_theta(self):
        """返回小车和目标点的夹角（基于源程序）"""
        if self.route_point == ():
            return 0.0
        return math.atan2(self.route_point[1] - self.absolute_point[1], 
                         self.route_point[0] - self.absolute_point[0])
    
    def tf_coordinates(self):
        """根据目标点相对坐标进行坐标变换，并设置绝对坐标下的目标点（基于源程序）"""
        if self.absolute_point == () or self.relative_point == ():
            return
        
        (x0, y0) = self.absolute_point
        (x2, y2) = self.relative_point
        theta1 = self.angle
        theta2 = math.atan2(y2, x2)
        relative_length = math.sqrt(x2 ** 2 + y2 ** 2)
        
        self.set_goal_point((
            x0 + relative_length * math.cos(theta1 + theta2), 
            y0 + relative_length * math.sin(theta1 + theta2)
        ))
    
    def turn_left(self):
        """左转（基于源程序）"""
        if self.v in self.turn_theta:
            turn_angle = self.turn_theta[self.v][self.circle - 1]
            self.drive(self.v, turn_angle)
    
    def start_navi(self):
        """开始导航（基于源程序）"""
        self.navi_pub.publish(Bool(data=False))
        time.sleep(3)
        self.navi_pub.publish(Bool(data=True))
    
    def active_pid(self, if_line):
        """动态PID（基于源程序）"""
        if if_line:  # 进入直道
            self.pid_params[self.v][0] = self.pid_params[self.v][0] * self.pid_k_factor
        else:
            self.pid_params[self.v][0] = self.origin_p
    
    def stop_the_car(self):
        """停车检测（基于源程序）"""
        if self.park and self.yellow:
            self.stop = True
    
    def car_drive(self):
        """车辆驱动逻辑（基于源程序）"""
        if (self.route_point != () and self.relative_point != () and 
            self.relative_point[0] > -10000 and self.relative_point[1] > -10000):
            
            self.get_logger().debug("正常驾驶模式")
            
            # 计算误差（车身姿态角 - 目标角度）
            error = self.angle - self.get_route_theta()
            
            # 角度误差归一化
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi
            
            # PID控制
            steering_theta = self.pid_control(error, self.last_error, self.sum_error, self.v)
            self.last_error = error
            
            # 积分项限幅
            if self.sum_error < self.sum_err_threshold and self.sum_error > -self.sum_err_threshold:
                self.sum_error += error
            
            # 大弯道自动降速：如果误差大于0.3弧度（约17度），降速8档（因为基础速度提高了）
            current_speed = self.v
            if abs(error) > 0.3:
                current_speed = max(20, self.v - 8)  # 降速8档，最低20档
                self.get_logger().debug(f"大弯道降速: {self.v} → {current_speed}")
            
            self.drive(current_speed, steering_theta)
        else:
            self.get_logger().debug("左转模式")
            self.turn_left()
    
    def run(self):
        """主运行逻辑（基于源程序）"""
        self.stop_the_car()
        self.detect_cooldown()
        
        if self.stop:
            # 停车
            self.get_logger().warn('--------Stop!!!---------')
            self.drive(-500, 0)
            return
        
        if self.red and self.cooldown < 1:
            self.get_logger().warn("=================>Red light!<==================")
            self.drive(-500, 0)
            self.cooldown = 300
            
            if self.circle == 1:
                # 第一圈识别到红灯
                self.start_navi()
                self.circle = 2
                self.v = self.circle2_speed
                self.update_origin_p()
                self.active_pid(True)
                return
            else:
                # 进入识别A、B阶段
                time.sleep(3)
                self.park = True
                self.v = self.park_speed
                self.update_origin_p()
                self.active_pid(True)
                return
        
        self.car_drive()
    
    def target_callback(self, msg: Float32MultiArray):
        """目标点回调（基于源程序getTarget函数）"""
        if not self.start:
            self.start_pub.publish(Bool(data=True))
            self.start = True
            # 不要return，继续处理这一帧数据
        
        data = msg.data
        if len(data) >= 2:
            self.set_relative_point((data[0], data[1]))
            if self.absolute_point != ():  # 只有在有里程计数据时才进行坐标变换
                self.tf_coordinates()
    
    def odom_callback(self, msg: Odometry):
        """里程计回调（基于源程序callBack函数）"""
        if not self.start:
            return
        
        self.delete_goal_point()
        self.set_car_point_and_angle(msg)
        self.run()
    
    def control_callback(self):
        """控制回调（定时器触发）"""
        # 这里可以添加额外的控制逻辑
        pass


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = ControlNode()
    
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
