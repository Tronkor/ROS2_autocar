#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import json
import time
import threading

class DetectionResultsNode(Node):
    def __init__(self):
        super().__init__('detection_results_reader')
        
        # 声明参数
        self.declare_parameter('file_path', '/home/davinci-mini/test/infer_project_om/detection_results.txt')
        
        # 获取参数值
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        
        # 创建发布者 - 发布检测结果
        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.cone_data_publisher = self.create_publisher(String, 'cone_positions', 10)
        
        # 创建速度发布者，用于巡锥桶控制
        self.cmd_vel_publisher = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        
        # 创建定时器，每0.1秒读取一次文件（匹配文件刷新频率）
        self.timer = self.create_timer(0.1, self.read_detection_file)
        
        # 文件锁对象
        self.file_lock = threading.Lock()
        
        # 上次修改时间，用于检测文件是否有更新
        self.last_modified_time = 0
        
        # PD控制器参数
        self.Kp = 60.0  # 比例系数
        self.Kd = 1.5   # 微分系数
        self.SPEED = 1560  # 默认前进速度值
        
        # PD状态变量
        self.error_ = 0.0
        self.last_error_ = 0.0
        self.last_time_ = self.get_clock().now()
        
        # 锥桶数据缓存
        self.red_cones = []
        self.blue_cones = []
        
        self.get_logger().info(f'Detection Results Reader started. Reading file: {self.file_path}')

    def read_detection_file(self):
        """读取检测结果文件并解析锥桶数据"""
        try:
            # 使用文件锁确保读取稳定性
            with self.file_lock:
                # 检查文件是否存在
                if not os.path.exists(self.file_path):
                    self.get_logger().warn(f'Detection file not found: {self.file_path}')
                    return
                
                # 检查文件是否有更新
                current_modified_time = os.path.getmtime(self.file_path)
                if current_modified_time == self.last_modified_time:
                    return  # 文件未更新，跳过读取
                
                self.last_modified_time = current_modified_time
                
                # 尝试多次读取以防止读取到不完整的数据
                detection_data = self.safe_read_file()
                
                if detection_data:
                    # 解析检测数据
                    parsed_results = self.parse_detection_data(detection_data)
                    
                    if parsed_results:
                        # 发布解析后的数据
                        self.publish_detection_results(parsed_results)
                        
                        # 巡锥桶控制
                        self.cone_following_control(parsed_results)
                        
                        self.get_logger().info(f'Parsed {len(parsed_results)} cone detections')
                    
        except Exception as e:
            self.get_logger().error(f'Error reading detection file: {str(e)}')

    def safe_read_file(self, max_retries=3):
        """安全读取文件，防止读取到不完整的数据"""
        for attempt in range(max_retries):
            try:
                with open(self.file_path, 'r', encoding='utf-8') as file:
                    content = file.read().strip()
                    
                    # 如果内容为空，等待一小段时间后重试
                    if not content and attempt < max_retries - 1:
                        time.sleep(0.01)  # 等待10ms
                        continue
                        
                    return content
                    
            except (IOError, OSError) as e:
                if attempt < max_retries - 1:
                    time.sleep(0.01)  # 等待后重试
                    continue
                else:
                    self.get_logger().error(f'Failed to read file after {max_retries} attempts: {str(e)}')
                    return ""
        
        return ""

    def parse_detection_data(self, content):
        """
        解析检测结果数据
        格式: bluep 0.9660072326660156 410.0 312.5 144.88137563584363 989.719271623672
        提取: 类别(bluep), x坐标(144.88137563584363), y坐标(989.719271623672)
        """
        parsed_results = []
        
        try:
            lines = content.strip().split('\n')
            
            for line in lines:
                if not line.strip():
                    continue
                    
                parts = line.strip().split()
                
                # 检查数据格式是否正确（应该有6个部分）
                if len(parts) != 6:
                    self.get_logger().warn(f'Invalid line format: {line}')
                    continue
                
                try:
                    category = parts[0]  # 锥桶类别 (bluep/redp)
                    confidence = float(parts[1])  # 置信度
                    pixel_x = float(parts[2])  # 像素x坐标
                    pixel_y = float(parts[3])  # 像素y坐标
                    world_x = float(parts[4])  # 世界坐标x
                    world_y = float(parts[5])  # 世界坐标y
                    
                    # 只保留有用的数据：类别、世界坐标x、世界坐标y
                    cone_data = {
                        'category': category,
                        'x': world_x,
                        'y': world_y,
                        'confidence': confidence  # 额外保留置信度用于质量判断
                    }
                    
                    # 日志打印每个锥桶的坐标信息
                    self.get_logger().info(f'Cone detected: {category} at position (x={world_x:.3f}, y={world_y:.3f}) confidence={confidence:.3f}')
                    
                    parsed_results.append(cone_data)
                    
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f'Error parsing line "{line}": {str(e)}')
                    continue
                    
        except Exception as e:
            self.get_logger().error(f'Error parsing detection data: {str(e)}')
            
        return parsed_results

    def publish_detection_results(self, parsed_results):
        """发布解析后的检测结果"""
        try:
            # 发布原始格式的检测结果
            raw_msg = String()
            raw_lines = []
            for result in parsed_results:
                raw_lines.append(f"{result['category']} x:{result['x']:.2f} y:{result['y']:.2f} conf:{result['confidence']:.3f}")
            raw_msg.data = '\n'.join(raw_lines)
            self.detection_publisher.publish(raw_msg)
            
            # 发布JSON格式的锥桶位置数据
            cone_msg = String()
            cone_msg.data = json.dumps(parsed_results, indent=2)
            self.cone_data_publisher.publish(cone_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detection results: {str(e)}')

    def get_cone_by_category(self, category):
        """根据类别筛选锥桶数据"""
        # 这个方法可以用于外部调用，获取特定类别的锥桶
        pass
    
    def cone_following_control(self, cone_detections):
        """巡锥桶控制算法 - 红锥桶为内道，蓝锥桶为外道"""
        try:
            # 分离红蓝锥桶并按y坐标排序（y坐标小的更靠近小车）
            red_cones = [cone for cone in cone_detections if cone['category'] == 'redp']
            blue_cones = [cone for cone in cone_detections if cone['category'] == 'bluep']
            
            # 按y坐标排序，y小的在前（更靠近小车，权重更大）
            red_cones.sort(key=lambda cone: cone['y'])
            blue_cones.sort(key=lambda cone: cone['y'])
            
            # 更新缓存
            self.red_cones = red_cones
            self.blue_cones = blue_cones
            
            # 检查是否有足够的锥桶进行控制
            if len(red_cones) == 0 and len(blue_cones) == 0:
                self.get_logger().warn('No cones detected for control')
                return
            
            # 计算控制误差
            error = self.calculate_control_error(red_cones, blue_cones)
            
            # PD控制计算
            control_output = self.pd_control(error)
            
            # 发布控制命令
            self.publish_control_command(control_output)
            
        except Exception as e:
            self.get_logger().error(f'Error in cone following control: {str(e)}')
    
    def calculate_control_error(self, red_cones, blue_cones):
        """计算控制误差（带权重滤波），前馈"""
        gamma = 0.7  # 权重衰减系数
        error = 0.0
        
        # 确定处理的锥桶数量（取红蓝锥桶数量的最小值）
        red_count = len(red_cones)
        blue_count = len(blue_cones)
        
        if red_count == 0 and blue_count == 0:
            return 0.0
        
        # 处理只有一种颜色锥桶的情况
        if red_count == 0:
            # 只有蓝锥桶，偏向蓝锥桶一侧（外道）
            for i, blue_cone in enumerate(blue_cones[:3]):  # 最多取前3个
                weight = gamma ** i
                error += weight * (-blue_cone['x'])  # 负号表示向内偏
                self.get_logger().debug(f'Blue cone {i}: x={blue_cone["x"]:.2f}, weight={weight:.2f}')
        elif blue_count == 0:
            # 只有红锥桶，偏向红锥桶一侧（内道）
            for i, red_cone in enumerate(red_cones[:3]):  # 最多取前3个
                weight = gamma ** i
                error += weight * red_cone['x']  # 正号表示向外偏
                self.get_logger().debug(f'Red cone {i}: x={red_cone["x"]:.2f}, weight={weight:.2f}')
        else:
            # 红蓝锥桶都存在，计算中线误差
            min_count = min(red_count, blue_count)
            for i in range(min_count):
                weight = gamma ** i
                # 红锥桶x + 蓝锥桶x的平均作为目标线，误差为当前位置偏离目标线的距离
                target_line = (red_cones[i]['x'] + blue_cones[min_count - i - 1]['x']) / 2.0
                error += weight * target_line
                
                self.get_logger().debug(f'Pair {i}: red_x={red_cones[i]["x"]:.2f}, blue_x={blue_cones[min_count-i-1]["x"]:.2f}, target={target_line:.2f}, weight={weight:.2f}')
        
        self.get_logger().info(f'Control error calculated: {error:.3f} (red_cones: {red_count}, blue_cones: {blue_count})')
        return error
    
    def pd_control(self, error):
        """PD控制器计算"""
        # 计算时间间隔（微分项需要基于时间变化率）
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_).nanoseconds / 1e9  # 转换为秒
        
        # 首次运行时初始化时间（避免dt为0导致微分计算错误）
        if dt <= 0.0:
            self.last_time_ = current_time
            self.last_error_ = error
            return 0.0
        
        # 计算微分项（误差变化率）
        derivative = (error - self.last_error_) / dt
        
        # 微分项限幅（可选）
        # deriv_limit = 5.0
        # derivative = max(-deriv_limit, min(derivative, deriv_limit))
        
        # PD总输出（转向角度）
        control_output = self.Kp * error - self.Kd * derivative
        
        # 保存当前状态供下次计算
        self.last_error_ = error
        self.last_time_ = current_time
        
        # 日志输出
        self.get_logger().info(f'PD Control - Error: {error:.3f}, Derivative: {derivative:.3f}, Output: {control_output:.3f}')
        
        return control_output
    
    def publish_control_command(self, control_output):
        """发布控制命令"""
        try:
            # 创建Twist消息
            twist = Twist()
            
            # 设置角速度（转向）
            angular_z = 90.0 + control_output
            
            # 角速度限幅
            if angular_z > 150.0:
                angular_z = 150.0
            elif angular_z < 30.0:
                angular_z = 30.0
            
            # 设置速度命令
            twist.linear.x = float(self.SPEED)
            twist.angular.z = angular_z
            
            # 其他分量设为0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            
            # 发布命令
            self.cmd_vel_publisher.publish(twist)
            
            self.get_logger().info(f'Control command published - Linear: {self.SPEED}, Angular: {angular_z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing control command: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionResultsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()