#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import json
import time
import threading

class DetectionResultsProcessor(Node):
    def __init__(self):
        super().__init__('detection_processor')
        
        # 声明参数
        self.declare_parameter('detection_file_path', 'detection_results.txt')
        
        # 获取参数值
        self.file_path = self.get_parameter('detection_file_path').get_parameter_value().string_value
        
        # 创建发布者
        self.raw_detection_pub = self.create_publisher(String, 'raw_detection_data', 10)
        self.processed_cones_pub = self.create_publisher(String, 'processed_cone_positions', 10)
        self.blue_cones_pub = self.create_publisher(String, 'blue_cones', 10)
        self.red_cones_pub = self.create_publisher(String, 'red_cones', 10)
        
        # 创建定时器，每0.1秒读取一次（匹配文件更新频率）
        self.timer = self.create_timer(0.1, self.process_detection_file)
        
        # 线程锁，确保文件读取的原子性
        self.file_lock = threading.Lock()
        
        # 文件监控
        self.last_modified_time = 0
        self.last_file_size = 0
        
        # 数据缓存
        self.latest_detections = []
        
        self.get_logger().info(f'Detection Results Processor initialized. File: {self.file_path}')

    def process_detection_file(self):
        """主要的文件处理函数"""
        try:
            # 检查文件状态
            if not self.check_file_updated():
                return
                
            # 安全读取文件
            with self.file_lock:
                detection_data = self.safe_read_detection_file()
                
                if detection_data:
                    # 解析检测数据
                    parsed_cones = self.parse_cone_detections(detection_data)
                    
                    if parsed_cones:
                        # 缓存最新数据
                        self.latest_detections = parsed_cones
                        
                        # 发布各种格式的数据
                        self.publish_all_formats(parsed_cones)
                        
                        self.get_logger().debug(f'Processed {len(parsed_cones)} cone detections')
                    
        except Exception as e:
            self.get_logger().error(f'Error in process_detection_file: {str(e)}')

    def check_file_updated(self):
        """检查文件是否已更新"""
        try:
            if not os.path.exists(self.file_path):
                return False
                
            current_modified_time = os.path.getmtime(self.file_path)
            current_file_size = os.path.getsize(self.file_path)
            
            # 检查修改时间和文件大小
            if (current_modified_time != self.last_modified_time or 
                current_file_size != self.last_file_size):
                
                self.last_modified_time = current_modified_time
                self.last_file_size = current_file_size
                return True
                
            return False
            
        except OSError:
            return False

    def safe_read_detection_file(self):
        """安全读取检测文件，处理并发读写问题"""
        max_retries = 5
        retry_delay = 0.005  # 5ms
        
        for attempt in range(max_retries):
            try:
                with open(self.file_path, 'r', encoding='utf-8') as file:
                    content = file.read().strip()
                    
                    # 验证内容完整性
                    if self.validate_content(content):
                        return content
                    elif attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        return content  # 最后一次尝试直接返回
                        
            except (IOError, OSError) as e:
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    self.get_logger().warn(f'Failed to read detection file: {str(e)}')
                    return ""
        
        return ""

    def validate_content(self, content):
        """验证文件内容完整性"""
        if not content:
            return True  # 空文件也算有效
            
        lines = content.strip().split('\n')
        
        # 检查每行是否有完整的6个字段
        for line in lines:
            if line.strip():
                parts = line.strip().split()
                if len(parts) != 6:
                    return False  # 数据不完整
                    
        return True

    def parse_cone_detections(self, content):
        """
        解析锥桶检测数据
        输入格式: bluep 0.9660072326660156 410.0 312.5 144.88137563584363 989.719271623672
        提取: 类别(bluep/redp), x坐标, y坐标
        """
        cone_detections = []
        
        if not content:
            return cone_detections
            
        try:
            lines = content.strip().split('\n')
            
            for line_num, line in enumerate(lines, 1):
                line = line.strip()
                if not line:
                    continue
                    
                try:
                    parts = line.split()
                    
                    if len(parts) != 6:
                        self.get_logger().warn(f'Line {line_num}: Invalid format (expected 6 fields, got {len(parts)})')
                        continue
                    
                    # 解析数据
                    category = parts[0]  # bluep 或 redp
                    confidence = float(parts[1])
                    pixel_x = float(parts[2])
                    pixel_y = float(parts[3])
                    world_x = float(parts[4])  # 关键数据：世界坐标X
                    world_y = float(parts[5])  # 关键数据：世界坐标Y
                    
                    # 验证类别
                    if category not in ['bluep', 'redp']:
                        self.get_logger().warn(f'Line {line_num}: Unknown category "{category}"')
                        continue
                    
                    # 创建锥桶数据
                    cone_data = {
                        'category': category,
                        'x': world_x,
                        'y': world_y,
                        'confidence': confidence,
                        'pixel_x': pixel_x,
                        'pixel_y': pixel_y,
                        'timestamp': time.time()
                    }
                    
                    cone_detections.append(cone_data)
                    
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f'Line {line_num}: Parse error - {str(e)}')
                    continue
                    
        except Exception as e:
            self.get_logger().error(f'Error parsing cone detections: {str(e)}')
            
        return cone_detections

    def publish_all_formats(self, cone_detections):
        """发布不同格式的检测结果"""
        try:
            # 1. 发布原始检测数据（简化格式）
            raw_lines = []
            for cone in cone_detections:
                raw_lines.append(f"{cone['category']}: x={cone['x']:.2f}, y={cone['y']:.2f}, conf={cone['confidence']:.3f}")
            
            raw_msg = String()
            raw_msg.data = '\n'.join(raw_lines)
            self.raw_detection_pub.publish(raw_msg)
            
            # 2. 发布JSON格式的完整数据
            processed_msg = String()
            processed_msg.data = json.dumps(cone_detections, indent=2)
            self.processed_cones_pub.publish(processed_msg)
            
            # 3. 按颜色分类发布
            blue_cones = [cone for cone in cone_detections if cone['category'] == 'bluep']
            red_cones = [cone for cone in cone_detections if cone['category'] == 'redp']
            
            if blue_cones:
                blue_msg = String()
                blue_msg.data = json.dumps(blue_cones, indent=2)
                self.blue_cones_pub.publish(blue_msg)
            
            if red_cones:
                red_msg = String()
                red_msg.data = json.dumps(red_cones, indent=2)
                self.red_cones_pub.publish(red_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing detection data: {str(e)}')

    def get_latest_detections(self):
        """获取最新的检测结果（供其他节点调用）"""
        return self.latest_detections.copy()
        
    def get_cones_by_category(self, category):
        """根据类别获取锥桶"""
        return [cone for cone in self.latest_detections if cone['category'] == category]


def main(args=None):
    rclpy.init(args=args)
    processor = DetectionResultsProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Detection processor shutting down...')
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()