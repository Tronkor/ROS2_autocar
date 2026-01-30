#!/usr/bin/env python3

import time
import random
import os

def generate_detection_data():
    """生成模拟的检测数据"""
    detections = []
    
    # 生成1-6个随机检测结果
    num_detections = random.randint(1, 6)
    
    for _ in range(num_detections):
        # 随机选择锥桶类型
        category = random.choice(['bluep', 'redp'])
        
        # 生成置信度 (0.8-0.99)
        confidence = random.uniform(0.8, 0.99)
        
        # 生成像素坐标
        pixel_x = random.uniform(50, 600)
        pixel_y = random.uniform(200, 400)
        
        # 生成世界坐标
        world_x = random.uniform(-2000, 2000)
        world_y = random.uniform(500, 4000)
        
        detection_line = f"{category} {confidence:.15f} {pixel_x} {pixel_y} {world_x} {world_y}"
        detections.append(detection_line)
    
    return '\n'.join(detections)

def simulate_detection_file(file_path, duration_seconds=60):
    """模拟检测文件的更新"""
    print(f"Starting detection file simulation...")
    print(f"File path: {file_path}")
    print(f"Duration: {duration_seconds} seconds")
    print(f"Update frequency: every 0.1 seconds")
    
    start_time = time.time()
    update_count = 0
    
    try:
        while time.time() - start_time < duration_seconds:
            # 生成新的检测数据
            detection_data = generate_detection_data()
            
            # 写入文件
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(detection_data)
            
            update_count += 1
            
            if update_count % 10 == 0:  # 每秒输出一次状态
                elapsed = time.time() - start_time
                print(f"Updates: {update_count}, Elapsed: {elapsed:.1f}s")
            
            # 等待0.1秒
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print(f"\nSimulation stopped by user after {update_count} updates")
    except Exception as e:
        print(f"Error during simulation: {e}")
    
    print(f"Simulation completed. Total updates: {update_count}")

if __name__ == "__main__":
    # 默认文件路径
    detection_file_path = "detection_results.txt"
    
    # 检查是否在正确的目录
    current_dir = os.getcwd()
    print(f"Current directory: {current_dir}")
    
    # 如果在demo_ws目录下，使用该路径
    if "demo_ws" in current_dir:
        detection_file_path = os.path.join(current_dir, "detection_results.txt")
    
    print(f"Will create/update file: {detection_file_path}")
    
    # 创建初始文件
    initial_data = generate_detection_data()
    with open(detection_file_path, 'w', encoding='utf-8') as f:
        f.write(initial_data)
    
    print(f"Initial file created with sample data")
    print("Starting continuous updates in 3 seconds...")
    time.sleep(3)
    
    # 开始模拟
    simulate_detection_file(detection_file_path, duration_seconds=300)  # 运行5分钟