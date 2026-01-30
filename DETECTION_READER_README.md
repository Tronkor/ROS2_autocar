# 检测结果文件读取节点说明

## 概述
这个ROS2节点专门用于读取和处理 `detection_results.txt` 文件，该文件包含锥桶检测数据。节点处理了文件频繁更新（0.1秒一次）的并发问题，确保数据读取的稳定性。

## 文件格式
```
bluep 0.9660072326660156 410.0 312.5 144.88137563584363 989.719271623672
redp 0.9396219253540039 48.65625 276.46875 -1754.0471040763907 2183.6367440887198
```

字段说明：
- `bluep/redp`: 锥桶类别（蓝色/红色锥桶）
- `0.966...`: 检测置信度 (0-1)
- `410.0`: 像素坐标X
- `312.5`: 像素坐标Y  
- `144.88...`: **世界坐标X** (关键数据)
- `989.72...`: **世界坐标Y** (关键数据)

## 核心特性

### 1. 数据稳定性保障
- **文件锁机制**: 使用线程锁防止读写冲突
- **重试机制**: 最多5次重试，防止读取不完整数据
- **内容验证**: 检查数据格式完整性
- **修改时间监控**: 只在文件更新时读取

### 2. 高频处理能力
- **0.1秒更新频率**: 匹配检测文件的刷新频率
- **高效文件监控**: 基于修改时间和文件大小
- **最小延迟**: 5ms重试间隔

### 3. 多格式输出
发布以下话题：
- `/raw_detection_data`: 人类可读格式
- `/processed_cone_positions`: JSON格式完整数据
- `/blue_cones`: 蓝色锥桶数据
- `/red_cones`: 红色锥桶数据

## 使用方法

### 1. 编译安装
```bash
cd ~/demo_ws
colcon build
source install/setup.bash
```

### 2. 运行节点

#### 方法1: 直接运行
```bash
# 使用默认文件路径
ros2 run python_pkg detection_processor

# 指定文件路径
ros2 run python_pkg detection_processor --ros-args -p detection_file_path:=/path/to/detection_results.txt
```

#### 方法2: 使用Launch文件
```bash
ros2 launch python_pkg detection_processor.launch.py detection_file_path:=/path/to/detection_results.txt
```

### 3. 查看输出数据

查看原始检测数据：
```bash
ros2 topic echo /raw_detection_data
```

查看JSON格式数据：
```bash
ros2 topic echo /processed_cone_positions
```

查看蓝色锥桶：
```bash
ros2 topic echo /blue_cones
```

查看红色锥桶：
```bash
ros2 topic echo /red_cones
```

### 4. 模拟测试

运行检测数据模拟器：
```bash
cd ~/demo_ws
python3 simulate_detection.py
```

## 数据结构

### 解析后的锥桶数据格式：
```json
{
  "category": "bluep",
  "x": 144.88137563584363,
  "y": 989.719271623672,
  "confidence": 0.9660072326660156,
  "pixel_x": 410.0,
  "pixel_y": 312.5,
  "timestamp": 1637123456.789
}
```

## 错误处理

### 常见问题和解决方案：

1. **文件不存在**
   - 检查文件路径是否正确
   - 确保检测程序正在运行并生成文件

2. **权限问题**
   - 确保ROS节点有文件读取权限
   - 检查文件所有者和权限设置

3. **数据格式错误**
   - 节点会跳过格式不正确的行
   - 检查日志中的警告信息

4. **并发读写问题**
   - 节点已内置文件锁和重试机制
   - 如仍有问题，可调整重试参数

## 性能监控

查看节点状态：
```bash
ros2 node info /detection_processor
```

查看话题发布频率：
```bash
ros2 topic hz /processed_cone_positions
```

查看系统资源使用：
```bash
top -p $(pgrep -f detection_processor)
```

## 自定义配置

可在节点中调整的参数：
- `max_retries`: 文件读取重试次数 (默认5)
- `retry_delay`: 重试间隔时间 (默认0.005秒)
- `timer_period`: 处理频率 (默认0.1秒)

## 集成到导航系统

在你的导航代码中使用检测数据：
```python
# 订阅处理后的锥桶数据
self.cone_sub = self.create_subscription(
    String, 
    'processed_cone_positions', 
    self.cone_callback, 
    10
)

def cone_callback(self, msg):
    cone_data = json.loads(msg.data)
    # 处理锥桶位置信息
    for cone in cone_data:
        if cone['category'] == 'bluep':
            # 处理蓝色锥桶
            x, y = cone['x'], cone['y']
        elif cone['category'] == 'redp':
            # 处理红色锥桶  
            x, y = cone['x'], cone['y']
```

这样就可以在你的导航节点中实时获取锥桶位置信息了！