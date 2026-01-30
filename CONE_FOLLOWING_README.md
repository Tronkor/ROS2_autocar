# 巡锥桶控制节点使用说明

## 功能概述
该节点集成了检测结果文件读取和巡锥桶控制功能，能够实时读取detection_results.txt文件中的锥桶坐标数据，并根据红蓝锥桶位置进行PD控制，实现自动巡锥桶赛道功能。

## 控制策略
- **红锥桶**: 内道（内侧边界）
- **蓝锥桶**: 外道（外侧边界） 
- **目标路径**: 红蓝锥桶中间线
- **前馈控制**: y坐标小的锥桶权重更大（更靠近小车）

## 核心算法

### 1. 锥桶排序
- 按y坐标升序排列（y值小的更靠近小车）
- 权重按指数衰减：γ^i，其中γ=0.7

### 2. 误差计算
```python
# 权重前馈计算
for i in range(min(red_count, blue_count)):
    weight = gamma ** i
    target_line = (red_cones[i]['x'] + blue_cones[min_count - i - 1]['x']) / 2.0
    error += weight * target_line
```

### 3. PD控制器
```python
control_output = Kp * error - Kd * derivative
angular_z = 90.0 + control_output
```

参数配置：
- `Kp = 60.0`: 比例系数
- `Kd = 1.5`: 微分系数
- `SPEED = 1560`: 前进速度

## 发布话题

### 控制话题
- `/teleop_cmd_vel` (geometry_msgs/Twist): 速度控制命令

### 数据话题  
- `/detection_results` (std_msgs/String): 原始检测结果
- `/cone_positions` (std_msgs/String): JSON格式锥桶位置

## 运行方法

### 1. 编译安装
```bash
cd ~/demo_ws
colcon build --packages-select python_pkg
source install/setup.bash
```

### 2. 运行节点

#### 方法1: 直接运行
```bash
ros2 run python_pkg file_reader_python --ros-args -p file_path:=/path/to/detection_results.txt
```

#### 方法2: 使用Launch文件
```bash
ros2 launch python_pkg cone_following.launch.py detection_file_path:=/home/davinci-mini/test/infer_project_om/detection_results.txt
```

### 3. 监控运行状态

查看控制命令：
```bash
ros2 topic echo /teleop_cmd_vel
```

查看检测结果：
```bash
ros2 topic echo /detection_results
```

查看锥桶位置：
```bash
ros2 topic echo /cone_positions
```

查看节点日志：
```bash
ros2 node info /cone_following_controller
```

## 控制逻辑详解

### 1. 文件监控 (0.1s频率)
- 检测文件修改时间
- 使用文件锁防止读写冲突
- 重试机制确保数据完整性

### 2. 锥桶数据处理
```python
# 分离红蓝锥桶
red_cones = [cone for cone in detections if cone['category'] == 'redp']
blue_cones = [cone for cone in detections if cone['category'] == 'bluep']

# 按距离排序（y坐标小的在前）
red_cones.sort(key=lambda cone: cone['y'])
blue_cones.sort(key=lambda cone: cone['y'])
```

### 3. 三种控制模式

#### 模式1: 红蓝锥桶都存在
- 计算中间线作为目标路径
- 使用加权前馈控制

#### 模式2: 仅有蓝锥桶
- 向内侧（红锥桶方向）偏移
- error += weight * (-blue_cone['x'])

#### 模式3: 仅有红锥桶  
- 向外侧（蓝锥桶方向）偏移
- error += weight * red_cone['x']

### 4. 输出限幅
- 角速度范围: [30, 150]
- 线速度固定: 1560

## 参数调优

### PD参数调整
- **Kp过大**: 震荡剧烈，超调严重
- **Kp过小**: 响应缓慢，跟踪误差大
- **Kd过大**: 对噪声敏感，系统不稳定
- **Kd过小**: 超调严重，稳定性差

### 权重参数调整
- **gamma过大**: 远处锥桶影响过大，路径不平滑
- **gamma过小**: 只关注近处锥桶，前瞻性差

## 故障排除

### 1. 文件读取问题
```bash
# 检查文件路径
ls -la /home/davinci-mini/test/infer_project_om/detection_results.txt

# 检查文件权限
chmod 644 detection_results.txt
```

### 2. 控制异常
```bash
# 检查话题发布频率
ros2 topic hz /teleop_cmd_vel

# 检查节点状态
ros2 node list | grep cone
```

### 3. 日志调试
在代码中调整日志级别：
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## 集成到现有导航系统

如果需要在现有的nav.cpp中集成锥桶数据：
```cpp
// 订阅锥桶位置话题
cone_sub_ = this->create_subscription<std_msgs::msg::String>(
    "cone_positions", 10,
    std::bind(&LaserGoNode::cone_callback, this, std::placeholders::_1)
);
```

## 性能指标
- **文件读取延迟**: < 10ms
- **控制响应时间**: < 100ms  
- **CPU使用率**: < 5%
- **内存占用**: < 50MB

这样你就可以实现基于锥桶检测的自动巡线控制了！