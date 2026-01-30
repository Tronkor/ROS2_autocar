# ROS2 文件读取节点示例

本示例演示了如何在ROS2中使用C++和Python节点读取.txt文件。

## 文件结构

```
demo_ws/
├── src/
│   ├── cpp_pkg/
│   │   ├── src/
│   │   │   ├── file_reader_cpp.cpp    # C++文件读取节点
│   │   │   └── nav.cpp                # 原有导航节点
│   │   ├── launch/
│   │   │   └── file_reader_demo.launch.py  # 启动文件
│   │   └── CMakeLists.txt
│   └── python_pkg/
│       └── python_pkg/
│           └── file_reader_python.py  # Python文件读取节点
└── sample_data.txt                    # 示例数据文件
```

## 编译

```bash
cd ~/demo_ws
colcon build
source install/setup.bash
```

## 运行方式

### 1. 分别运行节点

运行C++节点：
```bash
ros2 run cpp_pkg file_reader_cpp --ros-args -p file_path:=/path/to/your/file.txt
```

运行Python节点：
```bash
ros2 run python_pkg file_reader_python --ros-args -p file_path:=/path/to/your/file.txt
```

### 2. 使用Launch文件同时启动

```bash
ros2 launch cpp_pkg file_reader_demo.launch.py file_path:=/path/to/your/file.txt
```

### 3. 使用示例文件

```bash
# 使用提供的示例文件
ros2 launch cpp_pkg file_reader_demo.launch.py file_path:=$(pwd)/sample_data.txt
```

## 查看输出

查看发布的文件内容：
```bash
ros2 topic echo /file_content
```

查看节点日志：
```bash
ros2 node list
ros2 node info /file_reader_cpp_node
ros2 node info /file_reader_python_node
```

## 功能特点

### C++节点特点：
- 使用 `std::ifstream` 进行文件读取
- 提供多种读取方法（逐行读取、整文件读取）
- 完善的错误处理
- 参数配置支持

### Python节点特点：
- 使用Python内置 `open()` 函数
- 支持UTF-8编码
- 路径存在性检查
- 异常处理机制

### 共同特点：
- 定时读取文件（每5秒一次）
- 通过ROS参数配置文件路径
- 发布文件内容到 `/file_content` 话题
- 详细的日志输出

## 自定义使用

1. 修改文件路径参数
2. 调整读取频率（修改timer间隔）
3. 更改话题名称
4. 添加文件格式解析逻辑

## 注意事项

- 确保文件路径正确且文件可读
- 注意文件编码格式（建议UTF-8）
- 大文件读取时注意内存使用
- 可以通过参数动态修改文件路径