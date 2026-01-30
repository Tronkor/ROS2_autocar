from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 默认检测结果文件路径
    default_detection_file = os.path.join(os.path.expanduser('~'), 'demo_ws', 'detection_results.txt')
    
    # 声明启动参数
    detection_file_arg = DeclareLaunchArgument(
        'detection_file_path',
        default_value=default_detection_file,
        description='Path to the detection_results.txt file'
    )
    
    # 检测结果处理节点
    detection_processor_node = Node(
        package='python_pkg',
        executable='detection_processor',
        name='detection_processor',
        parameters=[{
            'detection_file_path': LaunchConfiguration('detection_file_path')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        detection_file_arg,
        detection_processor_node
    ])