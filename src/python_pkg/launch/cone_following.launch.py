from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 默认检测结果文件路径
    default_detection_file = '/home/davinci-mini/test/infer_project_om/detection_results.txt'
    
    # 声明启动参数
    detection_file_arg = DeclareLaunchArgument(
        'detection_file_path',
        default_value=default_detection_file,
        description='Path to the detection_results.txt file'
    )
    
    # 检测结果处理和巡锥桶控制节点
    cone_following_node = Node(
        package='python_pkg',
        executable='file_reader_python',
        name='cone_following_controller',
        parameters=[{
            'file_path': LaunchConfiguration('detection_file_path')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        detection_file_arg,
        cone_following_node
    ])