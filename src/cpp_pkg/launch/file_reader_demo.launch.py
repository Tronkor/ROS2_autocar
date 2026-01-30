from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取工作空间路径
    workspace_path = os.path.join(os.path.expanduser('~'), 'demo_ws')
    default_file_path = os.path.join(workspace_path, 'sample_data.txt')
    
    # 声明启动参数
    file_path_arg = DeclareLaunchArgument(
        'file_path',
        default_value=default_file_path,
        description='Path to the text file to read'
    )
    
    # C++ 节点
    cpp_node = Node(
        package='cpp_pkg',
        executable='file_reader_cpp',
        name='file_reader_cpp_node',
        parameters=[{'file_path': LaunchConfiguration('file_path')}],
        output='screen'
    )
    
    # Python 节点
    python_node = Node(
        package='python_pkg',
        executable='file_reader_python',
        name='file_reader_python_node',
        parameters=[{'file_path': LaunchConfiguration('file_path')}],
        output='screen'
    )
    
    return LaunchDescription([
        file_path_arg,
        cpp_node,
        python_node
    ])