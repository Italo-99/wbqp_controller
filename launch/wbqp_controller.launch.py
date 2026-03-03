from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('wbqp_controller')
    default_params = os.path.join(pkg, 'config', 'wbqp_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='WBQP parameter YAML file'
        ),
        Node(
            package='wbqp_controller',
            executable='wbqp_controller',
            name='wbqp_controller',
            output='screen',
            parameters=[params_file],
        )
    ])
