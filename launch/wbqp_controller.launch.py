from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('wbqp_controller')
    params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='wbqp_controller',
            executable='wbqp_controller',
            name='wbqp_controller',
            output='screen',
            parameters=[params],
        )
    ])
