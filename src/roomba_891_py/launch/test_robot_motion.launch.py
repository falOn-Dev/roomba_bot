from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='roomba_891_py',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen',
            parameters=[{'wheel_radius': 0.0375}, {'wheel_sep': 0.220}]
        )
    ])