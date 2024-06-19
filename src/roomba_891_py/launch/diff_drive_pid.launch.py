from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    differential_drive_controller = Node(
        package='roomba_891_py',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[{'wheel_radius': 0.0375}, {'wheel_sep': 0.220}]
    )

    left_wheel_controller = Node(
        package='roomba_891_py',
        executable='pid_controller',
        name='left_wheel_controller',
        output='screen',
        parameters=[{'kp': 0.1}, {'ki': 0.01}, {'kd': 0.01}, {'setpoint_topic': '/left_wheel_velocity'}]
    )

    right_wheel_controller = Node(
        package='roomba_891_py',
        executable='pid_controller',
        name='right_wheel_controller',
        output='screen',
        parameters=[{'kp': 0.1}, {'ki': 0.01}, {'kd': 0.01}, {'setpoint_topic': '/right_wheel_velocity'}]
    )

    return LaunchDescription([
        differential_drive_controller,
        left_wheel_controller,
        right_wheel_controller
    ])
