import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_name = 'roomba_891_py'
    file_subpath = 'description/robot_base.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                    launch_arguments={
                        'pause' : 'true',
                        'gz_args' : 'empty.sdf',
                    }.items(),
                )
    
    node_gz_bridge = Node(package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        ]
                    )
    
    create_entity = Node(package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', '/robot_description',
                                '-entity', 'robot'],
                    output='screen')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}, {'use_sim_time': True}] # add other parameters here if required
    )

    


    return LaunchDescription([
        gazebo,
        create_entity,
        node_gz_bridge,
        node_robot_state_publisher,
    ])