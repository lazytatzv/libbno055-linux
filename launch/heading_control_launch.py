import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('libbno055_linux')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'bno055_params.yaml'])

    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to BNO055 YAML parameters file.'
    )

    kp_arg = DeclareLaunchArgument('kp', default_value='0.05', description='Heading PID Proportional Gain')
    ki_arg = DeclareLaunchArgument('ki', default_value='0.001', description='Heading PID Integral Gain')
    kd_arg = DeclareLaunchArgument('kd', default_value='0.01', description='Heading PID Derivative Gain')

    # 1. BNO055 Core IMU Driver Node (Publishes /imu/data)
    imu_driver_node = Node(
        package='libbno055_linux',
        executable='bno055_publisher_node',
        name='bno055_publisher_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    # 2. Twist-In / Twist-Out Heading PID Corrector Node
    # Subscribes to raw input velocity (cmd_vel_in) & /imu/data -> Publishes corrected velocity (cmd_vel)
    heading_control_node = Node(
        package='libbno055_linux',
        executable='bno055_heading_control_node',
        name='bno055_heading_control_node',
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'imu_topic': 'imu/data',
            'cmd_vel_in_topic': 'cmd_vel_in',
            'cmd_vel_out_topic': 'cmd_vel'
        }],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        imu_driver_node,
        heading_control_node
    ])
