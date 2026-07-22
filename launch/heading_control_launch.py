import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('libbno055_linux')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'bno055_params.yaml'])

    # Launch arguments
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to use ROS 2 Composable Node Container for Zero-Copy Transport.'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to BNO055 YAML parameters file.'
    )

    kp_arg = DeclareLaunchArgument('kp', default_value='0.05', description='Heading PID Proportional Gain')
    ki_arg = DeclareLaunchArgument('ki', default_value='0.001', description='Heading PID Integral Gain')
    kd_arg = DeclareLaunchArgument('kd', default_value='0.01', description='Heading PID Derivative Gain')

    use_composition = LaunchConfiguration('use_composition')
    params_file = LaunchConfiguration('params_file')
    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')

    # -------------------------------------------------------------
    # 1. Zero-Copy Composable Node Container (Production High-Perf)
    # -------------------------------------------------------------
    composable_container = ComposableNodeContainer(
        name='bno055_heading_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='libbno055_linux',
                plugin='bno055_ros2::BNO055PublisherNode',
                name='bno055_publisher_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='libbno055_linux',
                plugin='bno055_ros2::BNO055HeadingControlNode',
                name='bno055_heading_control_node',
                parameters=[{
                    'kp': kp,
                    'ki': ki,
                    'kd': kd,
                    'imu_topic': 'imu/data',
                    'cmd_vel_in_topic': 'cmd_vel_in',
                    'cmd_vel_out_topic': 'cmd_vel',
                    'enable_diagnostics': True
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' == 'true'"]))
    )

    # -------------------------------------------------------------
    # 2. Standalone Processes (Fallback Mode)
    # -------------------------------------------------------------
    standalone_imu_node = Node(
        package='libbno055_linux',
        executable='bno055_publisher_node',
        name='bno055_publisher_node',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' != 'true'"]))
    )

    standalone_heading_node = Node(
        package='libbno055_linux',
        executable='bno055_heading_control_node',
        name='bno055_heading_control_node',
        parameters=[{
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'imu_topic': 'imu/data',
            'cmd_vel_in_topic': 'cmd_vel_in',
            'cmd_vel_out_topic': 'cmd_vel',
            'enable_diagnostics': True
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' != 'true'"]))
    )

    return LaunchDescription([
        use_composition_arg,
        params_file_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        composable_container,
        standalone_imu_node,
        standalone_heading_node
    ])
