"""
SLAM bringup (baseline): RPLIDAR A1 + simple_encoder_odom_node + slam_toolbox.

This is the comparison run for the proposed slip-aware odometry.
Only ONE odom -> base_link broadcaster is active: simple_encoder_odom_node.
slip_odom_node MUST NOT be launched here, or TF will conflict.

TF tree:
    map -> odom         (slam_toolbox)
    odom -> base_link   (simple_encoder_odom_node, publish_tf=True)
    base_link -> laser_frame  (static_transform_publisher)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ssmr_slip_odom')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    default_rviz_cfg = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    slam_params_file = LaunchConfiguration('slam_params_file')
    wheel_base = LaunchConfiguration('wheel_base')

    declared_args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        # If 256000 fails to bring up the A1, fall back to 115200.
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.12'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),
        DeclareLaunchArgument('wheel_base', default_value='0.13'),
    ]

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('scan', scan_topic)],
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
            '--roll', lidar_roll, '--pitch', lidar_pitch, '--yaw', lidar_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', frame_id,
        ],
    )

    baseline_odom_node = Node(
        package='ssmr_slip_odom',
        executable='simple_encoder_odom_node',
        name='simple_encoder_odom_node',
        output='screen',
        parameters=[{
            'wheel_base': wheel_base,
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic': '/odom',
            'wheel_speeds_topic': '/wheel_speeds',
            'use_sim_time': use_sim_time,
        }],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', scan_topic)],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_cfg],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declared_args + [
        GroupAction([
            rplidar_node,
            static_tf_laser,
            baseline_odom_node,
            slam_toolbox_node,
            rviz_node,
        ]),
    ])
