"""
SLAM bringup (proposed): RPLIDAR A1 + slip_odom_node + slam_toolbox.

TF tree:
    map -> odom         (slam_toolbox)
    odom -> base_link   (slip_odom_node, publish_tf=True)
    base_link -> laser_frame  (static_transform_publisher)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('ssmr_slip_odom')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    default_slip_params = os.path.join(pkg_share, 'config', 'slip_odom_params.yaml')
    default_rviz_cfg = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # ------------------------------------------------------------------
    # LaunchArguments
    # ------------------------------------------------------------------
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slip_odom = LaunchConfiguration('use_slip_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    slam_params_file = LaunchConfiguration('slam_params_file')
    slip_params_file = LaunchConfiguration('slip_params_file')

    declared_args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Serial port for RPLIDAR A1'),
        # RPLIDAR A1 high-speed FW expects 256000.
        # NOTE: if 256000 fails to start (no scan), switch to 115200.
        DeclareLaunchArgument('serial_baudrate', default_value='256000',
                              description='RPLIDAR A1 baudrate (256000 high-speed; fallback 115200)'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame',
                              description='Laser TF frame_id'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Topic for LaserScan output'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch rviz2 (avoid on Pi 5 headless)'),
        DeclareLaunchArgument('use_slip_odom', default_value='true',
                              description='Launch slip_odom_node from this package'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),

        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.12'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params,
                              description='slam_toolbox params YAML'),
        DeclareLaunchArgument('slip_params_file', default_value=default_slip_params,
                              description='slip_odom_node params YAML'),
    ]

    # ------------------------------------------------------------------
    # RPLIDAR A1
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Static TF: base_link -> laser_frame
    # ------------------------------------------------------------------
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', lidar_x,
            '--y', lidar_y,
            '--z', lidar_z,
            '--roll', lidar_roll,
            '--pitch', lidar_pitch,
            '--yaw', lidar_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', frame_id,
        ],
    )

    # ------------------------------------------------------------------
    # slip_odom_node (publishes /odom and odom -> base_link)
    # ------------------------------------------------------------------
    slip_odom_node = Node(
        package='ssmr_slip_odom',
        executable='slip_odom_node',
        name='slip_odom_node',
        output='screen',
        parameters=[slip_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_slip_odom),
    )

    # ------------------------------------------------------------------
    # slam_toolbox async online mapper
    # ------------------------------------------------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', scan_topic)],
    )

    # ------------------------------------------------------------------
    # Optional RViz
    # ------------------------------------------------------------------
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
            slip_odom_node,
            slam_toolbox_node,
            rviz_node,
        ]),
    ])
