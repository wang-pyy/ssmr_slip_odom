import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ssmr_slip_odom')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the parameters YAML file',
        ),
        DeclareLaunchArgument(
            'run_experiment',
            default_value='false',
            description='Whether to launch the experiment_runner node',
        ),
        DeclareLaunchArgument(
            'run_base',
            default_value='true',
            description='Whether to launch the Yahboom base driver node',
        ),
        DeclareLaunchArgument(
            'run_imu',
            default_value='true',
            description='Whether to launch the BNO055 IMU node',
        ),
        DeclareLaunchArgument(
            'run_slip_odom',
            default_value='true',
            description=(
                'Whether to launch slip_odom_node here. '
                'Set to false when slam_bringup.launch.py already starts it, '
                'to avoid two slip_odom_node instances and TF conflicts.'
            ),
        ),

        Node(
            package='ssmr_slip_odom',
            executable='yahboom_base_node',
            name='yahboom_base_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('run_base')),
        ),

        Node(
            package='ssmr_slip_odom',
            executable='bno055_node',
            name='bno055_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('run_imu')),
        ),

        Node(
            package='ssmr_slip_odom',
            executable='slip_odom_node',
            name='slip_odom_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('run_slip_odom')),
        ),

        Node(
            package='ssmr_slip_odom',
            executable='experiment_runner',
            name='experiment_runner',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('run_experiment')),
        ),
    ])
