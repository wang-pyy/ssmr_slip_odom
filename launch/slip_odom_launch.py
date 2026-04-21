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

        Node(
            package='ssmr_slip_odom',
            executable='slip_odom_node',
            name='slip_odom_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
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
