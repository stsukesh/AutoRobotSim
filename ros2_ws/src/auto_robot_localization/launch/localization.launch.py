import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_localization = get_package_share_directory('auto_robot_localization')
    ekf_config_path = os.path.join(pkg_localization, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot Localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        # Odometry Evaluator Node
        Node(
            package='auto_robot_localization',
            executable='odom_evaluator.py',
            name='odom_evaluator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_name': 'auto_robot'}]
        )
    ])
