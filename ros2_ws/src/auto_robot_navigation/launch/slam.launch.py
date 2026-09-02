import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('auto_robot_navigation')
    pkg_desc = get_package_share_directory('auto_robot_description')

    slam_params_file = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'view_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        # RViz2 for SLAM mapping
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
