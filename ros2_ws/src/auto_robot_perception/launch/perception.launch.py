import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_perception = get_package_share_directory('auto_robot_perception')
    params_file = os.path.join(pkg_perception, 'config', 'perception_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_servoing = LaunchConfiguration('enable_servoing', default='false')
    target_color = LaunchConfiguration('target_color', default='red')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('enable_servoing', default_value='false', description='Enable visual servoing controller'),
        DeclareLaunchArgument('target_color', default_value='red', description='Color to track (red/blue/green/yellow)'),

        # Visual Detector Node
        Node(
            package='auto_robot_perception',
            executable='visual_detector_node',
            name='visual_detector_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time, 'target_color': target_color}
            ]
        ),

        # Visual Servoing Controller Node (optional closed loop)
        Node(
            package='auto_robot_perception',
            executable='visual_servoing_node',
            name='visual_servoing_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(enable_servoing)
        )
    ])
