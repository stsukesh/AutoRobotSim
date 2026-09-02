import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('auto_robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_desc = get_package_share_directory('auto_robot_description')

    map_file = os.path.join(pkg_nav, 'maps', 'robot_arena_map.yaml')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'view_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=map_file)
    params_file = LaunchConfiguration('params_file', default=nav2_params_file)
    autostart = LaunchConfiguration('autostart', default='true')

    # Nav2 Bringup
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # RViz2
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=map_file, description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file, description='Full path to nav2 param file'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically start Nav2 lifecycle'),

        nav2_cmd,
        rviz_cmd
    ])
