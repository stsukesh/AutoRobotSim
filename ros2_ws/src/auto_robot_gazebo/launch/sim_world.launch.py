import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_auto_gazebo = get_package_share_directory('auto_robot_gazebo')

    # World configuration
    world_path = os.path.join(pkg_auto_gazebo, 'worlds', 'robot_arena.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to Gazebo world file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI client'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start simulation paused'
    )

    # Start Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': LaunchConfiguration('paused'),
            'verbose': 'true'
        }.items()
    )

    # Start Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        paused_arg,
        gzserver_cmd,
        gzclient_cmd
    ])
