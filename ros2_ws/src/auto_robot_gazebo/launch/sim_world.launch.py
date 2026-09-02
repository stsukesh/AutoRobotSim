import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auto_gazebo = get_package_share_directory('auto_robot_gazebo')

    # World configuration
    world_path = os.path.join(pkg_auto_gazebo, 'worlds', 'robot_arena.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to Gazebo Harmonic world SDF file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI client'
    )

    # Start Gazebo Harmonic Sim via ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        gz_sim
    ])
