import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_auto_description = get_package_share_directory('auto_robot_description')
    pkg_auto_gazebo = get_package_share_directory('auto_robot_gazebo')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='-1.5')
    z_pos = LaunchConfiguration('z_pos', default='0.05')
    yaw = LaunchConfiguration('yaw', default='1.57')
    gui = LaunchConfiguration('gui', default='true')

    xacro_file = os.path.join(pkg_auto_description, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Include world launch
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_auto_gazebo, 'launch', 'sim_world.launch.py')
        ),
        launch_arguments={'gui': gui}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'auto_robot',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-Y', yaw
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('x_pos', default_value='0.0', description='Robot initial X position'),
        DeclareLaunchArgument('y_pos', default_value='-1.5', description='Robot initial Y position'),
        DeclareLaunchArgument('z_pos', default_value='0.05', description='Robot initial Z position'),
        DeclareLaunchArgument('yaw', default_value='1.57', description='Robot initial Yaw orientation'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),

        world_cmd,
        robot_state_publisher,
        spawn_entity
    ])
