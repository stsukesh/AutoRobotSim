import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('auto_robot_gazebo')
    pkg_localization = get_package_share_directory('auto_robot_localization')
    pkg_perception = get_package_share_directory('auto_robot_perception')
    pkg_navigation = get_package_share_directory('auto_robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # 1. Gazebo Simulation & Robot Spawner
    sim_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'x_pos': '0.0',
            'y_pos': '-1.5',
            'yaw': '1.57'
        }.items()
    )

    # 2. Localization (EKF Sensor Fusion)
    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. OpenCV Visual Perception Detector
    perception_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'target_color': 'red',
            'enable_servoing': 'false'
        }.items()
    )

    # 4. Nav2 Autonomous Navigation (Delayed by 5 seconds to ensure Gazebo & EKF are fully active)
    nav2_stack = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo GUI'),

        sim_spawner,
        ekf_localization,
        perception_pipeline,
        nav2_stack
    ])
