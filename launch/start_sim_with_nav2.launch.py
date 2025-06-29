from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/turtlebot3_localization_project/maps/map.yaml'))

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Navigation2 Launch (inkl. AMCL, map_server, lifecycle manager)
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    # Partikelfilter-Node (auskommentiert für Kalman-Test)
    # particle_filter_node = Node(
    #     package='turtlebot3_localization',
    #     executable='particle_filter_node',
    #     name='particle_filter_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # Kalman-Filter Node (aktiv für aktuellen Test)
    kalman_filter_node = Node(
        package='turtlebot3_localization',
        executable='kf_node',
        name='kf_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/turtlebot3_localization_project/maps/map.yaml')),
        gazebo,
        nav2,
        kalman_filter_node  # <--- Jetzt aktiv
    ])
