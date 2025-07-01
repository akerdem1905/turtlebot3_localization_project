from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/map.yaml'))

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    # Pfad zur vorbereiteten RViz-Datei
    pf_node_pkg = FindPackageShare('particle_filter_node').find('particle_filter_node')
    rviz_config_path = os.path.join(pf_node_pkg, 'rviz', 'robot_view.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'rviz': 'false' # 🚫 Deaktiviert das zusätzliche RViz
        }.items()
    )

    particle_filter_node = Node(
        package='particle_filter_node',
        executable='pf_node',
        name='particle_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    kalman_filter_node = Node(
        package='particle_filter_node',
        executable='kf_node',
        name='kalman_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ekf_node = Node(
        package='particle_filter_node',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    pattern_node = Node(
        package='particle_filter_node',
        executable='pattern_node',
        name='pattern_driver',
        output='screen',
        parameters=[{
            'pattern': 'zigzag',
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'duration': 25.0,
            'use_sim_time': True
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.expanduser('~/ProLab/rviz/robot_view.rviz')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        nav2,
        particle_filter_node,
        kalman_filter_node,
        ekf_node,
        pattern_node,
        rviz_node
    ])
