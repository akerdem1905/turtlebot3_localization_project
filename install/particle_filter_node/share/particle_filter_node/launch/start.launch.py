from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/map.yaml'))

    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')

    pf_pkg_dir = get_package_share_directory('particle_filter_node')
    waypoint_yaml = os.path.join(pf_pkg_dir, 'config', 'waypoints.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'map': map_file, 'use_sim_time': use_sim_time}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    pf_node = Node(
        package='particle_filter_node',
        executable='pf_node',
        name='particle_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    waypoint_node = Node(
        package='particle_filter_node',
        executable='waypoint_navigator_node',
        name='waypoint_navigator',
        output='screen',
        parameters=[waypoint_yaml, {'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        nav2,
        rviz_node,
        pf_node,
        waypoint_node
    ])
