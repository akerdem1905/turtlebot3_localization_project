from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),

        Node(
            package='turtlebot3_localization',
            executable='kf_node',
            name='kalman_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
