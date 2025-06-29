from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_localization',
            executable='main_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
