from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Definiert, ob die Simulationszeit (z. B. aus Gazebo) genutzt wird
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Pfad zur zu ladenden Karte (für Navigation)
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/map.yaml'))

    # === 1. Starte die Simulationsumgebung (Gazebo) mit TurtleBot3 ===
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # === 2. Starte den Navigation Stack (Nav2) mit Karte und SLAM/AMCL ===
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # === 3. Lade RViz-Konfiguration für Visualisierung ===
    pf_node_pkg = FindPackageShare('particle_filter_node').find('particle_filter_node')
    rviz_config_path = os.path.join(pf_node_pkg, 'rviz', 'robot_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # === 4. Starte die Lokalisierungs-Algorithmen ===

    # 4.1 Particle Filter (Monte-Carlo Localization)
    particle_filter_node = Node(
        package='particle_filter_node',
        executable='pf_node',
        name='particle_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4.2 Klassischer Kalman-Filter (für lineare Zustandsmodelle)
    kalman_filter_node = Node(
        package='particle_filter_node',
        executable='kf_node',
        name='kalman_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4.3 Erweiterter Kalman-Filter (für nichtlineare Modelle, z. B. Robotik)
    ekf_node = Node(
        package='particle_filter_node',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # === 5. Steuerungsmuster für das Bewegungsverhalten des Roboters ===
    pattern_node = Node(
        package='particle_filter_node',
        executable='pattern_node',
        name='pattern_driver',
        output='screen',
        parameters=[{
            'pattern': 'wave',            # Auswahl des Bewegungsmusters: 'wave', 'circle', 'circle_straight_circle'
            'linear_speed': 0.2,          # Vorwärtsgeschwindigkeit [m/s]
            'angular_speed': 0.5,         # Rotationsgeschwindigkeit [rad/s]
            'duration': 25.0,             # Gesamtdauer des Musters in Sekunden
            'use_sim_time': True
        }]
    )

    # === Rückgabe der vollständigen Launch-Beschreibung ===
    return LaunchDescription([
        gazebo,
        nav2,
        particle_filter_node,
        kalman_filter_node,
        ekf_node,
        pattern_node,
        rviz_node
    ])
