README – Lokalisation & Filtervergleich (ROS 2 Jazzy)
====================================================

Dieses Projekt implementiert und vergleicht drei Lokalisationsverfahren:
- Particle Filter
- Kalman Filter
- Extended Kalman Filter (EKF)

1. Build & Setup
----------------
colcon build --packages-select particle_filter_node
source install/setup.bash

2. Launch Simulation & Filter-Nodes
-----------------------------------
ros2 launch particle_filter_node start.launch.py

Dies startet:
- Gazebo-Simulation
- Nav2
- RViz2
- Alle drei Lokalisierungsfilter
- Bewegungsmuster über pattern_driver

3. Aufzeichnen der ROS 2 Topics
-------------------------------
In einem neuen Terminal:

source install/setup.bash
ros2 bag record /odom /prediction /kf_prediction /ekf_state

Diese Daten werden später zur RMSE-Berechnung herangezogen.

4. RMSE-Analyse aus Rosbag
--------------------------
source venv/bin/activate       # Aktiviere Python-Virtual-Environment
python3 extract_from_bag.py    # Extrahiere .csv-Dateien aus .mcap-Bagfile
python3 RMSE.py                # Plotte und berechne RMSE-Werte

5. Ausgabeformate
-----------------
- odom.csv: Ground-Truth aus Gazebo
- prediction.csv: Ausgabe des Particle Filters
- kf_prediction.csv: Ausgabe des Kalman Filters
- ekf_state.csv: Ausgabe des EKF
- Visualisierung: Plot der Trajektorien und RMSE-Werte


