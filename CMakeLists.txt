cmake_minimum_required(VERSION 3.5)
project(turtlebot3_localization)

# Standard C++ Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
)

# Executable: main_filter_node (inkl. KalmanFilter)
add_executable(main_filter_node
  src/main_filter_node.cpp
  src/kalman_filter.cpp
)

ament_target_dependencies(main_filter_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
)

target_link_libraries(main_filter_node
  Eigen3::Eigen
)

add_executable(ekf_node
  src/ekf_node.cpp
  src/extended_kalman_filter.cpp
)

ament_target_dependencies(ekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_geometry_msgs
  Eigen3
)

target_link_libraries(ekf_node Eigen3::Eigen)

install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
)


# Installation
install(TARGETS
  main_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

