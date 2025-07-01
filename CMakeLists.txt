cmake_minimum_required(VERSION 3.8)
project(particle_filter_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(pf_node src/pf.cpp)
ament_target_dependencies(pf_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
)

add_executable(kf_node src/kf.cpp)
target_link_libraries(kf_node Eigen3::Eigen)
ament_target_dependencies(kf_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
)

add_executable(ekf_node src/ekf.cpp)
target_link_libraries(ekf_node Eigen3::Eigen)
ament_target_dependencies(ekf_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_geometry_msgs
)

add_executable(pattern_node src/pattern.cpp)
ament_target_dependencies(pattern_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  pf_node
  kf_node
  ekf_node
  pattern_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installiere nur config, wenn sie wirklich verwendet wird
# Falls "launch" noch verwendet wird, diesen Block ggf. anpassen oder entfernen
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
