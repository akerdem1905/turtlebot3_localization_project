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
find_package(nav2_msgs REQUIRED)

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

add_executable(waypoint_navigator_node src/waypoint_navigator.cpp)
ament_target_dependencies(waypoint_navigator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  pf_node
  waypoint_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()