cmake_minimum_required(VERSION 3.8)
project(turtlebot3_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

include_directories(include)

add_executable(kf_node src/kf_test.cpp)
ament_target_dependencies(kf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  tf2
  tf2_geometry_msgs
)
target_link_libraries(kf_node Eigen3::Eigen)
install(TARGETS kf_node DESTINATION lib/${PROJECT_NAME})

ament_package()
