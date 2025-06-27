cmake_minimum_required(VERSION 3.5)
project(vision_to_mavros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(vision_to_mavros_node src/vision_to_mavros.cpp)
ament_target_dependencies(vision_to_mavros_node
  rclcpp tf2 tf2_ros geometry_msgs nav_msgs mavros_msgs
)

install(TARGETS vision_to_mavros_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
