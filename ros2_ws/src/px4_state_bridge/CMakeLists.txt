cmake_minimum_required(VERSION 3.16)
project(px4_state_bridge LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_server REQUIRED)

add_executable(px4_state_bridge_node src/px4_state_bridge_node.cpp)
target_link_libraries(px4_state_bridge_node trajectory_server::trajectory_core)
ament_target_dependencies(px4_state_bridge_node
  rclcpp px4_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros)

install(TARGETS px4_state_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
