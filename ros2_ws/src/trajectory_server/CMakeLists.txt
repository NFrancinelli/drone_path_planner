cmake_minimum_required(VERSION 3.16)
project(trajectory_server LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS-independent core: trajectories and frame conversions.
add_library(trajectory_core src/trajectory.cpp)
target_include_directories(trajectory_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(trajectory_core PUBLIC Eigen3::Eigen)

add_executable(trajectory_server_node src/trajectory_server_node.cpp)
target_link_libraries(trajectory_server_node trajectory_core)
ament_target_dependencies(trajectory_server_node rclcpp px4_msgs nav_msgs)

install(TARGETS trajectory_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS trajectory_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory trajectory_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3)
ament_package()
