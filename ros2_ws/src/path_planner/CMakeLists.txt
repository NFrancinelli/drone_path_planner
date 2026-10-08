cmake_minimum_required(VERSION 3.16)
project(path_planner LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
# Unoptimized A* is ~25x slower (seconds instead of ~0.3 s per plan), so default to Release.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_server REQUIRED)

# ROS-independent core: search and smoothing on an abstract voxel map.
add_library(planner_core src/astar.cpp src/inflated_map.cpp src/smoothing.cpp)
target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_core PUBLIC Eigen3::Eigen trajectory_server::trajectory_core)

add_executable(planner_node src/planner_node.cpp)
target_include_directories(planner_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(planner_node planner_core ${OCTOMAP_LIBRARIES})
ament_target_dependencies(planner_node rclcpp geometry_msgs nav_msgs octomap_msgs tf2_ros)

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar planner_core)
  ament_add_gtest(test_smoothing test/test_smoothing.cpp)
  target_link_libraries(test_smoothing planner_core)
  ament_add_gtest(test_octomap_map test/test_octomap_map.cpp)
  target_include_directories(test_octomap_map PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_octomap_map planner_core ${OCTOMAP_LIBRARIES})
endif()

ament_package()
