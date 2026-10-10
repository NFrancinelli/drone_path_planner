cmake_minimum_required(VERSION 3.16)
project(drone_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mission_metrics.py DESTINATION lib/${PROJECT_NAME})

ament_package()
