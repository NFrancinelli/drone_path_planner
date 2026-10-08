cmake_minimum_required(VERSION 3.16)
project(drone_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
