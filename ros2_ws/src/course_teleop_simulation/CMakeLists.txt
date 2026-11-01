cmake_minimum_required(VERSION 3.8)
project(course_teleop_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/course_teleop_simulation
)

ament_package()
