cmake_minimum_required(VERSION 3.8)
project(course_teleop_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz
  DESTINATION share/course_teleop_description
)

ament_package()
