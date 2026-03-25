cmake_minimum_required(VERSION 3.8)
project(my_ur_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()