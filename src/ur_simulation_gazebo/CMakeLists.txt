cmake_minimum_required(VERSION 3.10)
project(ur_simulation_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
