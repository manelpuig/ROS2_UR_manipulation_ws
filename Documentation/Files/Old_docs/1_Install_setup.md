# 1. Install needed packages:

- be sure you have:
  ````shell
  sudo apt update && sudo apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller
  ````
- create a ur_simulation.repos file in ws:
    ````xml
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
    ```` 
- Delete .git folders:
    ````shell
    dir .\src\Universal_Robots_ROS2_Description -Force
    dir .\src\Universal_Robots_ROS2_Gazebo_Simulation -Force
    Remove-Item -Recurse -Force .\src\Universal_Robots_ROS2_Description\.git
    Remove-Item -Recurse -Force .\src\Universal_Robots_ROS2_Gazebo_Simulation\.git
    git rm --cached -r src/Universal_Robots_ROS2_Description
    git rm --cached -r src/Universal_Robots_ROS2_Gazebo_Simulation
    rosdep update && rosdep install --ignore-src --from-paths . -y
    ````
- Install dependencies:
    ````shell
    git add src/Universal_Robots_ROS2_Description
    git add src/Universal_Robots_ROS2_Gazebo_Simulation
    git commit -m "🎯 Ara sí: afegit el contingut complet dels subrepos"
    git push
    ````
