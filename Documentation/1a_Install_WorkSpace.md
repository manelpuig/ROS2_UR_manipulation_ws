# Setup Instructions

These are the minimal and clean steps to install and run the UR5e simulation using ROS 2 Humble, Gazebo Classic, and MoveIt2 on Ubuntu 22.04.

---

## 1. Create a clean workspace

```bash
mkdir -p ~/ROS2_UR_manipulation_ws/src
cd ~/ROS2_UR_manipulation_ws
```

---

## 2. Install system dependencies and Gazebo keys

Gazebo Classic repositories need valid GPG keys. Install everything with:

```bash
sudo apt update
sudo apt install -y lsb-release curl gnupg
sudo curl -sSL http://get.gazebosim.org | sh
```

---

## 3. Install ROS 2 packages from APT

These packages provide UR descriptions, drivers, MoveIt2 and Gazebo integration.

```bash
sudo apt install -y \
  ros-humble-ur \
  ros-humble-ur-description \
  ros-humble-ur-msgs \
  ros-humble-ur-robot-driver \
  ros-humble-ur-bringup \
  ros-humble-ur-moveit-config \
  ros-humble-moveit \
  ros-humble-moveit-ros \
  ros-humble-moveit-planners-ompl \
  gazebo \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control
```

---

## 4. Clone the UR simulation package (source)

```bash
cd ~/ROS2_UR_manipulation_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/.git
```
You have to delete the .git folder:

```bash
rm -rf Universal_Robots_ROS2_Gazebo_Simulation/.git
rm -rf Universal_Robots_ROS2_Tutorials/.git
```
---

## 5. Install dependencies and build

```bash
cd ~/ROS2_UR_manipulation_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

Verify the `.bashrc` file contains::

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash
cd ROS2_UR_manipulation_ws
```

---

## 6. Launch the UR5e bringup

Using Gazebo for simulation:
```bash
ros2 launch ur_simulation_gazebo ur_simulation.launch.py ur_type:=ur5e
```
Or launching the UR5e ROS 2 driver (fake hardware):
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=1.2.3.4 use_fake_hardware:=true launch_rviz:=false
````
> the parameter robot_ip is mandatory but ignored if use_fake_hardware:=true 
---

## 7. Launch MoveIt2 for UR5e

Using Gazebo for simulation:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```
---

This setup provides a stable UR5e simulation environment with ROS 2 Humble and Gazebo Classic.

🔹 1. Universal_Robots_ExternalControl_URCap (per robot real)

Això no és un package ROS2, és una URCap que va al robot, però és clau per treballar amb UR real + ROS2.

Repo: UniversalRobots/Universal_Robots_ExternalControl_URCap (el baixes, el poses al robot des de PolyScope).

Serveix per:

Definir el programa “External Control” al robot.

Deixar que el PC amb ROS2 controli el robot via el driver.

👉 Resum: imprescindible si vols controlar el UR5e real des de ROS2.