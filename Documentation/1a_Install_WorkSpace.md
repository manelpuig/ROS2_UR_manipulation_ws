# Setup Instructions

These are the minimal and clean steps to install and run the UR5e simulation using ROS 2 Humble, Gazebo Classic, and MoveIt2 on Ubuntu 22.04.


Webgraphy:
- https://docs.ros.org/en/humble/p/pymoveit2/index.html

---

## 1. Create a clean workspace

```bash
mkdir -p ~/ROS2_UR_manipulation_ws/src
cd ~/ROS2_UR_manipulation_ws
```

## 2. clone the required repositories
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
This will be done if you clone your forked github repository

## 3. Install the UR ROS2 environment

You will need to source the `setup_ur_env.sh` script to set up the environment variables for Universal Robots packages.

```bash
cd ROS2_UR_manipulation_ws
source setup_ur_env.sh
```
This will:
- Set the Gazebo keys 
- Install the UR packages by APT
- Install dependencies for UR repositories you cloned
- Build the workspace

Verify the `.bashrc` file contains::

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash
cd ROS2_UR_manipulation_ws
```

## 4. Launch the UR5e bringup

Using Gazebo for simulation:
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
```
Or launching the UR5e ROS 2 driver (fake hardware):
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=1.2.3.4 use_fake_hardware:=true launch_rviz:=false
````
> the parameter robot_ip is mandatory but ignored if use_fake_hardware:=true 
---

## 5. Launch MoveIt2 for UR5e

Using Gazebo for simulation:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
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