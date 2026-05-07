# UR5e Kinematics Control

This document describes how to create and use a simple ROS 2 package that provides **forward kinematics** for a **UR5e** robot

The package contains two nodes:

- `ur5e_joint_target` – you specify the 6 joint angles.
- `ur5e_joint_targets` – you specify the 6 joint angles for multiple waypoints.


## Create the package

In your workspace:

```bash
cd ~/ROS2_UR_manipulation_ws/src
ros2 pkg create ur5e_kinematics_demo --build-type ament_python
```

This creates a Python-based ROS 2 package.

## Build the workspace

```bash
cd ~/ROS2_UR_manipulation_ws
colcon build 
source install/setup.bash
```

### Forward kinematics for 1 joint target

Using Gazebo Classic for simulation:
- Bringup UR5e on Gazebo:
  ```bash
  ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
  ```
- Move the robot to a desired joint configuration: 
  ```bash
  ros2 launch ur5e_kinematics_control ur5e_joint_target.launch.py target_deg:="[0.0, -90.0, 90.0, 0.0, 90.0, 0.0]" time_sec:=5.0 controller_topic:=/joint_trajectory_controller/joint_trajectory
  ```
In a real robot UR5e:
- Run the UR driver:
  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.4 launch_rviz:=false
  ```
- Move the robot to a desired joint configuration: 
  ```bash
  ros2 launch ur5e_kinematics_control ur5e_joint_target.launch.py target_deg:="[0.0, -90.0, 90.0, 0.0, 90.0, 0.0]" time_sec:=5.0 controller_topic:=/scaled_joint_trajectory_controller/joint_trajectory
  ```

### Forward kinematics for multiple joint targets

An extension of the previous node allows to specify multiple joint targets (waypoints) and the time to reach each waypoint (the time is the accumulated time, not the time to execute each movement!).

Using Gazebo Classic for simulation:
- Bringup UR5e on Gazebo:
  ```bash
  ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
  ```
- Move the robot to a desired joint configuration: 
  ```bash
  ros2 launch ur5e_kinematics_control ur5e_joint_targets.launch.py controller_topic:=/joint_trajectory_controller/joint_trajectory trajectory_file:=trajectory.yaml
  ```
In a real robot UR5e:
- Run the UR driver:
  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.4 launch_rviz:=false
  ```
- Move the robot to a desired joint configuration: 
  ```bash
  ros2 launch ur5e_kinematics_control ur5e_joint_targets.launch.py controller_topic:=/scaled_joint_trajectory_controller/joint_trajectory trajectory_file:=trajectory.yaml
  ```