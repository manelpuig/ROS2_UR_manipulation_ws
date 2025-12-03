# UR5e Kinematics Package (ROS 2 Humble + MoveIt)

This document describes how to create and use a simple ROS 2 package that provides **forward** and **inverse kinematics** tools for a **UR5e** robot using **MoveIt 2**.

The package contains two nodes:

- `ur5e_forward_kinematics_node` – you specify the 6 joint angles → it computes the end–effector pose.
- `ur5e_inverse_kinematics_node` – you specify a target pose (x, y, z, roll, pitch, yaw) → it computes one joint solution.

Both nodes call the standard MoveIt services `/compute_fk` and `/compute_ik`, so **MoveIt’s `move_group` must be running** (e.g., via `ur_moveit_config`).


## 1. Prerequisites

On your Ubuntu 22.04 PC:

- ROS 2 Humble installed.
- UR driver + MoveIt installed (e.g. via your `setup_ur_env.sh`).
- A workspace, e.g.:
  ```bash
  ~/ROS2_UR_manipulation_ws
  ```
---

## 2. Create the package skeleton

In your workspace:

```bash
cd ~/ROS2_UR_manipulation_ws/src

ros2 pkg create ur5e_kinematics_demo --build-type ament_python
```

This creates a Python-based ROS 2 package.

---

## 3. Forward kinematics node


## 4. Inverse kinematics node


## 5. Launch files


### 5.1 Forward kinematics launch: `fk_ur5e.launch.py`


### 5.2 Inverse kinematics launch: `ik_ur5e.launch.py`

## 6. Build the workspace

```bash
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 7. How to run

### 7.1 Forward kinematics

Using Gazebo for simulation:
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
```
Start MoveIt:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```

Compute FK and move for a given joint configuration:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash

ros2 launch ur5e_kinematics_demo fk_ur5e.launch.py \
  joint1:=0.0 joint2:=-1.57 joint3:=1.57 joint4:=0.0 joint5:=1.57 joint6:=0.0 \
  execute:=true
```

### 7.2 Inverse kinematics

Using Gazebo for simulation:
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
```
Start MoveIt:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```
**Improvement**

You can construct a unic launch file for Gazebo and MoveIt (ur5e_sim_moveit.launch.py)

```bash
ros2 launch ur5e_kinematics_demo ur5e_sim_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```

Compute FK and move for a desired pose:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash

ros2 launch ur5e_kinematics_demo ik_ur5e.launch.py \
  target_x:=0.4 target_y:=0.1 target_z:=0.3 \
  target_roll:=0.0 target_pitch:=3.14159 target_yaw:=0.0 \
  execute:=true
```

