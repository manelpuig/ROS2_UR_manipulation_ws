# UR5e Kinematics Package (ROS 2 Humble + MoveIt)

This document describes how to create and use a simple ROS 2 package that provides **forward** and **inverse kinematics** tools for a **UR5e** robot using **MoveIt 2**.

The package contains two nodes:

- `ur5e_forward_kinematics_node` – you specify the 6 joint angles → it computes the end–effector pose.
- `ur5e_inverse_kinematics_node` – you specify a target pose (x, y, z, roll, pitch, yaw) → it computes one joint solution.

Both nodes call the standard MoveIt services `/compute_fk` and `/compute_ik`, so **MoveIt’s `move_group` must be running** (e.g., via `ur_moveit_config`).



## Create the package skeleton

In your workspace:

```bash
cd ~/ROS2_UR_manipulation_ws/src

ros2 pkg create ur5e_kinematics_demo --build-type ament_python
```

This creates a Python-based ROS 2 package.

## Build the workspace

```bash
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
source install/setup.bash
```

---

### Forward kinematics

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
ros2 launch ur5e_kinematics_demo ur5e_forward_kinematics.launch.py joints:="[0.0, -1.57, 1.57, 0.0, 1.57, 0.0]" execute:=true
```

### Inverse kinematics

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
ros2 launch ur5e_kinematics_demo ur5e_inverse_kinematics.launch.py target_xyz:="[0.4, 0.0, 0.3]" target_rpy:="[0.0, 3.14159, 0.0]" execute:=true
```
> You have to install

### Go to Pose

Using Gazebo and MoveIt for simulation:

```bash
ros2 launch ur5e_kinematics_demo ur5e_sim_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```

Compute FK and move for a desired pose:

```bash
ros2 launch ur5e_kinematics_demo ur5e_move_to_pose.launch.py target_xyz:="[0.45, 0.10, 0.25]" target_rpy:="[0.0, 3.14159, 0.0]" execute:=true
```
> You have to install `spatialmath` lib:
````python
python3 -m pip install --user spatialmath-python
````

Pick and Place
  ```bash
  ros2 launch ur5e_kinematics_demo pick_place.launch.py pick_xyz:="[0.45, 0.10, 0.12]" place_xyz:="[0.35, -0.25, 0.12]"
  ```