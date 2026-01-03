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

Is good practice to kill remaining process before starting a new moveit2 instance:
````shell
cd ~/Ros2_UR_manipulation_ws
chmod +x kill_ur_sim_moveit.sh
./kill_ur_sim_moveit.sh
````

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
ros2 launch ur5e_kinematics_demo ur5e_inverse_kinematics.launch.py target_xyz:="[0.4, 0.0, 0.2]" target_rpy:="[1.8, 0.0, 1.57]" execute:=true
```
> You have to install

### Go to Pose

The minimal proposed solution is based on a node that:
- converts the target pose into a quaternion, 
- calls MoveIt’s /compute_ik service to get a joint solution, 
- and then executes that joint goal with move_to_configuration()

Using Gazebo and MoveIt for simulation:

```bash
ros2 launch ur5e_kinematics_demo ur5e_sim_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```

Compute FK and move for a desired pose:

```bash
ros2 launch ur5e_kinematics_demo ur5e_move_to_pose.launch.py target_xyz:="[0.45, 0.10, 0.25]" target_rpy:="[0.0, 0.0, 0.0]" execute:=true
```
> You have to install `spatialmath` lib:
````python
python3 -m pip install --user spatialmath-python
````

### Pick and Place

This program is configured with:
- The robot first moves to a known home joint configuration.

- Each pick-and-place step is defined as a target pose (position + orientation).

- For every pose:

  - MoveIt’s /compute_ik service is called to compute joint angles.

  - The solution is seeded with joints close to the current posture to keep the same IK branch.

  - The joint goal is executed with move_to_configuration().

- The tool orientation is set to look downwards using
pitch ≈ 3.10 rad (instead of π) to avoid numerical edge cases.

- Pick and place poses are chosen close to the home posture to ensure feasibility and smooth motion.

  ```bash
  ros2 launch ur5e_kinematics_demo ur5e_pick_place.launch.py execute:=true sleep_sec_between_steps:=0.5
  ````