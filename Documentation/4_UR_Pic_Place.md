# UR5e Pick and Place

This document describes how to.
- Move joints
````bash
ros2 launch ur5e_kinematics_demo move_joints.launch.py \
  joints:="[0.2, -1.3, 1.4, -0.5, 1.2, 0.0]" \
  use_sim_time:=true
````
- Move to pose
````bash
ros2 launch ur5e_kinematics_demo move_to_pose.launch.py \
  target_xyz:="[0.45, 0.10, 0.25]" \
  target_rpy:="[0.0, 3.14159, 0.0]"
````
- Pick and Place
  ```bash
  ros2 launch ur5e_kinematics_demo pick_place.launch.py \
  pick_xyz:="[0.45, 0.10, 0.12]" \
  place_xyz:="[0.35, -0.25, 0.12]"
  ```
