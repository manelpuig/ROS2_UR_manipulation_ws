# Connecting and Controlling a Real UR5e Robot with ROS 2 Humble + MoveIt

This document summarizes the necessary steps to connect a **real Universal Robots UR5e** to a **PC running Ubuntu 22.04 + ROS 2 Humble**, install the required URCaps, configure networking, and control the robot using **MoveIt**.

---

## 1. Network Setup

Ensure the PC and UR5e are on the same LAN.

Example:
- PC IP: `192.168.1.45`
- UR5e IP: `192.168.1.4`

Verify connection:
```bash
ping 192.168.1.4
```

## 2. UR5e robot Configuration

There are requirements on Polyscope software version and URcap external control

### 2.1. Polyscope software
To properly work on ros2 Humble, the Polyscope version has to be higher than 5.9.5. We have installed the 5.25.1 version.

The file we have to download is: https://www.universal-robots.com/download/software-ur-series/update/latest-polyscope-software-update-sw-5251-ur-series-e-series/

### 2.2. URCap externalcontrol
Download:
```
externalcontrol-1.0.urcap
```
From:
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble/ur_robot_driver/resources

**Install using Teach Pendant**
- Copy `.urcap` file to USB (FAT32)
- On the teach pendant:
   - **Settings → System → URCaps → Manage**
   - **Add** the file from USB  
- Reboot the controller when prompted.

**Configuration**

The configuration is based on the PC IP the robot has to connect to. 
- We specify it on `Installation` menu:
    ```
    Installation → URCaps → External Control
    ```

- Set:
    - **Control PC IP:** (e.g., `192.168.1.55`)
    - **Port:** `50002`

## 2.3. ROS2 External Control Program
We first create a new program `ROS2_external_control.urp` including only the `External Control` instruction configured before

## 3. PC Configuration

The PC is an Ubuntu22 with ROS2 Humble and we have to install different modulus:
````bash
sudo apt install ros-humble-ur-robot-driver
sudo apt install ros-humble-ros2controlcli
sudo apt install ros-humble-ur-calibration
````

## 4. Quick start

To properly start working on the UR5e with ROS2 Humble we have to:
- First on PC:
    - Source ROS:
        ```bash
        source /opt/ros/humble/setup.bash
        source ~/ROS2_UR_manipulation_ws/install/setup.bash
        ```
    - Run the UR driver:
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.4 launch_rviz:=false
        ```
- Now on `Teach Pendant`:
    - Load program: **ROS2_external_control.urp**
    - Press **Play**  

## 5. Verify Joint States and run a first movement

- Open a new terminal and type:
    ```bash
    ros2 topic list
    ros2 topic echo /joint_states
    ```
- If messages stream → good connection.
- The good topic to publish a new target joint is: `/scaled_joint_trajectory_controller/joint_trajectory`
- Publish in a new terminal a target joint positions very close to the actual joint positions:
    ````bash
    ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
        joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        points: [
            {
            positions: [0.35, -2.3987, 2.5271, -3.2593, -0.5174, 3.1289],
            time_from_start: {sec: 4, nanosec: 0}
            }
        ]
    }"
    ````
- Publish in a new terminal a target joint positions to come back to the previous joint positions:
    ````bash
    ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
        joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        points: [
            {
            positions: [0.5149, -2.3987, 2.5271, -3.2593, -0.5174, 3.1289],
            time_from_start: {sec: 4, nanosec: 0}
            }
        ]
    }"
    ````

## 6. Use MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

Use RViz's *MotionPlanning* panel to plan and execute motions.

You can also do it in a unique instruction:
````bash
ros2 launch ur_robot_driver ur_moveit.launch.py ur_type:=ur5e robot_ip:=192.168.1.4 launch_rviz:=true
````

## 7. Example Python Script

```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy

def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    arm = moveit.get_planning_component("ur_manipulator")

    target = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.57,
        "elbow_joint": 1.57,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 1.57,
        "wrist_3_joint": 0.0
    }

    arm.set_goal_state(joint_positions=target)
    plan = arm.plan()
    arm.execute(plan)

if __name__ == "__main__":
    main()
```
