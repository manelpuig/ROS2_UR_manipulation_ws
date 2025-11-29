# Connecting and Controlling a Real UR5e Robot with ROS 2 Humble + MoveIt

This document summarizes the necessary steps to connect a **real Universal Robots UR5e** to a **PC running Ubuntu 22.04 + ROS 2 Humble**, install the required URCaps, configure networking, and control the robot using **MoveIt**.

---

## 1. Network Setup

Ensure the PC and UR5e are on the same LAN.

Example:
- PC IP: `192.168.1.55`
- UR5e IP: `192.168.1.4`

Verify connection:
```bash
ping 192.168.1.4
```

---

## 2. Install the Required URCap on the UR5e

### 2.1 Download the URCap
Download:
```
externalcontrol-1.0.urcap
```
From:
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble/ur_robot_driver/resources

### 2.2 Install via Teach Pendant
1. Copy `.urcap` file to USB (FAT32)
2. On the teach pendant:
   - **Settings → System → URCaps → Manage**
   - **Add** the file from USB  
3. Reboot the controller when prompted.

### 2.3 Configure External Control
```
Installation → URCaps → External Control
```

Set:
- **Control PC IP:** (e.g., `192.168.1.55`)
- **Port:** `50002`

---

## 3. Running the External Control Program

1. Load program:
   ```
   external_control.urp
   ```
2. Press **Play**  

3. Robot displays:
    ```
    Waiting for incoming RTDE connection…
    ```

---

## 4. Configure the PC

Source ROS:
```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash
```

Run the UR driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5e   robot_ip:=192.168.1.4   launch_rviz:=false
```

---

## 5. Verify Joint States

```bash
ros2 topic echo /joint_states
```

If messages stream → good connection.

---

## 6. Run MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```

Use RViz's *MotionPlanning* panel to plan and execute motions.

---

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

---

## Summary

### On the UR5e:
- Install URCap  
- Configure External Control  
- Run `external_control.urp`

### On the PC:
- Run the UR driver  
- Run MoveIt  

System ready for real-robot control.

