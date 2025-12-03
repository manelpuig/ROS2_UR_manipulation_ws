# UR5e Kinematics Package (ROS 2 Humble + MoveIt)

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services. (https://docs.ros.org/en/humble/p/pymoveit2/index.html
)

Instructions to install:
- Clone this repository, install dependencies and build with colcon.

    ```bash
    # Clone this repository into your favourite ROS 2 workspace
    git clone https://github.com/AndrejOrsula/pymoveit2.git
    # Install dependencies
    rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
    # Build
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
    ```

This document describes how to create and use a simple ROS 2 package that provides **forward** and **inverse kinematics** tools for a **UR5e** robot using **MoveIt 2**.

The package contains two nodes:

- `ur5e_forward_kinematics_node` – you specify the 6 joint angles → it computes the end–effector pose.
- `ur5e_inverse_kinematics_node` – you specify a target pose (x, y, z, roll, pitch, yaw) → it computes one joint solution.

Both nodes call the standard MoveIt services `/compute_fk` and `/compute_ik`, so **MoveIt’s `move_group` must be running** (e.g., via `ur_moveit_config`).

---

## 1. Prerequisites

On your Ubuntu 22.04 PC:

- ROS 2 Humble installed.
- UR driver + MoveIt installed (e.g. via your `setup_ur_env.sh`).
- A workspace, e.g.:
  ```bash
  ~/ROS2_UR_manipulation_ws
  ```
- UR5e MoveIt config already working:
  ```bash
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
  ```

You should be able to see the UR5e in RViz and plan motions before adding this package.

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

Create `ur5e_kinematics_demo/ur5e_kinematics_demo/ur5e_forward_kinematics_node.py`:

```python
import math
from typing import List

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK


class UR5eForwardKinematicsNode(Node):
    '''
    Simple node that calls MoveIt's /compute_fk service for the UR5e.

    Parameters (declared as ROS parameters):
    - joint1 ... joint6: joint angles in radians
    - group_name: MoveIt planning group (default: "ur_manipulator")
    - fk_link: end-effector link for FK (default: "tool0")
    '''

    def __init__(self):
        super().__init__('ur5e_forward_kinematics_node')

        # Declare parameters
        self.declare_parameter('joint1', 0.0)
        self.declare_parameter('joint2', 0.0)
        self.declare_parameter('joint3', 0.0)
        self.declare_parameter('joint4', 0.0)
        self.declare_parameter('joint5', 0.0)
        self.declare_parameter('joint6', 0.0)
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('fk_link', 'tool0')

        # Read parameters
        self.joints = [
            float(self.get_parameter('joint1').value),
            float(self.get_parameter('joint2').value),
            float(self.get_parameter('joint3').value),
            float(self.get_parameter('joint4').value),
            float(self.get_parameter('joint5').value),
            float(self.get_parameter('joint6').value),
        ]
        self.group_name = self.get_parameter('group_name').value
        self.fk_link = self.get_parameter('fk_link').value

        self.get_logger().info(f"Using joints (rad): {self.joints}")
        self.get_logger().info(f"Group: {self.group_name}, FK link: {self.fk_link}")

        # FK service client
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        self.get_logger().info("Waiting for /compute_fk service...")
        self.fk_client.wait_for_service()
        self.get_logger().info("Connected to /compute_fk service.")

        # Call FK once on startup
        self.compute_fk()

    def compute_fk(self):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'   # reference frame for the pose
        request.header.stamp = self.get_clock().now().to_msg()

        # We want the pose of this link
        request.fk_link_names.append(self.fk_link)

        # Build JointState
        js = JointState()
        js.header = request.header
        js.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        js.position = self.joints

        request.robot_state.joint_state = js

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('FK service call failed')
            return

        response = future.result()
        if len(response.pose_stamped) == 0:
            self.get_logger().error('FK response contains no poses')
            return

        pose: PoseStamped = response.pose_stamped[0]
        self.get_logger().info(f"FK result for link '{self.fk_link}':")
        self.get_logger().info(f"  Position: x={pose.pose.position.x:.3f}, "
                               f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")
        self.get_logger().info(f"  Orientation (quaternion): "
                               f"x={pose.pose.orientation.x:.3f}, "
                               f"y={pose.pose.orientation.y:.3f}, "
                               f"z={pose.pose.orientation.z:.3f}, "
                               f"w={pose.pose.orientation.w:.3f}")

        # Optional: shut down once done
        self.get_logger().info("FK computation finished. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eForwardKinematicsNode()
    # No spin needed: node calls FK once in constructor and then shuts down.
```

---

## 4. Inverse kinematics node

Create `ur5e_kinematics_demo/ur5e_kinematics_demo/ur5e_inverse_kinematics_node.py`:

```python
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK


def rpy_to_quaternion(roll, pitch, yaw):
    '''
    Convert roll, pitch, yaw (rad) to a quaternion (x, y, z, w).
    '''
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class UR5eInverseKinematicsNode(Node):
    '''
    Simple node that calls MoveIt's /compute_ik service for the UR5e.

    Parameters:
    - target_x, target_y, target_z: position [m]
    - target_roll, target_pitch, target_yaw: orientation [rad]
    - group_name: planning group (default: "ur_manipulator")
    - ik_link: end-effector link (default: "tool0")
    - seed_joint1 ... seed_joint6: optional seed joint configuration [rad]
    '''

    def __init__(self):
        super().__init__('ur5e_inverse_kinematics_node')

        # Target pose parameters
        self.declare_parameter('target_x', 0.4)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.3)
        self.declare_parameter('target_roll', 0.0)
        self.declare_parameter('target_pitch', math.pi)  # typical UR tool pointing down
        self.declare_parameter('target_yaw', 0.0)

        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('ik_link', 'tool0')

        # Seed joints (optional)
        self.declare_parameter('seed_joint1', 0.0)
        self.declare_parameter('seed_joint2', -math.pi/2)
        self.declare_parameter('seed_joint3', math.pi/2)
        self.declare_parameter('seed_joint4', 0.0)
        self.declare_parameter('seed_joint5', math.pi/2)
        self.declare_parameter('seed_joint6', 0.0)

        self.group_name = self.get_parameter('group_name').value
        self.ik_link = self.get_parameter('ik_link').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.target_roll = float(self.get_parameter('target_roll').value)
        self.target_pitch = float(self.get_parameter('target_pitch').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        self.seed_joints = [
            float(self.get_parameter('seed_joint1').value),
            float(self.get_parameter('seed_joint2').value),
            float(self.get_parameter('seed_joint3').value),
            float(self.get_parameter('seed_joint4').value),
            float(self.get_parameter('seed_joint5').value),
            float(self.get_parameter('seed_joint6').value),
        ]

        self.get_logger().info(f"Target position: ({self.target_x}, {self.target_y}, {self.target_z})")
        self.get_logger().info(f"Target RPY (rad): ({self.target_roll}, {self.target_pitch}, {self.target_yaw})")
        self.get_logger().info(f"Group: {self.group_name}, IK link: {self.ik_link}")
        self.get_logger().info(f"Seed joints (rad): {self.seed_joints}")

        # IK client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info("Waiting for /compute_ik service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("Connected to /compute_ik service.")

        self.compute_ik()

    def compute_ik(self):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link

        # Target pose
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z

        qx, qy, qz, qw = rpy_to_quaternion(self.target_roll,
                                           self.target_pitch,
                                           self.target_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request.ik_request.pose_stamped = pose

        # Seed state
        js = JointState()
        js.header = pose.header
        js.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        js.position = self.seed_joints
        request.ik_request.robot_state.joint_state = js

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('IK service call failed')
            return

        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(f"IK failed with error code: {response.error_code.val}")
            return

        solution = response.solution.joint_state
        self.get_logger().info("IK solution:")
        for name, pos in zip(solution.name, solution.position):
            self.get_logger().info(f"  {name}: {pos:.4f} rad")

        self.get_logger().info("IK computation finished. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eInverseKinematicsNode()
    # No spin needed: node calls IK once and then shuts down.
```

---

## 5. Launch files

Create the `launch` folder:

```bash
mkdir -p ur5e_kinematics_demo/launch
```

### 5.1 Forward kinematics launch: `fk_ur5e.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Joint arguments (in radians)
    joint1 = DeclareLaunchArgument('joint1', default_value='0.0')
    joint2 = DeclareLaunchArgument('joint2', default_value='-1.57')
    joint3 = DeclareLaunchArgument('joint3', default_value='1.57')
    joint4 = DeclareLaunchArgument('joint4', default_value='0.0')
    joint5 = DeclareLaunchArgument('joint5', default_value='1.57')
    joint6 = DeclareLaunchArgument('joint6', default_value='0.0')

    fk_link = DeclareLaunchArgument('fk_link', default_value='tool0')
    group_name = DeclareLaunchArgument('group_name', default_value='ur_manipulator')

    node = Node(
        package='ur5e_kinematics_demo',
        executable='ur5e_forward_kinematics_node',
        name='ur5e_fk_node',
        output='screen',
        parameters=[{
            'joint1': LaunchConfiguration('joint1'),
            'joint2': LaunchConfiguration('joint2'),
            'joint3': LaunchConfiguration('joint3'),
            'joint4': LaunchConfiguration('joint4'),
            'joint5': LaunchConfiguration('joint5'),
            'joint6': LaunchConfiguration('joint6'),
            'fk_link': LaunchConfiguration('fk_link'),
            'group_name': LaunchConfiguration('group_name'),
        }]
    )

    return LaunchDescription([
        joint1, joint2, joint3, joint4, joint5, joint6,
        fk_link, group_name,
        node
    ])
```

### 5.2 Inverse kinematics launch: `ik_ur5e.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Target pose (meters, radians)
    target_x = DeclareLaunchArgument('target_x', default_value='0.4')
    target_y = DeclareLaunchArgument('target_y', default_value='0.0')
    target_z = DeclareLaunchArgument('target_z', default_value='0.3')

    target_roll = DeclareLaunchArgument('target_roll', default_value='0.0')
    target_pitch = DeclareLaunchArgument('target_pitch', default_value='3.14159')
    target_yaw = DeclareLaunchArgument('target_yaw', default_value='0.0')

    group_name = DeclareLaunchArgument('group_name', default_value='ur_manipulator')
    ik_link = DeclareLaunchArgument('ik_link', default_value='tool0')

    node = Node(
        package='ur5e_kinematics_demo',
        executable='ur5e_inverse_kinematics_node',
        name='ur5e_ik_node',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_z': LaunchConfiguration('target_z'),
            'target_roll': LaunchConfiguration('target_roll'),
            'target_pitch': LaunchConfiguration('target_pitch'),
            'target_yaw': LaunchConfiguration('target_yaw'),
            'group_name': LaunchConfiguration('group_name'),
            'ik_link': LaunchConfiguration('ik_link'),
        }]
    )

    return LaunchDescription([
        target_x, target_y, target_z,
        target_roll, target_pitch, target_yaw,
        group_name, ik_link,
        node
    ])
```

---

## 6. Build the workspace

```bash
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 7. How to run

### 7.1 Forward kinematics

Terminal 1 – start MoveIt:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```

Terminal 2 – compute FK for a given joint configuration:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash

ros2 launch ur5e_kinematics_demo fk_ur5e.launch.py \
  joint1:=0.0 \
  joint2:=-1.57 \
  joint3:=1.57 \
  joint4:=0.0 \
  joint5:=1.57 \
  joint6:=0.0
```

### 7.2 Inverse kinematics

Terminal 1 – MoveIt (same as before).

Terminal 2 – request IK for a target pose:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash

ros2 launch ur5e_kinematics_demo ik_ur5e.launch.py \
  target_x:=0.4 \
  target_y:=0.0 \
  target_z:=0.3 \
  target_roll:=0.0 \
  target_pitch:=3.14159 \
  target_yaw:=0.0
```

The node prints the solution joint values.

---

## 8. Extensions

- Publish FK result on a topic, e.g. `/ur5e_fk_pose`.
- Turn IK node into a service that accepts pose requests at runtime.
- Use the IK solution to build a full trajectory and send it to the UR controller via MoveIt.
