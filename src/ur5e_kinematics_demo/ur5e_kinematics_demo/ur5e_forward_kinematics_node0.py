import math
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK

# New: pymoveit2
from pymoveit2 import MoveIt2


class UR5eForwardKinematicsNode(Node):
    """
    Simple node that:
      1) Calls MoveIt's /compute_fk service for the UR5e
      2) Optionally sends the same joint configuration to MoveIt2 to move the robot

    Parameters (ROS params):
      - joint1 ... joint6: joint angles in radians
      - group_name: MoveIt planning group (default: "ur_manipulator")
      - fk_link: end-effector link for FK (default: "tool0")
      - execute: bool, if true -> MoveIt2 moves the robot to the requested joints
    """

    def __init__(self):
        super().__init__("ur5e_forward_kinematics_node")

        # --- Parameters ---
        self.declare_parameter("joint1", 0.0)
        self.declare_parameter("joint2", -1.57)
        self.declare_parameter("joint3", 1.57)
        self.declare_parameter("joint4", 0.0)
        self.declare_parameter("joint5", 1.57)
        self.declare_parameter("joint6", 0.0)

        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("fk_link", "tool0")
        self.declare_parameter("execute", False)

        # Read parameters
        self.joints = [
            float(self.get_parameter("joint1").value),
            float(self.get_parameter("joint2").value),
            float(self.get_parameter("joint3").value),
            float(self.get_parameter("joint4").value),
            float(self.get_parameter("joint5").value),
            float(self.get_parameter("joint6").value),
        ]
        self.group_name = self.get_parameter("group_name").value
        self.fk_link = self.get_parameter("fk_link").value
        self.execute_motion = bool(self.get_parameter("execute").value)

        self.get_logger().info(f"Using joints (rad): {self.joints}")
        self.get_logger().info(f"Group: {self.group_name}, FK link: {self.fk_link}")
        self.get_logger().info(f"Execute motion: {self.execute_motion}")

        # --- FK service client (/compute_fk) ---
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.get_logger().info("Waiting for /compute_fk service...")
        self.fk_client.wait_for_service()
        self.get_logger().info("Connected to /compute_fk service.")

        # --- MoveIt2 interface (pymoveit2) ---
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name="base_link",
            end_effector_name=self.fk_link,      # usually "tool0"
            group_name=self.group_name,
        )

        # Optional: slow down a bit
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # Do everything once at startup
        self.compute_fk_and_optionally_move()

    def compute_fk_and_optionally_move(self):
        # --- Build FK request ---
        header = self.get_clock().now().to_msg()
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.header.stamp = header

        # link we want the pose for
        request.fk_link_names.append(self.fk_link)

        js = JointState()
        js.header = request.header
        js.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        js.position = self.joints

        request.robot_state.joint_state = js

        # --- Call FK service ---
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("FK service call failed")
            self.shutdown()
            return

        response = future.result()
        if len(response.pose_stamped) == 0:
            self.get_logger().error("FK response contains no poses")
            self.shutdown()
            return

        pose: PoseStamped = response.pose_stamped[0]
        self.get_logger().info(f"FK result for link '{self.fk_link}':")
        self.get_logger().info(
            f"  Position: x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}"
        )
        self.get_logger().info(
            "  Orientation (quaternion): "
            f"x={pose.pose.orientation.x:.3f}, "
            f"y={pose.pose.orientation.y:.3f}, "
            f"z={pose.pose.orientation.z:.3f}, "
            f"w={pose.pose.orientation.w:.3f}"
        )

        # --- Optionally move the robot to these joints via MoveIt2 ---
        if self.execute_motion:
            self.get_logger().info("Sending joint configuration to MoveIt2...")
            self.moveit2.move_to_configuration(self.joints)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Motion execution finished.")

        self.shutdown()

    def shutdown(self):
        self.get_logger().info("FK node finished. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eForwardKinematicsNode()
    # No need to spin here: node does everything in constructor and then shuts down.


if __name__ == "__main__":
    main()
