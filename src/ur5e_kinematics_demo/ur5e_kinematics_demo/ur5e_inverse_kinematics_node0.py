import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from pymoveit2 import MoveIt2


def rpy_to_quaternion(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw (rad) to quaternion (x, y, z, w).
    """
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
    """
    Simple node that:
      1) Calls MoveIt's /compute_ik service for the UR5e
      2) Prints the joint solution
      3) Optionally sends the solution to MoveIt2 to move the robot

    Parameters:
      - target_x, target_y, target_z: position [m]
      - target_roll, target_pitch, target_yaw: orientation [rad]
      - group_name: planning group (default: "ur_manipulator")
      - ik_link: end-effector link (default: "tool0")
      - seed_joint1 ... seed_joint6: optional seed configuration [rad]
      - execute: bool, if true -> MoveIt2 moves the robot to the IK solution
    """

    def __init__(self):
        super().__init__("ur5e_inverse_kinematics_node")

        # --- Target pose parameters ---
        self.declare_parameter("target_x", 0.4)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.3)

        self.declare_parameter("target_roll", 0.0)
        self.declare_parameter("target_pitch", math.pi)  # tool pointing down
        self.declare_parameter("target_yaw", 0.0)

        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter("execute", False)

        # Seed joints
        self.declare_parameter("seed_joint1", 0.0)
        self.declare_parameter("seed_joint2", -math.pi / 2)
        self.declare_parameter("seed_joint3", math.pi / 2)
        self.declare_parameter("seed_joint4", 0.0)
        self.declare_parameter("seed_joint5", math.pi / 2)
        self.declare_parameter("seed_joint6", 0.0)

        self.group_name = self.get_parameter("group_name").value
        self.ik_link = self.get_parameter("ik_link").value
        self.execute_motion = bool(self.get_parameter("execute").value)

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.target_roll = float(self.get_parameter("target_roll").value)
        self.target_pitch = float(self.get_parameter("target_pitch").value)
        self.target_yaw = float(self.get_parameter("target_yaw").value)

        self.seed_joints = [
            float(self.get_parameter("seed_joint1").value),
            float(self.get_parameter("seed_joint2").value),
            float(self.get_parameter("seed_joint3").value),
            float(self.get_parameter("seed_joint4").value),
            float(self.get_parameter("seed_joint5").value),
            float(self.get_parameter("seed_joint6").value),
        ]

        self.get_logger().info(
            f"Target position: ({self.target_x}, {self.target_y}, {self.target_z})"
        )
        self.get_logger().info(
            f"Target RPY (rad): ({self.target_roll}, "
            f"{self.target_pitch}, {self.target_yaw})"
        )
        self.get_logger().info(f"Group: {self.group_name}, IK link: {self.ik_link}")
        self.get_logger().info(f"Seed joints (rad): {self.seed_joints}")
        self.get_logger().info(f"Execute motion: {self.execute_motion}")

        # --- IK client ---
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("Connected to /compute_ik service.")

        # --- MoveIt2 interface ---
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
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )

        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # Compute IK once and optionally move
        self.compute_ik_and_optionally_move()

    def compute_ik_and_optionally_move(self):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link

        # Target pose
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z

        qx, qy, qz, qw = rpy_to_quaternion(
            self.target_roll, self.target_pitch, self.target_yaw
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request.ik_request.pose_stamped = pose

        # Seed state
        js = JointState()
        js.header = pose.header
        js.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        js.position = self.seed_joints
        request.ik_request.robot_state.joint_state = js

        # Call IK service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("IK service call failed")
            self.shutdown()
            return

        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(
                f"IK failed with error code: {response.error_code.val}"
            )
            self.shutdown()
            return

        solution = response.solution.joint_state
        self.get_logger().info("IK solution:")
        for name, pos in zip(solution.name, solution.position):
            self.get_logger().info(f"  {name}: {pos:.4f} rad")

        # --- Optionally move the robot to this IK solution ---
        if self.execute_motion:
            joint_goal = list(solution.position)
            self.get_logger().info("Sending IK solution to MoveIt2...")
            self.moveit2.move_to_configuration(joint_goal)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Motion execution finished.")

        self.shutdown()

    def shutdown(self):
        self.get_logger().info("IK node finished. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eInverseKinematicsNode()
    # No spin: everything is done in constructor.


if __name__ == "__main__":
    main()
