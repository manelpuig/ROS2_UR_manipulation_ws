#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from pymoveit2 import MoveIt2

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def quat_from_rpy_zyx(roll: float, pitch: float, yaw: float):
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
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
    return float(qx), float(qy), float(qz), float(qw)


class UR5eTwoPosesSimple(Node):
    """
    Minimal deterministic sequence: Pose1 -> IK -> execute joints -> Pose2 -> IK -> execute joints
    Uses the same pattern as your working "go to pose" node:
      /compute_ik + pymoveit2.move_to_configuration + wait_until_executed
    """

    def __init__(self):
        super().__init__("ur5e_pick_place")

        # --- Parameters
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter("execute", True)
        self.declare_parameter("sleep_sec", 0.5)
        self.declare_parameter("ik_timeout_sec", 0.3)

        # Pose1 / Pose2
        self.declare_parameter("pose1_xyz", [0.45, 0.15, 0.32])
        self.declare_parameter("pose2_xyz", [0.45, 0.15, 0.20])
        self.declare_parameter("target_rpy", [0.0, 3.1, 0.0])

        # Seed (same style you used)
        self.declare_parameter(
            "seed_joints",
            [0.0, -math.pi / 2.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0],
        )

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.sleep_sec = float(self.get_parameter("sleep_sec").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.pose1_xyz = [float(x) for x in self.get_parameter("pose1_xyz").value]
        self.pose2_xyz = [float(x) for x in self.get_parameter("pose2_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]

        # --- IK service
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # --- MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )

        self._ran = False
        self.create_timer(0.1, self._run_once)

    def _sleep_spin(self, seconds: float):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _make_pose(self, xyz) -> PoseStamped:
        roll, pitch, yaw = self.target_rpy
        qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)

        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(xyz[0])
        p.pose.position.y = float(xyz[1])
        p.pose.position.z = float(xyz[2])
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _call_ik_blocking(self, pose: PoseStamped, seed_joints):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose

        # Humble: timeout exists, attempts does NOT
        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int((self.ik_timeout - int(self.ik_timeout)) * 1e9)

        # Seed robot_state
        js = JointState()
        js.header = pose.header
        js.name = UR5E_JOINTS
        js.position = list(seed_joints)
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ik_timeout + 2.0)

        if not future.done() or future.result() is None:
            return None

        res = future.result()
        if res.error_code.val != res.error_code.SUCCESS:
            return None

        sol = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}
        return [float(name_to_pos[j]) for j in UR5E_JOINTS]

    def _go_pose(self, name: str, xyz, seed):
        self.get_logger().info(f"{name}: xyz={xyz} rpy={self.target_rpy}")

        pose = self._make_pose(xyz)

        self.get_logger().info("  IK...")
        q_goal = self._call_ik_blocking(pose, seed)
        if q_goal is None:
            self.get_logger().error(f"  IK failed for {name}")
            return None

        if not self.execute_motion:
            self.get_logger().warn("execute:=false -> stopping after IK (no motion).")
            return q_goal

        self.get_logger().info("  Executing...")
        self.moveit2.move_to_configuration(q_goal)
        self.moveit2.wait_until_executed()
        self.get_logger().info("  Done.")
        return q_goal

    def _run_once(self):
        if self._ran:
            return
        self._ran = True

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available. Start MoveIt first.")
            rclpy.shutdown()
            return

        seed = list(self.seed_joints)

        q1 = self._go_pose("POSE1", self.pose1_xyz, seed)
        if q1 is None:
            rclpy.shutdown()
            return
        seed = list(q1)

        if self.sleep_sec > 0.0:
            self._sleep_spin(self.sleep_sec)

        q2 = self._go_pose("POSE2", self.pose2_xyz, seed)
        if q2 is None:
            rclpy.shutdown()
            return

        self.get_logger().info("Two-pose sequence finished.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eTwoPosesSimple()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
