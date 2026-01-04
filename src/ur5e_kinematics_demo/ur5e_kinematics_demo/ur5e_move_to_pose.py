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
    """
    Convention:
      R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Returns quaternion (qx, qy, qz, qw).
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
    return float(qx), float(qy), float(qz), float(qw)


class UR5ePickPlaceSimple(Node):
    """
    Simple, deterministic pick&place:
      For each step:
        Pose goal -> /compute_ik -> joint goal -> MoveIt2 move_to_configuration -> wait
      Seed is updated after each step (seed = last IK solution).
    """

    def __init__(self):
        super().__init__("ur5e_pick_place")

        # --- Pick/place params
        self.declare_parameter("pick_xyz", [0.45, 0.15, 0.20])
        self.declare_parameter("place_xyz", [0.35, -0.15, 0.20])
        self.declare_parameter("z_approach", 0.12)
        self.declare_parameter("z_lift", 0.12)
        self.declare_parameter("target_rpy", [0.0, 3.1, 0.0])  # roll,pitch,yaw [rad]

        # --- IK settings
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter(
            "seed_joints",
            [0.0, -math.pi / 2.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0],
        )
        self.declare_parameter("ik_timeout_sec", 0.2)

        # --- Motion settings
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)
        self.declare_parameter("sleep_sec_between_steps", 0.5)
        self.declare_parameter("print_joints", False)

        self.pick_xyz = [float(x) for x in self.get_parameter("pick_xyz").value]
        self.place_xyz = [float(x) for x in self.get_parameter("place_xyz").value]
        self.z_approach = float(self.get_parameter("z_approach").value)
        self.z_lift = float(self.get_parameter("z_lift").value)
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.sleep_sec = float(self.get_parameter("sleep_sec_between_steps").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)

        # --- IK service client
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # --- MoveIt2 (joint execution)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(0.1, self._run_once)

    # -------------------------
    # Helpers
    # -------------------------
    def _sleep_spin(self, seconds: float):
        """Sleep but keep spinning so callbacks keep flowing."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _make_pose(self, xyz, rpy) -> PoseStamped:
        roll, pitch, yaw = rpy
        qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = f
