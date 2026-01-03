#!/usr/bin/env python3
import math
import time
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
    Returns quaternion in ROS order: (qx, qy, qz, qw).
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


class UR5ePickPlaceViaIK(Node):
    """
    Teaching-friendly Pick&Place:
      - Optional "home" first (known start posture)
      - For each step: pose -> /compute_ik (seeded) -> execute joints
    """

    def __init__(self):
        super().__init__("ur5e_pick_place")

        # ---- General
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")

        # IK seed control (key for choosing among multiple IK solutions)
        self.declare_parameter(
            "seed_joints",
            [0.0, -math.pi / 2.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0],
        )
        self.declare_parameter(
            "home_joints",
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        )
        self.declare_parameter("use_home_first", True)
        # seed_mode:
        #   "fixed"    -> always use seed_joints (constant reference posture)
        #   "last_goal"-> update seed after each step to keep same IK branch
        self.declare_parameter("seed_mode", "last_goal")

        self.declare_parameter("ik_timeout_sec", 0.2)
        self.declare_parameter("avoid_collisions", True)

        # ---- Motion settings
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)

        # ---- Pick & Place settings
        self.declare_parameter("pick_xyz", [0.45, 0.10, 0.22])
        self.declare_parameter("place_xyz", [0.35, -0.20, 0.22])
        self.declare_parameter("z_approach", 0.10)
        self.declare_parameter("z_lift", 0.12)
        self.declare_parameter("target_rpy", [0.0, 0.0, 0.0])  # start simple

        # Optional: slow down sequence for visualization
        self.declare_parameter("sleep_sec_between_steps", 0.0)

        # ---- Read parameters
        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)

        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.home_joints = [float(x) for x in self.get_parameter("home_joints").value]
        self.use_home_first = bool(self.get_parameter("use_home_first").value)
        self.seed_mode = str(self.get_parameter("seed_mode").value).strip().lower()

        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)

        self.pick_xyz = [float(x) for x in self.get_parameter("pick_xyz").value]
        self.place_xyz = [float(x) for x in self.get_parameter("place_xyz").value]
        self.z_approach = float(self.get_parameter("z_approach").value)
        self.z_lift = float(self.get_parameter("z_lift").value)
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]
        self.sleep_sec = float(self.get_parameter("sleep_sec_between_steps").value)

        if self.seed_mode not in ("fixed", "last_goal"):
            self.get_logger().warn("seed_mode must be 'fixed' or 'last_goal'. Falling back to 'last_goal'.")
            self.seed_mode = "last_goal"

        # ---- IK client and MoveIt2 executor
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        # Build sequence
        self.sequence = self._build_sequence()

        self._step_idx = 0
        self._started = False
        self.create_timer(0.1, self._tick)

    def _build_sequence(self):
        px, py, pz = self.pick_xyz
        qx, qy, qz = self.place_xyz

        return [
            ("pre_pick",  [px, py, pz + self.z_approach], self.target_rpy),
            ("pick",      [px, py, pz],                   self.target_rpy),
            ("lift",      [px, py, pz + self.z_lift],     self.target_rpy),

            ("pre_place", [qx, qy, qz + self.z_approach], self.target_rpy),
            ("place",     [qx, qy, qz],                   self.target_rpy),
            ("retreat",   [qx, qy, qz + self.z_lift],     self.target_rpy),
        ]

    def _tick(self):
        if self._started:
            return
        self._started = True

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /compute_ik not available. Start MoveIt first.")
            rclpy.shutdown()
            return

        # (0) Optional: go to HOME first for a known starting posture
        if self.use_home_first and self.execute_motion:
            self.get_logger().info("Moving to HOME joints first...")
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()
            self.get_logger().info("HOME reached.")
            if self.seed_mode == "last_goal":
                self.seed_joints = list(self.home_joints)

        # Start sequence
        self._start_next_step()

    def _start_next_step(self):
        if self._step_idx >= len(self.sequence):
            self.get_logger().info("Pick&Place sequence finished.")
            rclpy.shutdown()
            return

        name, xyz, rpy = self.sequence[self._step_idx]
        self.get_logger().info(f"[{self._step_idx+1}/{len(self.sequence)}] Step: {name}  xyz={xyz}  rpy={rpy}")

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz

        roll, pitch, yaw = rpy
        qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # Build IK request (seed controls which IK solution we converge to)
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = self.avoid_collisions

        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int((self.ik_timeout - int(self.ik_timeout)) * 1e9)

        seed = JointState()
        seed.header = pose.header
        seed.name = UR5E_JOINTS
        seed.position = self.seed_joints
        req.ik_request.robot_state.joint_state = seed

        self.get_logger().info(
            f"  IK seed (first 3 joints): [{self.seed_joints[0]:.3f}, {self.seed_joints[1]:.3f}, {self.seed_joints[2]:.3f}]"
        )

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik)

    def _on_ik(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            rclpy.shutdown()
            return

        step_name, xyz, rpy = self.sequence[self._step_idx]

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(
                f"IK failed at step '{step_name}' (xyz={xyz}, rpy={rpy}), error code: {res.error_code.val}"
            )
            rclpy.shutdown()
            return

        sol = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}
        joint_goal = [float(name_to_pos[j]) for j in UR5E_JOINTS]

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> not executing. Stopping.")
            rclpy.shutdown()
            return

        # Execute joint goal
        self.moveit2.move_to_configuration(joint_goal)
        self.moveit2.wait_until_executed()

        # Update seed to keep consistent IK branch (recommended)
        if self.seed_mode == "last_goal":
            self.seed_joints = list(joint_goal)

        # Next step
        self._step_idx += 1

        if self.sleep_sec > 0.0:
            time.sleep(self.sleep_sec)

        self._start_next_step()


def main():
    rclpy.init()
    node = UR5ePickPlaceViaIK()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
