#!/usr/bin/env python3
"""
Robust MoveIt2 pick&place via:
  - /compute_ik              (moveit_msgs/srv/GetPositionIK)
  - /plan_kinematic_path     (moveit_msgs/srv/GetMotionPlan)
  - /execute_trajectory      (moveit_msgs/action/ExecuteTrajectory)

Key properties (solves "moves once then stops"):
  - Waits for /joint_states before starting
  - Runs sequence in a worker thread (no blocking timer callback)
  - Uses spin-sleep instead of time.sleep to keep callbacks alive
  - Seeds IK with current joints and refreshes seed after each executed step
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, MotionPlanRequest
from moveit_msgs.action import ExecuteTrajectory

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.action import ActionClient


UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@dataclass
class Step:
    name: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert RPY to quaternion (x,y,z,w) without external deps."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class UR5ePickPlaceRobust(Node):
    def __init__(self):
        super().__init__("ur5e_pick_place")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter("frame_id", "base_link")

        self.declare_parameter("execute", True)
        self.declare_parameter("sleep_sec_between_steps", 0.5)

        self.declare_parameter("ik_timeout_sec", 0.5)
        self.declare_parameter("ik_attempts", 8)

        self.declare_parameter("planning_time_sec", 5.0)
        self.declare_parameter("planning_attempts", 5)

        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)

        self.declare_parameter("use_home_first", False)
        self.declare_parameter("home_joints", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.declare_parameter("home_max_joint_distance", 2.0)

        # Pick/place definition (same idea as your previous node)
        self.declare_parameter("pick_xyz", [0.45, 0.15, 0.20])
        self.declare_parameter("place_xyz", [0.35, -0.15, 0.20])
        self.declare_parameter("z_approach", 0.12)
        self.declare_parameter("z_lift", 0.12)

        self.declare_parameter("target_rpy", [0.0, 3.1, 0.0])

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.execute_motion = bool(self.get_parameter("execute").value)
        self.sleep_sec_between_steps = float(self.get_parameter("sleep_sec_between_steps").value)

        self.ik_timeout_sec = float(self.get_parameter("ik_timeout_sec").value)
        self.ik_attempts = int(self.get_parameter("ik_attempts").value)

        self.planning_time_sec = float(self.get_parameter("planning_time_sec").value)
        self.planning_attempts = int(self.get_parameter("planning_attempts").value)

        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)

        self.use_home_first = bool(self.get_parameter("use_home_first").value)
        self.home_joints = [float(x) for x in self.get_parameter("home_joints").value]
        self.home_max_joint_distance = float(self.get_parameter("home_max_joint_distance").value)

        pick_xyz = [float(x) for x in self.get_parameter("pick_xyz").value]
        place_xyz = [float(x) for x in self.get_parameter("place_xyz").value]
        self.z_approach = float(self.get_parameter("z_approach").value)
        self.z_lift = float(self.get_parameter("z_lift").value)

        rpy = [float(x) for x in self.get_parameter("target_rpy").value]
        target_rpy = (rpy[0], rpy[1], rpy[2])

        # Build sequence
        px, py, pz = pick_xyz
        qx, qy, qz = place_xyz

        self.steps: List[Step] = [
            Step("pre_pick", (px, py, pz + self.z_approach), target_rpy),
            Step("pick",     (px, py, pz),               target_rpy),
            Step("lift",     (px, py, pz + self.z_lift), target_rpy),
            Step("pre_place",(qx, qy, qz + self.z_approach), target_rpy),
            Step("place",    (qx, qy, qz),               target_rpy),
            Step("retreat",  (qx, qy, qz + self.z_lift),  target_rpy),
        ]

        # ---------------------------
        # Joint state cache
        # ---------------------------
        self._last_joint_map: Dict[str, float] = {}
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        # ---------------------------
        # MoveIt clients
        # ---------------------------
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # Seed joints (updated at runtime)
        self.seed_joints: Optional[List[float]] = None

        # Worker thread control
        self._worker_started = False
        self._worker_ok = False

        # Start worker once ROS is spinning
        self.create_timer(0.2, self._start_worker_once)

    # ---------------------------
    # Utilities
    # ---------------------------
    def _on_joint_states(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        n = min(len(msg.name), len(msg.position))
        with self._js_lock:
            for i in range(n):
                self._last_joint_map[msg.name[i]] = float(msg.position[i])

    def _get_current_ur_joints(self) -> Optional[List[float]]:
        with self._js_lock:
            if not self._last_joint_map:
                return None
            out = []
            for j in UR5E_JOINTS:
                if j not in self._last_joint_map:
                    return None
                out.append(self._last_joint_map[j])
            return out

    def spin_sleep(self, seconds: float):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        t0 = self.get_clock().now()
        while rclpy.ok():
            ik_ok = self.ik_cli.wait_for_service(timeout_sec=0.2)
            plan_ok = self.plan_cli.wait_for_service(timeout_sec=0.2)
            exec_ok = self.exec_ac.wait_for_server(timeout_sec=0.2)
            if ik_ok and plan_ok and exec_ok:
                return True
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout_sec:
                return False
        return False

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> Optional[List[float]]:
        t0 = self.get_clock().now()
        while rclpy.ok():
            cur = self._get_current_ur_joints()
            if cur is not None:
                return cur
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout_sec:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    # ---------------------------
    # MoveIt helpers
    # ---------------------------
    def _make_pose(self, xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        qx, qy, qz, qw = rpy_to_quat(rpy[0], rpy[1], rpy[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def call_ik(self, target_pose: PoseStamped, seed_joints: List[float]) -> Optional[List[float]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = target_pose
        req.ik_request.timeout = DurationMsg(sec=int(self.ik_timeout_sec),
                                             nanosec=int((self.ik_timeout_sec % 1.0) * 1e9))
        req.ik_request.attempts = int(self.ik_attempts)

        # Seed state
        rs = RobotState()
        rs.joint_state.name = list(UR5E_JOINTS)
        rs.joint_state.position = [float(x) for x in seed_joints]
        req.ik_request.robot_state = rs

        fut = self.ik_cli.call_async(req)
        # IMPORTANT: spin until complete (avoid deadlock)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.ik_timeout_sec + 2.0)
        if not fut.done() or fut.result() is None:
            return None

        res = fut.result()
        # moveit_msgs/MoveItErrorCodes: SUCCESS = 1
        if res.error_code.val != 1:
            return None

        js = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        out = []
        for j in UR5E_JOINTS:
            if j not in name_to_pos:
                return None
            out.append(float(name_to_pos[j]))
        return out

    def plan_to_joints(self, start_joints: List[float], goal_joints: List[float]):
        req = GetMotionPlan.Request()

        mpr = MotionPlanRequest()
        mpr.group_name = self.group_name
        mpr.num_planning_attempts = int(self.planning_attempts)
        mpr.allowed_planning_time = float(self.planning_time_sec)
        mpr.max_velocity_scaling_factor = float(self.velocity_scaling)
        mpr.max_acceleration_scaling_factor = float(self.acceleration_scaling)

        # Start state
        mpr.start_state = RobotState()
        mpr.start_state.joint_state.name = list(UR5E_JOINTS)
        mpr.start_state.joint_state.position = [float(x) for x in start_joints]

        # Goal constraints as JointConstraints
        c = Constraints()
        for jname, jpos in zip(UR5E_JOINTS, goal_joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jpos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints = [c]

        req.motion_plan_request = mpr

        fut = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.planning_time_sec + 5.0)
        if not fut.done() or fut.result() is None:
            return None

        res = fut.result()
        if res.motion_plan_response.error_code.val != 1:
            return None

        return res.motion_plan_response.trajectory

    def execute_traj(self, traj) -> bool:
        if not self.execute_motion:
            self.get_logger().warn("execute:=false -> skipping execution (planning only).")
            return True

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self.exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=5.0)
        if not send_fut.done() or send_fut.result() is None:
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            return False

        res_fut = goal_handle.get_result_async()
        # IMPORTANT: spin until result (avoid deadlock)
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=self.planning_time_sec + 20.0)
        if not res_fut.done() or res_fut.result() is None:
            return False

        result = res_fut.result().result
        # ExecuteTrajectoryResult has error_code (MoveItErrorCodes)
        return getattr(result.error_code, "val", 0) == 1

    # ---------------------------
    # Worker
    # ---------------------------
    def _start_worker_once(self):
        if self._worker_started:
            return
        self._worker_started = True
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self):
        self.get_logger().info("Waiting for MoveIt services and action server...")
        if not self.wait_for_services(timeout_sec=12.0):
            self.get_logger().error("Timeout waiting for /compute_ik, /plan_kinematic_path, /execute_trajectory.")
            return

        cur = self.wait_for_joint_states(timeout_sec=5.0)
        if cur is None:
            self.get_logger().error("Timeout waiting for /joint_states (UR joints).")
            return

        self.seed_joints = list(cur)
        self.get_logger().info(
            f"Initial joints (first 3): [{self.seed_joints[0]:.3f}, {self.seed_joints[1]:.3f}, {self.seed_joints[2]:.3f}]"
        )

        # Optional HOME
        if self.use_home_first and self.execute_motion:
            dist = sum(abs(a - b) for a, b in zip(self.seed_joints, self.home_joints))
            if dist > self.home_max_joint_distance:
                self.get_logger().warn(
                    f"HOME is far (L1 distance={dist:.3f} rad). Skipping HOME."
                )
            else:
                self.get_logger().info("Planning/executing HOME...")
                traj = self.plan_to_joints(self.seed_joints, self.home_joints)
                if traj is None:
                    self.get_logger().error("Failed planning to HOME.")
                    return
                if not self.execute_traj(traj):
                    self.get_logger().error("Failed executing HOME.")
                    return
                self.spin_sleep(self.sleep_sec_between_steps)
                cur = self.wait_for_joint_states(timeout_sec=2.0)
                if cur is not None:
                    self.seed_joints = list(cur)

        # Steps
        n = len(self.steps)
        for i, step in enumerate(self.steps, start=1):
            self.get_logger().info(
                f"[{i}/{n}] Step: {step.name}  xyz={list(step.xyz)}  rpy={list(step.rpy)}"
            )

            # Refresh current joints as seed to stay on the same branch
            cur = self.wait_for_joint_states(timeout_sec=2.0)
            if cur is not None:
                self.seed_joints = list(cur)

            if self.seed_joints is None:
                self.get_logger().error("No seed joints available.")
                return

            self.get_logger().info(
                f"  IK seed (first 3 joints): [{self.seed_joints[0]:.3f}, {self.seed_joints[1]:.3f}, {self.seed_joints[2]:.3f}]"
            )

            target_pose = self._make_pose(step.xyz, step.rpy)

            self.get_logger().info("  Requesting IK...")
            q_goal = self.call_ik(target_pose, self.seed_joints)
            if q_goal is None:
                self.get_logger().error(f"  IK failed at step '{step.name}'.")
                return

            self.get_logger().info("  Planning to IK solution...")
            traj = self.plan_to_joints(self.seed_joints, q_goal)
            if traj is None:
                self.get_logger().error(f"  Planning failed at step '{step.name}'.")
                return

            self.get_logger().info("  Executing trajectory...")
            if not self.execute_traj(traj):
                self.get_logger().error(f"  Execution failed at step '{step.name}'.")
                return

            self.get_logger().info(f"  Step '{step.name}' done.")
            self.spin_sleep(self.sleep_sec_between_steps)

            # Update seed after execution
            cur = self.wait_for_joint_states(timeout_sec=2.0)
            if cur is not None:
                self.seed_joints = list(cur)

        self._worker_ok = True
        self.get_logger().info("Pick&place sequence completed successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = UR5ePickPlaceRobust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
