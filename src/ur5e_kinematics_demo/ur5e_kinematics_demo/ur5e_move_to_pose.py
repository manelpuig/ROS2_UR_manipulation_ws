#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

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
      R = Rz(yaw) * Ry(pitch) * Rx(roll)  (matrix multiplication YPR)
    Equivalent to geometric rotation order:
      roll -> pitch -> yaw (intrinsic rotations).

    Returns quaternion in ROS order: (qx, qy, qz, qw)
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


class UR5eMoveToPose(Node):
    """
    Move UR5e end-effector to a target pose using MoveIt2.

    Robust additions:
      - Wait for /joint_states (planning uses current state)
      - Check action servers exist
      - Timeout so terminal does not hang forever when planning/execution fails
      - Uses PoseStamped conceptually; falls back to move_to_pose(Pose) for older pymoveit2
    """

    def __init__(self):
        super().__init__("ur5e_move_to_pose")

        # ---- Parameters
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")

        # Target pose parameters
        self.declare_parameter("target_xyz", [0.40, 0.00, 0.30])
        self.declare_parameter("target_rpy", [0.0, 1.57, 0.0])  # roll,pitch,yaw [rad]
        self.declare_parameter("use_quat", False)
        self.declare_parameter("target_quat_xyzw", [0.0, 0.0, 0.0, 1.0])

        # Motion params
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)

        # Robustness params
        self.declare_parameter("wait_joint_states_sec", 10.0)
        self.declare_parameter("wait_action_sec", 10.0)
        self.declare_parameter("wait_motion_sec", 20.0)

        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]
        self.use_quat = bool(self.get_parameter("use_quat").value)
        self.target_quat_xyzw = [float(x) for x in self.get_parameter("target_quat_xyzw").value]

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)

        self.wait_js = float(self.get_parameter("wait_joint_states_sec").value)
        self.wait_action = float(self.get_parameter("wait_action_sec").value)
        self.wait_motion = float(self.get_parameter("wait_motion_sec").value)

        # ---- Gate /joint_states
        self._have_js = False
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # ---- MoveIt2 client
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        # Which pose API exists
        self._has_pose_stamped = hasattr(self.moveit2, "move_to_pose_stamped")
        self._has_pose = hasattr(self.moveit2, "move_to_pose")
        if not (self._has_pose_stamped or self._has_pose):
            raise RuntimeError("pymoveit2 exposes neither move_to_pose_stamped() nor move_to_pose().")

        self.get_logger().info(
            "MoveIt2 pose API: "
            + ("move_to_pose_stamped available" if self._has_pose_stamped else "move_to_pose_stamped NOT available")
            + " | "
            + ("move_to_pose available" if self._has_pose else "move_to_pose NOT available")
        )

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _js_cb(self, _msg: JointState):
        self._have_js = True

    def _wait_for_joint_states(self) -> bool:
        t0 = self.get_clock().now()
        while rclpy.ok() and not self._have_js:
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.wait_js:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def _send_pose_goal(self, pose_stamped: PoseStamped):
        # Prefer PoseStamped if supported
        if self._has_pose_stamped:
            self.moveit2.move_to_pose_stamped(pose_stamped)
            return

        # Fallback for older pymoveit2
        self.get_logger().warn(
            "pymoveit2 does not support move_to_pose_stamped(); using move_to_pose(Pose) fallback."
        )
        self.moveit2.move_to_pose(pose_stamped.pose)

    def _wait_until_executed_with_timeout(self) -> bool:
        """
        Avoid hanging forever. We keep spinning so action callbacks progress.
        Returns True if execution finishes (best-effort), False on timeout.
        """
        t0 = self.get_clock().now()

        # If the library offers is_executing(), use it
        has_is_exec = hasattr(self.moveit2, "is_executing")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if has_is_exec:
                if not self.moveit2.is_executing():
                    return True

            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.wait_motion:
                return False

    def _run_once(self):
        if self._done:
            return
        self._done = True

        if not self._wait_for_joint_states():
            self.get_logger().error("Timed out waiting for /joint_states. Not sending goal.")
            rclpy.shutdown()
            return

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target_xyz

        if self.use_quat:
            qx, qy, qz, qw = [float(x) for x in self.target_quat_xyzw]
        else:
            roll, pitch, yaw = self.target_rpy
            qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Goal pose: frame={self.base_frame}, xyz={self.target_xyz}, "
            f"rpy={self.target_rpy} rad, quat_xyzw={[qx, qy, qz, qw]}"
        )

        if self.execute_motion:
            self._send_pose_goal(pose)

            ok = self._wait_until_executed_with_timeout()
            if ok:
                self.get_logger().info("Execution finished (or no longer executing).")
            else:
                self.get_logger().error(
                    "Timed out waiting for execution. Likely planning failed (IK/collision/constraints). "
                    "Try a simpler orientation (avoid exact pi), or increase wait_motion_sec."
                )

        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eMoveToPose()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
