#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

from pymoveit2 import MoveIt2
from spatialmath.base import rpy2q


UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5ePickPlaceSimple(Node):
    """
    Simple pick&place sequence using MoveIt2 planning for main moves
    and short Cartesian approach/retreat segments.

    This is intentionally "minimal" to be reliable and easy to extend.
    """

    def __init__(self):
        super().__init__("ur5e_pick_place_simple")

        # Frames / group
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")

        # Pick pose (xyz + rpy)
        self.declare_parameter("pick_xyz", [0.45, 0.10, 0.12])
        self.declare_parameter("pick_rpy", [0.0, math.pi, 0.0])

        # Place pose (xyz + rpy)
        self.declare_parameter("place_xyz", [0.35, -0.25, 0.12])
        self.declare_parameter("place_rpy", [0.0, math.pi, 0.0])

        # Offsets (meters)
        self.declare_parameter("pregrasp_z", 0.12)     # above pick
        self.declare_parameter("approach_z", 0.08)     # approach distance down
        self.declare_parameter("lift_z", 0.12)         # lift after grasp

        self.declare_parameter("preplace_z", 0.12)     # above place
        self.declare_parameter("descend_z", 0.08)      # descend distance down
        self.declare_parameter("retreat_z", 0.10)      # retreat up

        # Motion params
        self.declare_parameter("max_velocity", 0.25)
        self.declare_parameter("max_acceleration", 0.25)

        # Execution flags
        self.declare_parameter("execute", True)
        self.declare_parameter("sleep_gripper_s", 0.5)

        # Read params
        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.pick_xyz = [float(x) for x in self.get_parameter("pick_xyz").value]
        self.pick_rpy = [float(x) for x in self.get_parameter("pick_rpy").value]
        self.place_xyz = [float(x) for x in self.get_parameter("place_xyz").value]
        self.place_rpy = [float(x) for x in self.get_parameter("place_rpy").value]

        self.pregrasp_z = float(self.get_parameter("pregrasp_z").value)
        self.approach_z = float(self.get_parameter("approach_z").value)
        self.lift_z = float(self.get_parameter("lift_z").value)
        self.preplace_z = float(self.get_parameter("preplace_z").value)
        self.descend_z = float(self.get_parameter("descend_z").value)
        self.retreat_z = float(self.get_parameter("retreat_z").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.sleep_gripper_s = float(self.get_parameter("sleep_gripper_s").value)

        # MoveIt2 client
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(0.1, self._run_once)

    # -----------------------
    # Helpers
    # -----------------------
    def _pose_from_xyz_rpy(self, xyz, rpy):
        qw, qx, qy, qz = rpy2q(*rpy, order="xyz", unit="rad")
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x = float(qx)
        p.orientation.y = float(qy)
        p.orientation.z = float(qz)
        p.orientation.w = float(qw)
        return p

    def _move_to_pose(self, pose: Pose):
        if hasattr(self.moveit2, "move_to_pose"):
            self.moveit2.move_to_pose(pose)
        else:
            # Some versions require PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose = pose
            if hasattr(self.moveit2, "move_to_pose_stamped"):
                self.moveit2.move_to_pose_stamped(ps)
            else:
                raise RuntimeError("pymoveit2 does not provide move_to_pose or move_to_pose_stamped.")
        self.moveit2.wait_until_executed()

    def _cartesian_delta_z(self, dz: float):
        """
        Small Cartesian move along Z in base_frame direction.
        Depending on your pymoveit2 version, there may be a dedicated cartesian API.
        If not available, you can approximate by sending a new pose with z offset.
        """
        if hasattr(self.moveit2, "compute_cartesian_path") and hasattr(self.moveit2, "execute"):
            # If your pymoveit2 exposes a cartesian planner, use it (best).
            # NOTE: API names differ by version; adapt if needed.
            raise NotImplementedError("Cartesian API exists but requires version-specific call signature.")
        else:
            # Conservative fallback: re-command pose with adjusted z
            # This assumes current pose is close; for small dz it works well.
            current = self.moveit2.get_current_pose() if hasattr(self.moveit2, "get_current_pose") else None
            if current is None:
                raise RuntimeError("No cartesian API and cannot get current pose; update to pymoveit2 version with pose getter.")
            target = Pose()
            target.position.x = current.position.x
            target.position.y = current.position.y
            target.position.z = current.position.z + dz
            target.orientation = current.orientation
            self._move_to_pose(target)

    # -----------------------
    # Gripper stubs (replace with your real interface)
    # -----------------------
    def gripper_open(self):
        self.get_logger().info("Gripper OPEN (stub). Replace with your gripper action/service.")
        time.sleep(self.sleep_gripper_s)

    def gripper_close(self):
        self.get_logger().info("Gripper CLOSE (stub). Replace with your gripper action/service.")
        time.sleep(self.sleep_gripper_s)

    # -----------------------
    # Main sequence
    # -----------------------
    def _run_once(self):
        if self._done:
            return
        self._done = True

        if not self.execute_motion:
            self.get_logger().warn("execute:=false, not moving. Shutting down.")
            rclpy.shutdown()
            return

        # Build poses
        pick_pose = self._pose_from_xyz_rpy(self.pick_xyz, self.pick_rpy)
        place_pose = self._pose_from_xyz_rpy(self.place_xyz, self.place_rpy)

        pregrasp = self._pose_from_xyz_rpy(
            [self.pick_xyz[0], self.pick_xyz[1], self.pick_xyz[2] + self.pregrasp_z],
            self.pick_rpy,
        )
        grasp = self._pose_from_xyz_rpy(
            [self.pick_xyz[0], self.pick_xyz[1], self.pick_xyz[2]],
            self.pick_rpy,
        )
        lift = self._pose_from_xyz_rpy(
            [self.pick_xyz[0], self.pick_xyz[1], self.pick_xyz[2] + self.lift_z],
            self.pick_rpy,
        )

        preplace = self._pose_from_xyz_rpy(
            [self.place_xyz[0], self.place_xyz[1], self.place_xyz[2] + self.preplace_z],
            self.place_rpy,
        )
        place = self._pose_from_xyz_rpy(
            [self.place_xyz[0], self.place_xyz[1], self.place_xyz[2]],
            self.place_rpy,
        )
        retreat = self._pose_from_xyz_rpy(
            [self.place_xyz[0], self.place_xyz[1], self.place_xyz[2] + self.retreat_z],
            self.place_rpy,
        )

        # ---- Sequence
        self.get_logger().info("1) Open gripper")
        self.gripper_open()

        self.get_logger().info("2) Move to PREGRASP")
        self._move_to_pose(pregrasp)

        self.get_logger().info("3) Approach to GRASP (short move)")
        self._move_to_pose(grasp)

        self.get_logger().info("4) Close gripper")
        self.gripper_close()

        self.get_logger().info("5) Lift")
        self._move_to_pose(lift)

        self.get_logger().info("6) Move to PREPLACE")
        self._move_to_pose(preplace)

        self.get_logger().info("7) Descend to PLACE")
        self._move_to_pose(place)

        self.get_logger().info("8) Open gripper")
        self.gripper_open()

        self.get_logger().info("9) Retreat")
        self._move_to_pose(retreat)

        self.get_logger().info("Pick&Place finished.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5ePickPlaceSimple()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
