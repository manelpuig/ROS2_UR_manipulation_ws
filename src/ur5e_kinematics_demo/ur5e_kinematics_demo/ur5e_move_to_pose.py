#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pymoveit2 import MoveIt2
from spatialmath.base import rpy2q  # returns [w, x, y, z]


UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5eMoveToPose(Node):
    """
    Minimal node to move UR5e end-effector to a target pose using MoveIt2.
    Works for simulation and real robot (same code), backend must provide MoveIt action servers.
    """

    def __init__(self):
        super().__init__("ur5e_move_to_pose")

        # ---- Parameters
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("use_sim_time", False)

        # Target pose parameters
        self.declare_parameter("target_xyz", [0.40, 0.00, 0.30])
        self.declare_parameter("target_rpy", [0.0, math.pi, 0.0])  # roll,pitch,yaw [rad]
        self.declare_parameter("use_quat", False)
        self.declare_parameter("target_quat_xyzw", [0.0, 0.0, 0.0, 1.0])

        # Motion params
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]
        self.use_quat = bool(self.get_parameter("use_quat").value)
        self.target_quat_xyzw = [float(x) for x in self.get_parameter("target_quat_xyzw").value]

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)

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

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _run_once(self):
        if self._done:
            return
        self._done = True

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target_xyz

        if self.use_quat:
            qx, qy, qz, qw = self.target_quat_xyzw
        else:
            # spatialmath returns [w,x,y,z]
            qw, qx, qy, qz = rpy2q(*self.target_rpy, order="xyz", unit="rad")

        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.get_logger().info(
            f"Moving to pose in {self.base_frame}: "
            f"xyz={self.target_xyz}, quat_xyzw={[qx, qy, qz, qw]}"
        )

        if self.execute_motion:
            # Depending on pymoveit2 version, this may be move_to_pose / move_to_pose_stamped
            if hasattr(self.moveit2, "move_to_pose"):
                self.moveit2.move_to_pose(pose.pose)
            elif hasattr(self.moveit2, "move_to_pose_stamped"):
                self.moveit2.move_to_pose_stamped(pose)
            else:
                raise RuntimeError("pymoveit2 MoveIt2 does not expose a move_to_pose method in this version.")

            self.moveit2.wait_until_executed()
            self.get_logger().info("Motion finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eMoveToPose()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
