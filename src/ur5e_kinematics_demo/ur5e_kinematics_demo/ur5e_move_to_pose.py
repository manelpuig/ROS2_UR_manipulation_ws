#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
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

class UR5eMoveToPoseMin(Node):
    def __init__(self):
        super().__init__("ur5e_move_to_pose")

        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")

        self.declare_parameter("target_xyz", [0.35, 0.00, 0.45])
        self.declare_parameter("target_rpy", [0.0, 0.0, 0.0])  # roll,pitch,yaw [rad]

        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)

        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.execute_motion = bool(self.get_parameter("execute").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        # Run once
        self.create_timer(0.1, self.run_once)
        self._done = False

    def run_once(self):
        if self._done:
            return
        self._done = True

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target_xyz

        roll, pitch, yaw = self.target_rpy
        qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Goal pose: xyz={self.target_xyz}, rpy={self.target_rpy}, quat_xyzw={[qx,qy,qz,qw]}"
        )

        if self.execute_motion:
            # Your pymoveit2 has move_to_pose(), so use it directly.
            self.moveit2.move_to_pose(pose.pose)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Execution finished.")
        else:
            self.get_logger().info("execute:=false -> not moving.")

        rclpy.shutdown()

def main():
    rclpy.init()
    node = UR5eMoveToPoseMin()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
