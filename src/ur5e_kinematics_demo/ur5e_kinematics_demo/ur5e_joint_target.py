#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveUR5e(Node):
    def __init__(self):
        super().__init__("move_ur5e_joint_target")

        self.declare_parameter(
            "target_deg",
            [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]
        )
        self.declare_parameter("time_sec", 5.0)

        self.pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.sent = False

    def send_trajectory(self):
        if self.sent:
            return

        target_deg = self.get_parameter("target_deg").value
        time_sec = float(self.get_parameter("time_sec").value)

        if len(target_deg) != 6:
            self.get_logger().error("Parameter 'target_deg' must contain exactly 6 values")
            rclpy.shutdown()
            return

        target_rad = [math.radians(x) for x in target_deg]

        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = target_rad
        point.time_from_start = Duration(sec=int(time_sec), nanosec=0)

        msg.points.append(point)

        self.pub.publish(msg)
        self.get_logger().info(f"Published target_deg = {target_deg}")
        self.get_logger().info(f"Published target_rad = {target_rad}")

        self.sent = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5e()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()