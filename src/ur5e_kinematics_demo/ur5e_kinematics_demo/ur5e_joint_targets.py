#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveUR5eTrajectory(Node):
    def __init__(self):
        super().__init__("move_ur5e_joint_targets")

        self.declare_parameter(
            "targets_deg",
            [
                [0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 3.0],
                [20.0, -70.0, 110.0, -40.0, 80.0, 10.0, 6.0],
                [10.0, -80.0, 100.0, -20.0, 85.0, 0.0, 9.0],
            ],
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.sent = False

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def send_trajectory(self):
        if self.sent:
            return

        targets_deg = self.get_parameter("targets_deg").value

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for row in targets_deg:
            if len(row) != 7:
                self.get_logger().error(
                    "Each target must contain 7 values: 6 joint angles in deg + time_sec"
                )
                rclpy.shutdown()
                return

            q_deg = row[:6]
            t_sec = float(row[6])

            point = JointTrajectoryPoint()
            point.positions = [math.radians(q) for q in q_deg]

            sec = int(t_sec)
            nanosec = int((t_sec - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            msg.points.append(point)

        self.pub.publish(msg)
        self.get_logger().info(f"Published trajectory with {len(msg.points)} points")

        self.sent = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()