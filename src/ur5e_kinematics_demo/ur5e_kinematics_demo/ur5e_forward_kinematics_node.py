#!/usr/bin/env python3
import ast
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2


class UR5eMoveJoints(Node):
    def __init__(self):
        super().__init__("ur5e_move_joints")

        # Params
        self.declare_parameter("joints", "[0.0, -1.57, 1.57, 0.0, 1.57, 0.0]")
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("execute", True)
        self.declare_parameter("use_sim_time", False)
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)

        # Parse joints robustly (string "[...]" or list)
        raw = self.get_parameter("joints").value
        if isinstance(raw, str):
            self.joints = [float(v) for v in ast.literal_eval(raw)]
        else:
            self.joints = [float(v) for v in raw]

        self.group_name = str(self.get_parameter("group_name").value)
        self.execute = bool(self.get_parameter("execute").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)

        # Wait for joint states before executing
        self._have_js = False
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _js_cb(self, msg: JointState):
        self._have_js = True

    def _run_once(self):
        if self._done:
            return
        if not self._have_js:
            self.get_logger().warn("Waiting for /joint_states...")
            return

        self._done = True
        self.get_logger().info(f"Target joints (rad): {self.joints}")

        if self.execute:
            self.moveit2.move_to_configuration(self.joints)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Motion finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eMoveJoints()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
