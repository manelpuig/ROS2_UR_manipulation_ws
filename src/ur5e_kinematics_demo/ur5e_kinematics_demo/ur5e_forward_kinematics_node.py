#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2


class UR5eMoveJoints(Node):
    def __init__(self):
        super().__init__("ur5e_move_joints")

        # Parameters
        self.declare_parameter("joints", [0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("execute", True)
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter(
            "follow_joint_traj_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("wait_joint_states_sec", 10.0)

        self.joints = [float(v) for v in self.get_parameter("joints").value]
        self.group_name = str(self.get_parameter("group_name").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.fjt_action = str(self.get_parameter("follow_joint_traj_action").value)
        self.wait_js = float(self.get_parameter("wait_joint_states_sec").value)

        # Joint states gate
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

    def _js_cb(self, _msg: JointState):
        self._have_js = True

    def run_once(self):
        self.get_logger().info(f"Target joints (rad): {self.joints}")

        # 1) Wait for joint states
        t0 = self.get_clock().now()
        while rclpy.ok() and not self._have_js:
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.wait_js:
                self.get_logger().error("Timed out waiting for /joint_states.")
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Joint states are available now.")

        # 2) Check FollowJointTrajectory action exists (key for Gazebo execution)
        actions = self.get_action_names_and_types()
        if not any(name == self.fjt_action for name, _types in actions):
            self.get_logger().error(
                f"Missing action '{self.fjt_action}'. "
                "MoveIt may be in fake execution or controller is not exposed. "
                "Gazebo will not move until this exists."
            )
            self.get_logger().info("Hint: ros2 action list | grep -i follow_joint_trajectory")
            return 2

        if not self.execute:
            self.get_logger().info("execute:=false -> exiting without motion.")
            return 0

        # 3) Execute
        self.get_logger().info("Sending joint configuration to MoveIt2...")
        try:
            self.moveit2.move_to_configuration(self.joints)
            self.moveit2.wait_until_executed()
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted by user (Ctrl+C).")
            return 130

        self.get_logger().info("Motion finished.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveJoints()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()