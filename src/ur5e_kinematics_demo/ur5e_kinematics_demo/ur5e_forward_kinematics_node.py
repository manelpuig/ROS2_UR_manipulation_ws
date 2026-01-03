#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2

from geometry_msgs.msg import TransformStamped
import tf2_ros

# ROS-friendly transformations (no SciPy dependency)
from tf_transformations import euler_from_quaternion


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

        # TF parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("wait_tf_sec", 5.0)

        self.joints = [float(v) for v in self.get_parameter("joints").value]
        self.group_name = str(self.get_parameter("group_name").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.fjt_action = str(self.get_parameter("follow_joint_traj_action").value)
        self.wait_js = float(self.get_parameter("wait_joint_states_sec").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.wait_tf_sec = float(self.get_parameter("wait_tf_sec").value)

        # Joint states gate
        self._have_js = False
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # TF listener (to read current EE pose from TF)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

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
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

    def _js_cb(self, _msg: JointState):
        self._have_js = True

    def _get_ee_pose(self):
        """
        Returns:
          (p, q_xyzw, rpy_rad)
            p = (x, y, z)
            q_xyzw = (qx, qy, qz, qw)
            rpy_rad = (roll, pitch, yaw) in radians

        Note:
          tf_transformations.euler_from_quaternion expects [x, y, z, w]
          and returns (roll, pitch, yaw) using the standard ROS convention.
        """
        t0 = self.get_clock().now()
        while rclpy.ok():
            # Keep spinning so TF messages are processed
            rclpy.spin_once(self, timeout_sec=0.05)

            try:
                tf: TransformStamped = self._tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time(),  # latest available
                )

                tr = tf.transform.translation
                rot = tf.transform.rotation

                p = (float(tr.x), float(tr.y), float(tr.z))
                q_xyzw = (float(rot.x), float(rot.y), float(rot.z), float(rot.w))

                roll, pitch, yaw = euler_from_quaternion(q_xyzw)  # (r, p, y)

                return p, q_xyzw, (float(roll), float(pitch), float(yaw))

            except Exception:
                if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.wait_tf_sec:
                    raise TimeoutError(
                        f"Timed out waiting for TF {self.base_frame} -> {self.ee_frame}"
                    )

    def _log_ee_pose(self, label: str):
        try:
            p, q_xyzw, rpy = self._get_ee_pose()
            self.get_logger().info(
                f"{label} EE pose in '{self.base_frame}' -> '{self.ee_frame}': "
                f"p=[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m, "
                f"q(xyzw)=[{q_xyzw[0]:.5f}, {q_xyzw[1]:.5f}, {q_xyzw[2]:.5f}, {q_xyzw[3]:.5f}], "
                f"rpy=[{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}] rad"
            )
        except Exception as e:
            self.get_logger().warn(f"{label} Could not read EE pose from TF: {e}")

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

        # Optional: log current EE pose before motion
        self._log_ee_pose("BEFORE")

        if not self.execute:
            self.get_logger().info("execute:=false -> exiting without motion.")
            return 0

        # 2) Execute
        self.get_logger().info("Sending joint configuration to MoveIt2...")
        try:
            self.moveit2.move_to_configuration(self.joints)
            self.moveit2.wait_until_executed()
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted by user (Ctrl+C).")
            return 130

        self.get_logger().info("Motion finished.")

        # Log obtained EE pose after motion
        self._log_ee_pose("AFTER")

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
