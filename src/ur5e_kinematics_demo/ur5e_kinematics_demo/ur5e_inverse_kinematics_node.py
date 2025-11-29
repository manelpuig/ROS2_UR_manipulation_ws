import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK


def rpy_to_quaternion(roll, pitch, yaw):
    '''
    Convert roll, pitch, yaw (rad) to a quaternion (x, y, z, w).
    '''
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

    return qx, qy, qz, qw


class UR5eInverseKinematicsNode(Node):
    '''
    Simple node that calls MoveIt's /compute_ik service for the UR5e.

    Parameters:
    - target_x, target_y, target_z: position [m]
    - target_roll, target_pitch, target_yaw: orientation [rad]
    - group_name: planning group (default: "ur_manipulator")
    - ik_link: end-effector link (default: "tool0")
    - seed_joint1 ... seed_joint6: optional seed joint configuration [rad]
    '''

    def __init__(self):
        super().__init__('ur5e_inverse_kinematics_node')

        # Target pose parameters
        self.declare_parameter('target_x', 0.4)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.3)
        self.declare_parameter('target_roll', 0.0)
        self.declare_parameter('target_pitch', math.pi)  # typical UR tool pointing down
        self.declare_parameter('target_yaw', 0.0)

        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('ik_link', 'tool0')

        # Seed joints (optional)
        self.declare_parameter('seed_joint1', 0.0)
        self.declare_parameter('seed_joint2', -math.pi/2)
        self.declare_parameter('seed_joint3', math.pi/2)
        self.declare_parameter('seed_joint4', 0.0)
        self.declare_parameter('seed_joint5', math.pi/2)
        self.declare_parameter('seed_joint6', 0.0)

        self.group_name = self.get_parameter('group_name').value
        self.ik_link = self.get_parameter('ik_link').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.target_roll = float(self.get_parameter('target_roll').value)
        self.target_pitch = float(self.get_parameter('target_pitch').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        self.seed_joints = [
            float(self.get_parameter('seed_joint1').value),
            float(self.get_parameter('seed_joint2').value),
            float(self.get_parameter('seed_joint3').value),
            float(self.get_parameter('seed_joint4').value),
            float(self.get_parameter('seed_joint5').value),
            float(self.get_parameter('seed_joint6').value),
        ]

        self.get_logger().info(f"Target position: ({self.target_x}, {self.target_y}, {self.target_z})")
        self.get_logger().info(f"Target RPY (rad): ({self.target_roll}, {self.target_pitch}, {self.target_yaw})")
        self.get_logger().info(f"Group: {self.group_name}, IK link: {self.ik_link}")
        self.get_logger().info(f"Seed joints (rad): {self.seed_joints}")

        # IK client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info("Waiting for /compute_ik service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("Connected to /compute_ik service.")

        self.compute_ik()

    def compute_ik(self):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link

        # Target pose
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z

        qx, qy, qz, qw = rpy_to_quaternion(self.target_roll,
                                           self.target_pitch,
                                           self.target_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request.ik_request.pose_stamped = pose

        # Seed state
        js = JointState()
        js.header = pose.header
        js.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        js.position = self.seed_joints
        request.ik_request.robot_state.joint_state = js

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('IK service call failed')
            return

        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(f"IK failed with error code: {response.error_code.val}")
            return

        solution = response.solution.joint_state
        self.get_logger().info("IK solution:")
        for name, pos in zip(solution.name, solution.position):
            self.get_logger().info(f"  {name}: {pos:.4f} rad")

        self.get_logger().info("IK computation finished. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eInverseKinematicsNode()
    # No spin needed: node calls IK once and then shuts down.