import math
from typing import List

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK


class UR5eForwardKinematicsNode(Node):
    '''
    Simple node that calls MoveIt's /compute_fk service for the UR5e.

    Parameters (declared as ROS parameters):
    - joint1 ... joint6: joint angles in radians
    - group_name: MoveIt planning group (default: "ur_manipulator")
    - fk_link: end-effector link for FK (default: "tool0")
    '''

    def __init__(self):
        super().__init__('ur5e_forward_kinematics_node')

        # Declare parameters
        self.declare_parameter('joint1', 0.0)
        self.declare_parameter('joint2', 0.0)
        self.declare_parameter('joint3', 0.0)
        self.declare_parameter('joint4', 0.0)
        self.declare_parameter('joint5', 0.0)
        self.declare_parameter('joint6', 0.0)
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('fk_link', 'tool0')

        # Read parameters
        self.joints = [
            float(self.get_parameter('joint1').value),
            float(self.get_parameter('joint2').value),
            float(self.get_parameter('joint3').value),
            float(self.get_parameter('joint4').value),
            float(self.get_parameter('joint5').value),
            float(self.get_parameter('joint6').value),
        ]
        self.group_name = self.get_parameter('group_name').value
        self.fk_link = self.get_parameter('fk_link').value

        self.get_logger().info(f"Using joints (rad): {self.joints}")
        self.get_logger().info(f"Group: {self.group_name}, FK link: {self.fk_link}")

        # FK service client
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        self.get_logger().info("Waiting for /compute_fk service...")
        self.fk_client.wait_for_service()
        self.get_logger().info("Connected to /compute_fk service.")

        # Call FK once on startup
        self.compute_fk()

    def compute_fk(self):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'   # reference frame for the pose
        request.header.stamp = self.get_clock().now().to_msg()

        # We want the pose of this link
        request.fk_link_names.append(self.fk_link)

        # Build JointState
        js = JointState()
        js.header = request.header
        js.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        js.position = self.joints

        request.robot_state.joint_state = js

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('FK service call failed')
            return

        response = future.result()
        if len(response.pose_stamped) == 0:
            self.get_logger().error('FK response contains no poses')
            return

        pose: PoseStamped = response.pose_stamped[0]
        self.get_logger().info(f"FK result for link '{self.fk_link}':")
        self.get_logger().info(f"  Position: x={pose.pose.position.x:.3f}, "
                               f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")
        self.get_logger().info(f"  Orientation (quaternion): "
                               f"x={pose.pose.orientation.x:.3f}, "
                               f"y={pose.pose.orientation.y:.3f}, "
                               f"z={pose.pose.orientation.z:.3f}, "
                               f"w={pose.pose.orientation.w:.3f}")

        # Optional: shut down once done
        self.get_logger().info("FK computation finished. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eForwardKinematicsNode()
    # No spin needed: node calls FK once in constructor and then shuts down.