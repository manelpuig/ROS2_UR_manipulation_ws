from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "targets_deg",
            default_value="""
            [
              [0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 3.0],
              [20.0, -70.0, 110.0, -40.0, 80.0, 10.0, 6.0],
              [10.0, -80.0, 100.0, -20.0, 85.0, 0.0, 9.0]
            ]
            """
        ),

        Node(
            package="ur5e_kinematics_demo",
            executable="ur5e_joint_targets_exec",
            output="screen",
            parameters=[{
                "targets_deg": LaunchConfiguration("targets_deg"),
            }],
        )
    ])