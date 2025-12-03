from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Target pose (position + RPY)
    target_x = DeclareLaunchArgument("target_x", default_value="0.4")
    target_y = DeclareLaunchArgument("target_y", default_value="0.0")
    target_z = DeclareLaunchArgument("target_z", default_value="0.3")

    target_roll = DeclareLaunchArgument("target_roll", default_value="0.0")
    target_pitch = DeclareLaunchArgument("target_pitch", default_value="3.14159")
    target_yaw = DeclareLaunchArgument("target_yaw", default_value="0.0")

    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    ik_link = DeclareLaunchArgument("ik_link", default_value="tool0")
    execute = DeclareLaunchArgument("execute", default_value="false")

    node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_inverse_kinematics_node",
        name="ur5e_ik_node",
        output="screen",
        parameters=[
            {
                "target_x": LaunchConfiguration("target_x"),
                "target_y": LaunchConfiguration("target_y"),
                "target_z": LaunchConfiguration("target_z"),
                "target_roll": LaunchConfiguration("target_roll"),
                "target_pitch": LaunchConfiguration("target_pitch"),
                "target_yaw": LaunchConfiguration("target_yaw"),
                "group_name": LaunchConfiguration("group_name"),
                "ik_link": LaunchConfiguration("ik_link"),
                "execute": LaunchConfiguration("execute"),
            }
        ],
    )

    return LaunchDescription(
        [
            target_x,
            target_y,
            target_z,
            target_roll,
            target_pitch,
            target_yaw,
            group_name,
            ik_link,
            execute,
            node,
        ]
    )
