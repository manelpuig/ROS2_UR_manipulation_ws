from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    target_xyz = DeclareLaunchArgument("target_xyz", default_value="[0.4, 0.0, 0.3]")
    target_rpy = DeclareLaunchArgument("target_rpy", default_value="[0.0, 3.14159, 0.0]")
    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value="[0.0, -1.57, 1.57, 0.0, 1.57, 0.0]",
        description="IK seed joint configuration [rad] in UR5e order",
    )

    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    ik_link = DeclareLaunchArgument("ik_link", default_value="tool0")
    execute = DeclareLaunchArgument("execute", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_inverse_kinematics_node",
        name="ur5e_inverse_kinematics_node",
        output="screen",
        parameters=[
            {
                "target_xyz": LaunchConfiguration("target_xyz"),
                "target_rpy": LaunchConfiguration("target_rpy"),
                "seed_joints": LaunchConfiguration("seed_joints"),
                "group_name": LaunchConfiguration("group_name"),
                "ik_link": LaunchConfiguration("ik_link"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            target_xyz,
            target_rpy,
            seed_joints,
            group_name,
            ik_link,
            execute,
            use_sim_time,
            node,
        ]
    )
