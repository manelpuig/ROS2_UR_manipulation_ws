from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Frames / group
    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    ee_frame = DeclareLaunchArgument("ee_frame", default_value="tool0")

    # Target pose: xyz + rpy
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.4, 0.0, 0.3]",
        description="Target position [m] as [x,y,z] in base_frame",
    )
    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 3.14159, 0.0]",
        description="Target orientation [rad] as [roll,pitch,yaw] (XYZ order)",
    )

    # Optional: quaternion mode
    use_quat = DeclareLaunchArgument("use_quat", default_value="false")
    target_quat_xyzw = DeclareLaunchArgument(
        "target_quat_xyzw",
        default_value="[0.0, 0.0, 0.0, 1.0]",
        description="Quaternion [qx,qy,qz,qw] used if use_quat:=true",
    )

    # Motion / runtime
    execute = DeclareLaunchArgument("execute", default_value="true")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.3")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.3")

    node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_move_to_pose_exe",
        name="ur5e_move_to_pose",
        output="screen",
        parameters=[
            {
                "group_name": LaunchConfiguration("group_name"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "target_xyz": LaunchConfiguration("target_xyz"),
                "target_rpy": LaunchConfiguration("target_rpy"),
                "use_quat": LaunchConfiguration("use_quat"),
                "target_quat_xyzw": LaunchConfiguration("target_quat_xyzw"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
            }
        ],
    )

    return LaunchDescription(
        [
            group_name,
            base_frame,
            ee_frame,
            target_xyz,
            target_rpy,
            use_quat,
            target_quat_xyzw,
            execute,
            use_sim_time,
            max_velocity,
            max_acceleration,
            node,
        ]
    )
