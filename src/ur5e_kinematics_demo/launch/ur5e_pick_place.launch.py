from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- General / MoveIt
    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    ee_frame = DeclareLaunchArgument("ee_frame", default_value="tool0")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # --- Pick / Place target poses (position + orientation)
    # Use RPY by default, but you can extend to quaternion like in move_to_pose if needed.
    pick_xyz = DeclareLaunchArgument("pick_xyz", default_value="[0.45, 0.10, 0.12]")
    pick_rpy = DeclareLaunchArgument("pick_rpy", default_value="[0.0, 3.14159, 0.0]")

    place_xyz = DeclareLaunchArgument("place_xyz", default_value="[0.35, -0.25, 0.12]")
    place_rpy = DeclareLaunchArgument("place_rpy", default_value="[0.0, 3.14159, 0.0]")

    # --- Offsets (meters)
    pregrasp_z = DeclareLaunchArgument("pregrasp_z", default_value="0.12")
    approach_z = DeclareLaunchArgument("approach_z", default_value="0.08")   # not used if you go directly to grasp pose
    lift_z = DeclareLaunchArgument("lift_z", default_value="0.12")

    preplace_z = DeclareLaunchArgument("preplace_z", default_value="0.12")
    descend_z = DeclareLaunchArgument("descend_z", default_value="0.08")    # not used if you go directly to place pose
    retreat_z = DeclareLaunchArgument("retreat_z", default_value="0.10")

    # --- Motion limits
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.25")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.25")

    # --- Execution flags
    execute = DeclareLaunchArgument("execute", default_value="true")

    # --- Gripper integration (simple knobs; your node can use these to call services/actions)
    # Leave them empty for now; later you can pass the real service/action names.
    gripper_open_srv = DeclareLaunchArgument("gripper_open_srv", default_value="")
    gripper_close_srv = DeclareLaunchArgument("gripper_close_srv", default_value="")
    sleep_gripper_s = DeclareLaunchArgument("sleep_gripper_s", default_value="0.5")

    # --- Optional perception input (future-proof)
    # If enable_object_pose_sub:=true, your node can subscribe to object_pose_topic
    # and override pick pose dynamically.
    enable_object_pose_sub = DeclareLaunchArgument("enable_object_pose_sub", default_value="false")
    object_pose_topic = DeclareLaunchArgument("object_pose_topic", default_value="/object_pose")

    node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_pick_place_exe",
        name="ur5e_pick_place",
        output="screen",
        parameters=[
            {
                "group_name": LaunchConfiguration("group_name"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),

                "pick_xyz": LaunchConfiguration("pick_xyz"),
                "pick_rpy": LaunchConfiguration("pick_rpy"),
                "place_xyz": LaunchConfiguration("place_xyz"),
                "place_rpy": LaunchConfiguration("place_rpy"),

                "pregrasp_z": LaunchConfiguration("pregrasp_z"),
                "approach_z": LaunchConfiguration("approach_z"),
                "lift_z": LaunchConfiguration("lift_z"),
                "preplace_z": LaunchConfiguration("preplace_z"),
                "descend_z": LaunchConfiguration("descend_z"),
                "retreat_z": LaunchConfiguration("retreat_z"),

                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),

                "execute": LaunchConfiguration("execute"),

                "gripper_open_srv": LaunchConfiguration("gripper_open_srv"),
                "gripper_close_srv": LaunchConfiguration("gripper_close_srv"),
                "sleep_gripper_s": LaunchConfiguration("sleep_gripper_s"),

                "enable_object_pose_sub": LaunchConfiguration("enable_object_pose_sub"),
                "object_pose_topic": LaunchConfiguration("object_pose_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            group_name,
            base_frame,
            ee_frame,
            use_sim_time,

            pick_xyz,
            pick_rpy,
            place_xyz,
            place_rpy,

            pregrasp_z,
            approach_z,
            lift_z,
            preplace_z,
            descend_z,
            retreat_z,

            max_velocity,
            max_acceleration,

            execute,

            gripper_open_srv,
            gripper_close_srv,
            sleep_gripper_s,

            enable_object_pose_sub,
            object_pose_topic,

            node,
        ]
    )
