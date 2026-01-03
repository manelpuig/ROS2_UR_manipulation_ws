#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Basic
    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    ik_link = DeclareLaunchArgument("ik_link", default_value="tool0")
    execute = DeclareLaunchArgument("execute", default_value="true")

    # --- Motion
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.25")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.25")

    # --- IK
    ik_timeout_sec = DeclareLaunchArgument("ik_timeout_sec", default_value="0.3")
    avoid_collisions = DeclareLaunchArgument("avoid_collisions", default_value="true")

    seed_mode = DeclareLaunchArgument(
        "seed_mode",
        default_value="last_goal",
        description="IK seed mode: fixed | last_goal",
    )

    # --- Home first (known starting posture)
    use_home_first = DeclareLaunchArgument("use_home_first", default_value="true")

    # A common UR5e "ready" posture in many examples/sims.
    # If you already have a preferred one, keep it and align pick/place around it.
    home_joints = DeclareLaunchArgument(
        "home_joints",
        default_value='[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]',
        description="Home joints executed before the sequence (UR5e joint order).",
    )

    # Seed should be close to HOME to bias IK toward that branch
    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value=LaunchConfiguration("home_joints"),
        description="Initial IK seed joints (typically same as home_joints).",
    )

    # --- Pick & Place geometry (coherent with HOME)
    # Conservative "front workspace" positions in base_link:
    # - x in [0.35..0.50]
    # - y small (symmetric)
    # - z above ground/table (adjust if you have a table collision object)
    pick_xyz = DeclareLaunchArgument(
        "pick_xyz",
        default_value='[0.45, 0.15, 0.20]',
        description="Pick position [x,y,z] in base_link [m].",
    )
    place_xyz = DeclareLaunchArgument(
        "place_xyz",
        default_value='[0.45, -0.15, 0.20]',
        description="Place position [x,y,z] in base_link [m].",
    )

    # Approach/lift (keep vertical moves to simplify)
    z_approach = DeclareLaunchArgument("z_approach", default_value="0.12")
    z_lift = DeclareLaunchArgument("z_lift", default_value="0.14")

    # --- Orientation: tool0 looking "down"
    # Use pitch ~ 3.10 rad instead of pi to avoid edge/singularity-like numerical issues.
    # RPY here is interpreted in the node with R = Rz(yaw) * Ry(pitch) * Rx(roll).
    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value='[0.0, 3.10, 0.0]',
        description="Tool orientation as RPY [rad]. Downward tool: pitch≈3.10 instead of pi.",
    )

    # Teaching/visualization
    sleep_sec_between_steps = DeclareLaunchArgument("sleep_sec_between_steps", default_value="0.3")

    node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_pick_place_exe",
        name="ur5e_pick_place_via_ik",
        output="screen",
        parameters=[
            {
                "group_name": LaunchConfiguration("group_name"),
                "ik_link": LaunchConfiguration("ik_link"),
                "execute": LaunchConfiguration("execute"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
                "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
                "avoid_collisions": LaunchConfiguration("avoid_collisions"),
                "seed_mode": LaunchConfiguration("seed_mode"),
                "seed_joints": LaunchConfiguration("seed_joints"),
                "use_home_first": LaunchConfiguration("use_home_first"),
                "home_joints": LaunchConfiguration("home_joints"),
                "pick_xyz": LaunchConfiguration("pick_xyz"),
                "place_xyz": LaunchConfiguration("place_xyz"),
                "z_approach": LaunchConfiguration("z_approach"),
                "z_lift": LaunchConfiguration("z_lift"),
                "target_rpy": LaunchConfiguration("target_rpy"),
                "sleep_sec_between_steps": LaunchConfiguration("sleep_sec_between_steps"),
            }
        ],
    )

    return LaunchDescription(
        [
            group_name,
            ik_link,
            execute,
            max_velocity,
            max_acceleration,
            ik_timeout_sec,
            avoid_collisions,
            seed_mode,
            use_home_first,
            home_joints,
            seed_joints,
            pick_xyz,
            place_xyz,
            z_approach,
            z_lift,
            target_rpy,
            sleep_sec_between_steps,
            node,
        ]
    )
