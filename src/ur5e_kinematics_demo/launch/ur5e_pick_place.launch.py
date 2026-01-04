#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _build(context, *args, **kwargs):
    pkg_share = get_package_share_directory("ur5e_kinematics_demo")
    cfg_path = os.path.join(pkg_share, "config", "ur5e_pick_place.yaml")

    with open(cfg_path, "r") as f:
        data = yaml.safe_load(f)

    common = data["common"]
    steps = data["steps"]

    nodes = []
    for s in steps:
        params = {
            **common,
            "target_xyz": s["target_xyz"],
            "target_rpy": s["target_rpy"],
        }

        if "seed_from_joint_states" in s:
            params["seed_from_joint_states"] = s["seed_from_joint_states"]
        if "seed_joints" in s:
            params["seed_joints"] = s["seed_joints"]

        node = Node(
            package="ur5e_kinematics_demo",
            executable="ur5e_move_to_pose_exe",
            name=f'ur5e_move_{s["name"]}',
            output="screen",
            parameters=[params],
        )

        nodes.append((node, float(s.get("sleep_after", 0.5))))

    actions = [nodes[0][0]]

    for (cur, dt), (nxt, _) in zip(nodes[:-1], nodes[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=cur,
                    on_exit=[TimerAction(period=dt, actions=[nxt])],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build)])
