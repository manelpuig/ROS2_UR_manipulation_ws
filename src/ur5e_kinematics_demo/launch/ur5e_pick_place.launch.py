#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _build(context, *args, **kwargs):
    cfg_path = LaunchConfiguration("config").perform(context)
    data = _load_yaml(cfg_path)

    common = data.get("common", {})
    steps = data.get("steps", [])

    if not isinstance(steps, list) or len(steps) < 2:
        raise RuntimeError(f"YAML 'steps' ha de ser una llista amb almenys 2 elements: {cfg_path}")

    # Crea Node() per cada pas
    nodes = []
    for idx, step in enumerate(steps, start=1):
        name = step.get("name", f"step{idx}")
        params = dict(common)

        # Paràmetres requerits pel teu node ur5e_move_to_pose.py
        if "target_xyz" not in step or "target_rpy" not in step:
            raise RuntimeError(f"Pas '{name}' ha de contenir 'target_xyz' i 'target_rpy'")

        params["target_xyz"] = [float(x) for x in step["target_xyz"]]
        params["target_rpy"] = [float(x) for x in step["target_rpy"]]

        # Opcional: permet sobreescriure common per pas
        for k in ("execute", "group_name", "ik_link", "max_velocity", "max_acceleration", "print_joints"):
            if k in step:
                params[k] = step[k]

        node = Node(
            package="ur5e_kinematics_demo",
            executable="ur5e_move_to_pose_exe",
            name=f"ur5e_move_{name}",
            output="screen",
            parameters=[params],
        )
        nodes.append((node, float(step.get("sleep_after", 0.5))))

    # Llança el primer node immediatament
    actions = [nodes[0][0]]

    # Encadena la resta: quan acaba el node i, espera sleep_after_i i llança node i+1
    for (current_node, sleep_after), (next_node, _) in zip(nodes[:-1], nodes[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=current_node,
                    on_exit=[TimerAction(period=sleep_after, actions=[next_node])],
                )
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("ur5e_kinematics_demo")
    default_cfg = os.path.join(pkg_share, "config", "ur5e_pick_place.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_cfg,
            description="YAML de pick&place (common + steps)",
        ),
        OpaqueFunction(function=_build),
    ])
