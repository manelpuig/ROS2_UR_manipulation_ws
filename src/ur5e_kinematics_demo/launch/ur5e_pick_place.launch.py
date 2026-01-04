#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # -------- HOME --------
    home_node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_move_to_pose_exe",
        name="ur5e_move_home",
        output="screen",
        parameters=[
            {
                "target_xyz": [0.35, 0.0, 0.45],
                "target_rpy": [0.0, 0.0, 0.0],
                "execute": True,
            }
        ],
    )

    # -------- PICK --------
    pick_node = Node(
        package="ur5e_kinematics_demo",
        executable="ur5e_move_to_pose_exe",
        name="ur5e_move_pick",
        output="screen",
        parameters=[
            {
                "target_xyz": [0.45, 0.15, 0.20],
                "target_rpy": [0.0, 3.1, 0.0],
                "execute": True,
            }
        ],
    )

    # Quan acaba HOME → llança PICK
    home_to_pick = RegisterEventHandler(
        OnProcessExit(
            target_action=home_node,
            on_exit=[
                TimerAction(period=0.5, actions=[pick_node]),
            ],
        )
    )

    return LaunchDescription([
        home_node,
        home_to_pick,
    ])
