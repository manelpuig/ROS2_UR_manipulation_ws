#!/usr/bin/env bash

echo "=============================================="
echo " Killing UR / MoveIt / Gazebo ROS 2 processes "
echo "=============================================="

# Stop ros2 launch processes
pkill -f "ros2 launch" && echo "✓ ros2 launch killed" || echo "- no ros2 launch running"

# MoveIt
pkill -f move_group && echo "✓ move_group killed" || echo "- no move_group running"

# RViz
pkill -f rviz2 && echo "✓ rviz2 killed" || echo "- no rviz2 running"

# Gazebo
pkill -f gzserver && echo "✓ gzserver killed" || echo "- no gzserver running"
pkill -f gzclient && echo "✓ gzclient killed" || echo "- no gzclient running"
pkill -f gazebo  && echo "✓ gazebo killed"  || echo "- no gazebo running"

# Controllers / ros2 control leftovers (safe)
pkill -f controller_manager && echo "✓ controller_manager killed" || true
pkill -f spawner && echo "✓ controller spawners killed" || true

# Small wait to ensure processes die
sleep 1

# Restart ros2 daemon to clear graph cache
echo "Restarting ros2 daemon..."
ros2 daemon stop
sleep 1
ros2 daemon start

echo "----------------------------------------------"
echo " Remaining critical ROS processes (should be none):"
ps aux | egrep "move_group|gzserver|gzclient|rviz2|ros2 launch" | grep -v egrep || echo "✓ CLEAN"
echo "----------------------------------------------"
echo " System ready to launch UR sim + MoveIt "
echo "=============================================="
