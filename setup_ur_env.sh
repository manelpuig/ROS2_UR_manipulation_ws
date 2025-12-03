#!/usr/bin/env bash
set -e

echo "🔧 Setting up ROS2 + UR + Gazebo environment..."

# =======================================================
# 1. Fix / ensure ROS 2 APT repository (safe on Ubuntu22)
# =======================================================
echo "🔑 Fixing ROS 2 repository key..."
sudo mkdir -p /etc/apt/keyrings

# Remove old or broken keys (no problem if they don't exist)
sudo rm -f /etc/apt/trusted.gpg.d/ros2-archive-keyring.gpg
sudo rm -f /etc/apt/keyrings/ros2-archive-keyring.gpg
sudo rm /etc/apt/sources.list.d/ros2.sources

# Install fresh key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  gpg --dearmor | sudo tee /etc/apt/keyrings/ros2-archive-keyring.gpg > /dev/null

# Ensure ROS2 repo is present (Jammy + Humble)
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null


# =======================================================
# 2. Install ROS2 + UR + Gazebo + MoveIt dependencies
# =======================================================
echo "🔧 Installing ROS2 simulation & MoveIt dependencies..."
sudo apt update -y || true

sudo apt install -y \
  ros-humble-ur \
  ros-humble-ur-description \
  ros-humble-ur-msgs \
  ros-humble-ur-robot-driver \
  ros-humble-ur-bringup \
  ros-humble-ur-moveit-config \
  ros-humble-moveit \
  ros-humble-moveit-ros \
  ros-humble-moveit-planners-ompl \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-controller-interface \
  ros-humble-realtime-tools \
  gazebo \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control || true


# =======================================================
# 3. Install dependencies for UR repositories you cloned
# =======================================================
echo "🔧 Installing rosdep dependencies..."
cd ~/Desktop/ROS2_UR_manipulation_ws

# IMPORTANT: run once in your life (outside this script) on a fresh Ubuntu:
#   sudo rosdep init
# Després ja pots fer:
rosdep update
rosdep install --from-paths src --ignore-src -y || true

# =======================================================
# 4. Build the workspace
# =======================================================
echo "🔧 Building workspace..."
colcon build --symlink-install

echo "✅ UR environment ready!"
