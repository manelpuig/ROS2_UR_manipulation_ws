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
    # X11 graphical support
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    mesa-utils \
    libqt5x11extras5 \
    libxkbcommon-x11-0 \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    \
    # Dev tools
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    unzip \
    git \
    iputils-ping \
    \
    # ROS 2 add-ons
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    ros-humble-cartographer-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-server \
    \
    # Gazebo Classic (Gazebo 11)
    gazebo \
    libgazebo-dev \
    \
    # MoveIt 2
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo \
    \
    # Universal Robots
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-ur-robot-driver \
    \
    # ros2_control core
    ros-humble-ros2-control \
    ros-humble-ros2-controllers || true


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
colcon build

echo "✅ UR environment ready!"
