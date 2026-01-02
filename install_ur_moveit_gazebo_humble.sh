#!/usr/bin/env bash
set -euo pipefail

###############################################################################
# Usage
###############################################################################
usage() {
  cat <<'EOF'
Usage:
  ./install_ur_moveit_gazebo_humble.sh --ws /path/to/workspace [--pin-moveit <version>]

Examples:
  ./install_ur_moveit_gazebo_humble.sh --ws ~/ROS2_UR_manipulation_ws
  ./install_ur_moveit_gazebo_humble.sh --ws ~/ROS2_UR_manipulation_ws \
      --pin-moveit 2.5.9-1jammy.20251119.004651
EOF
}

###############################################################################
# Parse arguments
###############################################################################
WS=""
PIN_MOVEIT_VERSION=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws) WS="${2:-}"; shift 2 ;;
    --pin-moveit) PIN_MOVEIT_VERSION="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      if [[ -z "${WS}" ]]; then WS="$1"; shift
      else echo "Unknown argument: $1"; usage; exit 1
      fi
      ;;
  esac
done

if [[ -z "${WS}" ]]; then
  echo "ERROR: workspace path not provided."
  usage
  exit 1
fi

WS="${WS/#\~/${HOME}}"

ROS_DISTRO="humble"
UBUNTU_CODENAME="jammy"

ROS_KEYRING="/etc/apt/keyrings/ros2-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Pin MoveIt version: ${PIN_MOVEIT_VERSION:-<none>}"

###############################################################################
# 0) HARD FIX: clean ROS2 APT sources BEFORE any apt command
###############################################################################
echo "Repairing APT ROS 2 sources (Signed-By conflicts)..."

sudo mkdir -p /etc/apt/keyrings

sudo bash -c '
set -e
# Remove any ROS2 repo references from .list
for f in /etc/apt/sources.list.d/*.list; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    rm -f "$f"
  fi
done

# Remove any ROS2 repo references from .sources
for f in /etc/apt/sources.list.d/*.sources; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    rm -f "$f"
  fi
done

# Remove ROS2 lines from main sources.list (rare but possible)
if [ -f /etc/apt/sources.list ]; then
  sed -i "\|packages.ros.org/ros2/ubuntu|d" /etc/apt/sources.list
fi
'

###############################################################################
# 1) Install minimal tools + add ROS2 repo cleanly
###############################################################################
sudo apt-get update -y
sudo apt-get install -y curl gnupg ca-certificates

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo gpg --dearmor -o "${ROS_KEYRING}"

echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
  sudo tee "${ROS_LIST}" > /dev/null

###############################################################################
# 2) Update + FULL upgrade (prevents partial MoveIt installs)
###############################################################################
sudo apt-get update -y
sudo apt-get full-upgrade -y

###############################################################################
# 3) Install ROS tooling + UR + Gazebo + MoveIt
###############################################################################
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  python3-setuptools \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-${ROS_DISTRO}-ur \
  ros-${ROS_DISTRO}-ur-description \
  ros-${ROS_DISTRO}-ur-msgs \
  ros-${ROS_DISTRO}-ur-robot-driver \
  ros-${ROS_DISTRO}-ur-bringup \
  ros-${ROS_DISTRO}-ur-moveit-config \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-ros \
  ros-${ROS_DISTRO}-moveit-planners-ompl \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-controller-interface \
  ros-${ROS_DISTRO}-realtime-tools \
  gazebo \
  ros-${ROS_DISTRO}-gazebo-ros \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-gazebo-ros2-control

###############################################################################
# 4) Ensure MoveIt version coherence (pin optional)
###############################################################################
MOVEIT_PKGS=(
  ros-${ROS_DISTRO}-moveit-core
  ros-${ROS_DISTRO}-moveit-common
  ros-${ROS_DISTRO}-moveit-ros-move-group
  ros-${ROS_DISTRO}-moveit-ros-planning-interface
  ros-${ROS_DISTRO}-moveit-ros-planning
  ros-${ROS_DISTRO}-moveit-planners-ompl
  ros-${ROS_DISTRO}-moveit-ros
)

if [[ -n "${PIN_MOVEIT_VERSION}" ]]; then
  sudo apt-get install -y \
    ros-${ROS_DISTRO}-moveit-core=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-common=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-ros-move-group=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-ros-planning=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-planners-ompl=${PIN_MOVEIT_VERSION} \
    ros-${ROS_DISTRO}-moveit-ros=${PIN_MOVEIT_VERSION}

  sudo apt-mark hold "${MOVEIT_PKGS[@]}"
else
  sudo apt-get install -y --only-upgrade "${MOVEIT_PKGS[@]}"
  sudo apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-planners-ompl
fi

sudo ldconfig

###############################################################################
# 5) rosdep for workspace
###############################################################################
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

if [ -d "${WS}/src" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  rosdep install --from-paths "${WS}/src" --ignore-src -y
fi

###############################################################################
# 6) Build workspace (development mode)
###############################################################################
if [ -d "${WS}" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  rm -rf "${WS}/build" "${WS}/install" "${WS}/log"
  colcon build --symlink-install --base-paths "${WS}"
fi

###############################################################################
# 7) Post-check
###############################################################################
echo "MoveIt versions:"
dpkg -l | grep -E "ros-${ROS_DISTRO}-moveit-(core|common|ros-move-group|ros-planning-interface|planners-ompl)" || true

echo "DONE."
echo "Open a NEW terminal and run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
