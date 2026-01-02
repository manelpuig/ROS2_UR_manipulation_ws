#!/usr/bin/env bash
set -euo pipefail

# -------------------------------------------------------
# Usage
# -------------------------------------------------------
usage() {
  cat <<'EOF'
Usage:
  ./install_ur_moveit_gazebo_humble.sh --ws /path/to/workspace [--pin-moveit <version>]

Examples:
  ./install_ur_moveit_gazebo_humble.sh --ws ~/ROS2_UR_manipulation_ws
  ./install_ur_moveit_gazebo_humble.sh --ws ~/ROS2_UR_manipulation_ws --pin-moveit 2.5.9-1jammy.20251119.004651

Notes:
  - --pin-moveit pins selected MoveIt packages to the exact apt version string.
  - If you pin, you assume that version exists in your ROS apt repository.
EOF
}

WS=""
PIN_MOVEIT_VERSION=""

# Allow both --ws and positional $1 for convenience
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)
      WS="${2:-}"
      shift 2
      ;;
    --pin-moveit)
      PIN_MOVEIT_VERSION="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      # If WS not set yet, accept first positional argument as WS
      if [[ -z "${WS}" ]]; then
        WS="$1"
        shift
      else
        echo "Unknown argument: $1"
        usage
        exit 1
      fi
      ;;
  esac
done

if [[ -z "${WS}" ]]; then
  echo "ERROR: workspace path not provided."
  usage
  exit 1
fi

# Expand ~
WS="${WS/#\~/${HOME}}"

ROS_DISTRO="humble"
UBUNTU_CODENAME="jammy"
ROS_KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Pin MoveIt version: ${PIN_MOVEIT_VERSION:-<none>}"

# -------------------------------------------------------
# 0) Basic prerequisites
# -------------------------------------------------------
sudo apt-get update -y
sudo apt-get install -y curl gnupg lsb-release ca-certificates

# -------------------------------------------------------
# 1) Ensure ROS 2 APT repo (idempotent)
# -------------------------------------------------------
ROS_KEYRING="/etc/apt/keyrings/ros2-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2-latest.list"

echo "Ensuring ROS 2 repository key and source list..."

sudo mkdir -p /etc/apt/keyrings

# Remove any ROS2 repo lists that reference the same repo but a different Signed-By keyring
# (prevents 'Conflicting values set for option Signed-By')
for f in /etc/apt/sources.list.d/*.list; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    sudo rm -f "$f"
  fi
done

# Install/refresh keyring (idempotent)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo gpg --dearmor -o "${ROS_KEYRING}"

# Create the single authoritative repo list
echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
  sudo tee "${ROS_LIST}" > /dev/null
# -------------------------------------------------------
# 2) Update + FULL upgrade to avoid partial upgrades
# -------------------------------------------------------
echo "Updating apt indexes and performing full upgrade..."
sudo apt-get update -y
sudo apt-get full-upgrade -y

# -------------------------------------------------------
# 3) Install ROS tooling + UR + Gazebo + MoveIt
# -------------------------------------------------------
echo "Installing ROS tooling + UR + Gazebo + MoveIt packages..."

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

# -------------------------------------------------------
# 4) Optional: pin MoveIt packages to an exact version
# -------------------------------------------------------
MOVEIT_PKGS=(
  "ros-${ROS_DISTRO}-moveit-core"
  "ros-${ROS_DISTRO}-moveit-common"
  "ros-${ROS_DISTRO}-moveit-ros-move-group"
  "ros-${ROS_DISTRO}-moveit-ros-planning-interface"
  "ros-${ROS_DISTRO}-moveit-ros-planning"
  "ros-${ROS_DISTRO}-moveit-planners-ompl"
  "ros-${ROS_DISTRO}-moveit-ros"
)

if [[ -n "${PIN_MOVEIT_VERSION}" ]]; then
  echo "Pinning MoveIt packages to version: ${PIN_MOVEIT_VERSION}"

  # Install the exact version explicitly (fails if unavailable)
  sudo apt-get install -y \
    "ros-${ROS_DISTRO}-moveit-core=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-common=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-ros-move-group=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-ros-planning-interface=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-ros-planning=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-planners-ompl=${PIN_MOVEIT_VERSION}" \
    "ros-${ROS_DISTRO}-moveit-ros=${PIN_MOVEIT_VERSION}"

  # Mark as held to prevent drifting on later upgrades
  sudo apt-mark hold "${MOVEIT_PKGS[@]}"

else
  echo "Forcing MoveIt packages to a consistent candidate version (recommended default)..."
  sudo apt-get install -y --only-upgrade "${MOVEIT_PKGS[@]}"

  # If anything is still mismatched, reinstall core ones (safe)
  sudo apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-planners-ompl
fi

sudo ldconfig

# -------------------------------------------------------
# 5) rosdep for workspace (ignore src packages)
# -------------------------------------------------------
echo "Installing rosdep dependencies for workspace..."

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "rosdep not initialized; initializing..."
  sudo rosdep init
fi
rosdep update

if [ -d "${WS}/src" ]; then
  pushd "${WS}" > /dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  rosdep install --from-paths src --ignore-src -y
  popd > /dev/null
else
  echo "Workspace src folder not found at: ${WS}/src"
  echo "Skipping rosdep install for workspace."
fi

# -------------------------------------------------------
# 6) Build workspace
# -------------------------------------------------------
echo "Building workspace..."
if [ -d "${WS}" ]; then
  pushd "${WS}" > /dev/null
  rm -rf build install log
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  colcon build --symlink-install
  popd > /dev/null
fi

# -------------------------------------------------------
# 7) Post-check: report MoveIt versions
# -------------------------------------------------------
echo "Post-check: MoveIt package versions (should match)."
dpkg -l | grep -E "ros-${ROS_DISTRO}-moveit-(core|common|ros-move-group|ros-planning-interface|planners-ompl)" || true

if [[ -n "${PIN_MOVEIT_VERSION}" ]]; then
  echo "MoveIt is pinned/held. To unhold later:"
  echo "  sudo apt-mark unhold ${MOVEIT_PKGS[*]}"
fi

echo "Done."
echo "Open a NEW terminal, then:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
