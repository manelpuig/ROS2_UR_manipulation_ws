#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./install_ur_moveit_gazebo_classic_humble.sh --ws /path/to/workspace [--pin-moveit <version>]

Examples:
  ./install_ur_moveit_gazebo_classic_humble.sh --ws ~/ROS2_UR_manipulation_ws
  ./install_ur_moveit_gazebo_classic_humble.sh --ws ~/ROS2_UR_manipulation_ws \
      --pin-moveit 2.5.9-1jammy.20251119.004651
EOF
}

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
  echo "ERROR: --ws is required."
  usage
  exit 1
fi

WS="${WS/#\~/${HOME}}"

ROS_DISTRO="humble"
UBUNTU_CODENAME="jammy"

# Canonical ROS2 repo keyring + list
ROS_KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Pin MoveIt version: ${PIN_MOVEIT_VERSION:-<none>}"
echo "Simulator target: Gazebo Classic (gazebo11)"

###############################################################################
# 0) HARD FIX: clean ROS2 APT sources BEFORE any apt command
###############################################################################
echo "[0/8] Repairing APT ROS 2 sources (Signed-By conflicts)..."

sudo mkdir -p /etc/apt/keyrings

sudo bash -c '
set -e
# Remove ROS2 repo references from .list
for f in /etc/apt/sources.list.d/*.list; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    rm -f "$f"
  fi
done
# Remove ROS2 repo references from .sources
for f in /etc/apt/sources.list.d/*.sources; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    rm -f "$f"
  fi
done
# Remove ROS2 lines from /etc/apt/sources.list (rare)
if [ -f /etc/apt/sources.list ]; then
  sed -i "\|packages.ros.org/ros2/ubuntu|d" /etc/apt/sources.list
fi
'

###############################################################################
# 1) Add ROS2 repo cleanly
###############################################################################
echo "[1/8] Adding ROS 2 apt repository (single signed-by)..."

sudo apt-get update -y
sudo apt-get install -y curl gnupg ca-certificates

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  gpg --dearmor | sudo tee "${ROS_KEYRING}" > /dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
  sudo tee "${ROS_LIST}" > /dev/null

sudo apt-get update -y

###############################################################################
# 2) Remove Gazebo (gz/ign) stack to stay on Gazebo Classic only
###############################################################################
echo "[2/8] Removing Gazebo (gz/ign) packages (keeping Gazebo Classic)..."

# Remove ros_gz / gz / ign shims that you don't want for Classic
sudo apt-get remove -y \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-sim-demos \
  ros-humble-ros-ign-gazebo \
  ros-humble-ign-ros2-control \
  ros-humble-ign-ros2-control-demos \
  ros-humble-gz-ros2-control-demos \
  || true

# Clean up dependencies pulled by them
sudo apt-get autoremove -y

# Repair APT state if anything was half-installed
sudo apt-get --fix-broken install -y

###############################################################################
# 3) Update + FULL upgrade (prevents partial installs)
###############################################################################
echo "[3/8] Full system upgrade..."
sudo apt-get full-upgrade -y

###############################################################################
# 4) Install base Python tooling for ROS builds
###############################################################################
echo "[4/8] Installing base Python tooling..."
sudo apt-get install -y python3-pip python3-setuptools
# venv is optional; don't fail the whole setup if python venv packages are held
sudo apt-get install -y python3-venv || true

###############################################################################
# 5) Install ROS packages for UR + MoveIt + Gazebo Classic
###############################################################################
echo "[5/8] Installing UR + MoveIt + Gazebo Classic packages..."

sudo apt-get install -y \
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
# 6) Ensure MoveIt coherence (pin optional)
###############################################################################
echo "[6/8] Ensuring MoveIt version coherence..."

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
# 7) rosdep for workspace
###############################################################################
echo "[7/8] Running rosdep for workspace..."

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

if [ -d "${WS}/src" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${ROS_DISTRO}" || true
else
  echo "WARNING: ${WS}/src not found; skipping rosdep."
fi

###############################################################################
# 8) Build workspace (symlink for dev)
###############################################################################
echo "[8/8] Building workspace..."

if [ -d "${WS}" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  rm -rf "${WS}/build" "${WS}/install" "${WS}/log"
  cd "${WS}"
  colcon build --symlink-install
fi

echo "DONE. Open a new terminal and run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"

echo "Sanity check (should NOT show ros-gz / gz-ros2-control anymore):"
echo "  dpkg -l | grep -E 'ros-humble-(ros-gz|gz-ros2-control|ign-)' || echo OK"
