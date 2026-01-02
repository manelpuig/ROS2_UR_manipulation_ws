#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Defaults
# -----------------------------
WS=""
DISTRO="humble"
PIN_MOVEIT_VERSION=""   # e.g. "2.5.9-1jammy.20251119.004651" (optional)

usage() {
  cat <<EOF
Usage:
  $0 --ws <workspace_path> [--distro humble] [--pin-moveit <apt_version_string>]

Examples:
  $0 --ws ~/ROS2_UR_manipulation_ws
  $0 --ws ~/ROS2_UR_manipulation_ws --pin-moveit "2.5.9-1jammy.20251119.004651"
EOF
}

# -----------------------------
# Parse args
# -----------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws) WS="$2"; shift 2;;
    --distro) DISTRO="$2"; shift 2;;
    --pin-moveit) PIN_MOVEIT_VERSION="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown argument: $1"; usage; exit 1;;
  esac
done

if [[ -z "${WS}" ]]; then
  echo "ERROR: --ws <workspace_path> is required."
  exit 1
fi

WS="$(realpath -m "${WS}")"

echo "Workspace: ${WS}"
echo "ROS distro: ${DISTRO}"
echo "Pin MoveIt version: ${PIN_MOVEIT_VERSION:-<none>}"

# -----------------------------
# 1) Ensure ROS 2 apt repo is clean (NO signed-by conflicts)
#    We standardize on: /etc/apt/keyrings/ros-archive-keyring.gpg
# -----------------------------
echo ""
echo "[1/7] Fixing ROS 2 apt repository (remove duplicates + ensure single signed-by)..."

sudo mkdir -p /etc/apt/keyrings

# Remove common conflicting ROS list files that cause signed-by mismatches
sudo rm -f /etc/apt/sources.list.d/ros2*.list
sudo rm -f /etc/apt/sources.list.d/ros2*.sources
sudo rm -f /etc/apt/sources.list.d/ros-latest.list
sudo rm -f /etc/apt/sources.list.d/ros2-latest.list

# Create/refresh the canonical ROS keyring
# (We do NOT use ros2-archive-keyring.gpg to avoid mismatch with other machines/scripts)
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  gpg --dearmor | sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

# Add a single ROS2 repo entry
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update

# -----------------------------
# 2) Base python tooling (avoid pip setuptools conflicts)
# -----------------------------
echo ""
echo "[2/7] Installing base Python tooling from APT (recommended for ROS builds)..."
sudo apt-get install -y python3-pip python3-setuptools python3-venv

# Strong recommendation: avoid /usr/local pip shadowing (common cause of weird setuptools behavior)
# Not failing if present, just warning.
if python3 -c "import setuptools,sys; print(setuptools.__file__)" 2>/dev/null | grep -q "/usr/local/"; then
  echo "WARNING: setuptools is being imported from /usr/local (pip). This can cause ROS build warnings."
  echo "         Prefer Ubuntu packages (python3-setuptools) and avoid sudo pip installs."
fi

# -----------------------------
# 3) Install UR + MoveIt + Gazebo deps
# -----------------------------
echo ""
echo "[3/7] Installing UR + MoveIt + Gazebo packages..."
sudo apt-get install -y \
  ros-${DISTRO}-ur \
  ros-${DISTRO}-ur-description \
  ros-${DISTRO}-ur-msgs \
  ros-${DISTRO}-ur-robot-driver \
  ros-${DISTRO}-ur-bringup \
  ros-${DISTRO}-ur-moveit-config \
  ros-${DISTRO}-moveit \
  ros-${DISTRO}-moveit-ros \
  ros-${DISTRO}-moveit-planners-ompl \
  ros-${DISTRO}-ros2-control \
  ros-${DISTRO}-ros2-controllers \
  ros-${DISTRO}-controller-manager \
  ros-${DISTRO}-controller-interface \
  ros-${DISTRO}-realtime-tools \
  gazebo \
  ros-${DISTRO}-gazebo-ros \
  ros-${DISTRO}-gazebo-ros-pkgs \
  ros-${DISTRO}-gazebo-ros2-control

# -----------------------------
# 4) Optional: pin MoveIt version
#    WARNING: pinning is for reproducibility, but can create dependency conflicts later.
# -----------------------------
if [[ -n "${PIN_MOVEIT_VERSION}" ]]; then
  echo ""
  echo "[4/7] Pinning MoveIt packages to: ${PIN_MOVEIT_VERSION}"
  sudo apt-get install -y \
    ros-${DISTRO}-moveit-core="${PIN_MOVEIT_VERSION}" \
    ros-${DISTRO}-moveit-ros-move-group="${PIN_MOVEIT_VERSION}" || {
      echo "ERROR: Could not install requested pinned MoveIt version."
      echo "       Either the version string is not available on this machine, or dependencies differ."
      exit 1
    }

  # Hold them so future upgrades don't change version
  sudo apt-mark hold \
    ros-${DISTRO}-moveit-core \
    ros-${DISTRO}-moveit-ros-move-group
else
  echo ""
  echo "[4/7] MoveIt pinning skipped (recommended default)."
fi

# -----------------------------
# 5) rosdep
# -----------------------------
echo ""
echo "[5/7] Installing rosdep dependencies from your workspace..."
source /opt/ros/${DISTRO}/setup.bash

# rosdep init may already be done; don't fail if it is
if ! sudo test -f /etc/ros/rosdep/sources.list.d/20-default.list; then
  sudo rosdep init
fi
rosdep update

if [[ -d "${WS}/src" ]]; then
  # Skip ament_python if your packages include it and rosdep complains on some machines
  rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${DISTRO}" --skip-keys "ament_python" || true
else
  echo "WARNING: ${WS}/src not found. Skipping rosdep install."
fi

# -----------------------------
# 6) Build workspace
# -----------------------------
echo ""
echo "[6/7] Building workspace..."
cd "${WS}"
rm -rf build install log

colcon build --symlink-install

# -----------------------------
# 7) Final hints
# -----------------------------
echo ""
echo "[7/7] Done."
echo "Next:"
echo "  source /opt/ros/${DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
