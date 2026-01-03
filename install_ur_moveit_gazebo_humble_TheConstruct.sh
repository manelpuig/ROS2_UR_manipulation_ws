#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# install_ur_moveit_gazebo_humble.sh
#
# هدف:
# - Fix ROS 2 apt repo/key automatically when EXPKEYSIG happens
# - Install UR + MoveIt2 + Gazebo Classic deps (Humble/Jammy)
# - rosdep install workspace deps
# - Build workspace
# - Install spatialmath-python in user space (pip --user)
#
# Works well on:
# - TheConstruct (often ephemeral rootfs)
# - Regular Ubuntu 22.04 PCs
# ============================================================

# ----------------------------
# Defaults
# ----------------------------
ROS_DISTRO="humble"
WS=""
FIX_ROS_REPO="false"
PIN_MOVEIT_MINOR=""   # example: "2.5.9" (optional)

# ----------------------------
# Args
# ----------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)
      WS="${2:-}"; shift 2;;
    --rosdistro)
      ROS_DISTRO="${2:-humble}"; shift 2;;
    --fix-ros-repo)
      FIX_ROS_REPO="true"; shift 1;;
    --pin-moveit-minor)
      PIN_MOVEIT_MINOR="${2:-}"; shift 2;;
    -h|--help)
      echo "Usage:"
      echo "  $0 --ws <workspace_path> [--rosdistro humble] [--fix-ros-repo] [--pin-moveit-minor 2.5.9]"
      exit 0;;
    *)
      echo "Unknown argument: $1"
      exit 1;;
  esac
done

if [[ -z "${WS}" ]]; then
  echo "ERROR: --ws <workspace_path> is required"
  exit 1
fi

WS="$(realpath -m "${WS}")"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Fix ROS repo: ${FIX_ROS_REPO}"
echo "Pin MoveIt minor: ${PIN_MOVEIT_MINOR:-<none>}"

# ----------------------------
# ROS APT repo/key fix helpers
# ----------------------------
ROS_APT_URL="http://packages.ros.org/ros2/ubuntu"
ROS_APT_DIST="jammy"
ROS_KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
ROS_LEGACY_FPR="F42ED6FBAB17C654"   # commonly seen expired key in older images

fix_ros_repo() {
  echo "[fix-ros-repo] Resetting ROS 2 apt repo + keyring..."

  sudo mkdir -p /etc/apt/keyrings

  # Remove duplicate / old source definitions
  sudo rm -f /etc/apt/sources.list.d/ros2.list \
             /etc/apt/sources.list.d/ros2-latest.list \
             /etc/apt/sources.list.d/ros2.sources \
             /etc/apt/sources.list.d/ros2-*.list || true

  # Remove old/broken keyrings (both common names)
  sudo rm -f /etc/apt/keyrings/ros2-archive-keyring.gpg \
             /etc/apt/keyrings/ros-archive-keyring.gpg \
             /etc/apt/trusted.gpg.d/ros2-archive-keyring.gpg || true

  # Remove legacy key from trusted.gpg if present (deprecated, but used for cleanup)
  sudo apt-key del "${ROS_LEGACY_FPR}" >/dev/null 2>&1 || true

  # Install fresh keyring (modern signed-by workflow)
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor \
    | sudo tee "${ROS_KEYRING}" > /dev/null

  # Single, canonical repo entry
  echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] ${ROS_APT_URL} ${ROS_APT_DIST} main" \
    | sudo tee "${ROS_LIST}" > /dev/null

  echo "[fix-ros-repo] Done."
}

apt_update_with_auto_fix() {
  echo "[apt] Running apt-get update..."
  set +e
  out="$(sudo apt-get update 2>&1)"
  rc=$?
  set -e

  if [[ $rc -ne 0 ]]; then
    echo "$out" | tail -n 40

    if echo "$out" | grep -Eq "EXPKEYSIG|NO_PUBKEY|not signed|The following signatures were invalid"; then
      echo "[apt] Detected ROS repo signature/key error. Applying fix..."
      fix_ros_repo
      sudo apt-get update
    else
      echo "[apt] apt-get update failed (not a ROS key issue)."
      return $rc
    fi
  fi
}

# ----------------------------
# Optional explicit repo fix
# ----------------------------
if [[ "${FIX_ROS_REPO}" == "true" ]]; then
  echo ""
  echo "[0/7] Forcing ROS repo/key fix (requested)..."
  fix_ros_repo
fi

# ============================================================
# [1/7] APT health
# ============================================================
echo ""
echo "[1/7] APT health: update + fix-broken..."
apt_update_with_auto_fix

# On ephemeral systems, dpkg can be mid-state. This makes the script recover.
sudo apt --fix-broken install -y || true

# Note: full-upgrade can be heavy on TheConstruct; keep it off by default.
# If you really want it, uncomment the next line:
# sudo apt-get -y full-upgrade || true

# ============================================================
# [2/7] Base Python tooling (APT)
# ============================================================
echo ""
echo "[2/7] Installing base Python tooling from APT (recommended for ROS builds)..."
sudo apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-venv \
  python3.10-venv \
  curl \
  gnupg || true

# ============================================================
# [3/7] Install ROS packages (UR + MoveIt + Gazebo Classic stack)
# ============================================================
echo ""
echo "[3/7] Installing ROS packages (UR + MoveIt + Gazebo Classic)..."

# We DO NOT remove gz/ign packages (you asked to keep ros-humble-desktop-full intact).
# We also target Gazebo Classic (gazebo11 + gazebo_ros_pkgs + gazebo_ros2_control).
sudo apt-get install -y \
  gazebo \
  ros-"${ROS_DISTRO}"-gazebo-ros-pkgs \
  ros-"${ROS_DISTRO}"-gazebo-ros2-control \
  ros-"${ROS_DISTRO}"-ros2-control \
  ros-"${ROS_DISTRO}"-ros2-controllers \
  ros-"${ROS_DISTRO}"-controller-manager \
  ros-"${ROS_DISTRO}"-realtime-tools \
  ros-"${ROS_DISTRO}"-ur \
  ros-"${ROS_DISTRO}"-ur-description \
  ros-"${ROS_DISTRO}"-ur-msgs \
  ros-"${ROS_DISTRO}"-ur-robot-driver \
  ros-"${ROS_DISTRO}"-ur-bringup \
  ros-"${ROS_DISTRO}"-ur-moveit-config \
  ros-"${ROS_DISTRO}"-moveit \
  ros-"${ROS_DISTRO}"-moveit-ros \
  ros-"${ROS_DISTRO}"-moveit-planners-ompl || true

# Optional: pin MoveIt minor (only if you really need strict reproducibility).
# This is best-effort; if the exact version isn't available in the repo, apt will fail.
if [[ -n "${PIN_MOVEIT_MINOR}" ]]; then
  echo ""
  echo "[3.1/7] Pinning MoveIt packages to version ${PIN_MOVEIT_MINOR}* (best-effort)..."
  sudo apt-get install -y \
    ros-"${ROS_DISTRO}"-moveit-core="${PIN_MOVEIT_MINOR}"* \
    ros-"${ROS_DISTRO}"-moveit-ros-move-group="${PIN_MOVEIT_MINOR}"* \
    ros-"${ROS_DISTRO}"-moveit-planners-ompl="${PIN_MOVEIT_MINOR}"* || true
fi

# ============================================================
# [4/7] ROS environment + rosdep
# ============================================================
echo ""
echo "[4/7] rosdep install for workspace (ignore-src)..."

# Source ROS (required for rosdep to resolve ROS keys reliably)
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found. Is ROS 2 ${ROS_DISTRO} installed?"
  exit 1
fi

# Ensure workspace exists
if [[ ! -d "${WS}" ]]; then
  echo "ERROR: Workspace folder does not exist: ${WS}"
  exit 1
fi

# rosdep init is "once per machine"; in TheConstruct it may already be done.
sudo rosdep init >/dev/null 2>&1 || true
rosdep update

if [[ -d "${WS}/src" ]]; then
  rosdep install --from-paths "${WS}/src" --ignore-src -y || true
else
  echo "WARNING: ${WS}/src not found. Skipping rosdep install."
fi

# ============================================================
# [5/7] Build workspace
# ============================================================
echo ""
echo "[5/7] Building workspace..."
cd "${WS}"
rm -rf build install log || true
colcon build --symlink-install

# ============================================================
# [6/7] Python user-space deps (safe on TheConstruct)
# ============================================================
echo ""
echo "[6/7] Installing Python tools in user-space (pip --user)..."

# Keep pip installs in ~/.local to avoid breaking system Python / ROS stacks
python3 -m pip install --user --upgrade pip

# Your requested extra dependency:
python3 -m pip install --user spatialmath-python

# ============================================================
# [7/7] Summary
# ============================================================
echo ""
echo "[7/7] Done."
echo "Next:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
echo ""
echo "Notes:"
echo " - If TheConstruct resets the root filesystem, re-run this script."
echo " - Python deps are installed in ~/.local (pip --user) to avoid NumPy/system conflicts."
