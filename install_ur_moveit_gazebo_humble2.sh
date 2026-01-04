#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# install_ur_moveit_gazebo_humble_TheConstruct.sh
#
# Based on your previous script (robust apt key fix + TheConstruct behavior) :contentReference[oaicite:1]{index=1}
# Enhancements:
#   - Avoid global full-upgrade by default (TheConstruct disk constraint)
#   - Force MoveIt2 packages to be version-consistent (fix OMPL plugin missing .so)
#   - Simple env cleanup to avoid AMENT_PREFIX_PATH warnings:
#       unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
#
# Works on:
#   - TheConstruct (ephemeral, tight disk)
#   - Ubuntu 22.04 PC (optionally with --do-full-upgrade)
# ============================================================

# ----------------------------
# Defaults
# ----------------------------
ROS_DISTRO="humble"
WS=""
FIX_ROS_REPO="false"
PIN_MOVEIT_MINOR=""         # Optional, e.g. "2.5.9" (best-effort pin)
DO_FULL_UPGRADE="false"     # Recommended only on a real Ubuntu PC with space

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
    --do-full-upgrade)
      DO_FULL_UPGRADE="true"; shift 1;;
    -h|--help)
      echo "Usage:"
      echo "  $0 --ws <workspace_path> [--rosdistro humble] [--fix-ros-repo] [--pin-moveit-minor 2.5.9] [--do-full-upgrade]"
      echo ""
      echo "Examples:"
      echo "  TheConstruct (recommended):"
      echo "    $0 --ws ~/ROS2_UR_manipulation_ws --fix-ros-repo"
      echo ""
      echo "  Ubuntu PC (recommended if you have disk space):"
      echo "    $0 --ws ~/ROS2_UR_manipulation_ws --do-full-upgrade"
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
echo "Do full upgrade: ${DO_FULL_UPGRADE}"

# ----------------------------
# Helpers
# ----------------------------

is_theconstruct() {
  [[ -n "${THECONSTRUCT:-}" ]] || [[ -n "${ROSJECT_NAME:-}" ]] || [[ -d "/home/simulations" ]]
}

safe_source_ros() {
  local setup_file="$1"

  if [[ ! -f "${setup_file}" ]]; then
    echo "ERROR: ${setup_file} not found. Is ROS 2 installed?"
    return 1
  fi

  # TheConstruct sometimes breaks with nounset due to setup scripts reading unset vars
  set +u
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  export AMENT_TRACE_SETUP_FILES_OVERRIDE="${AMENT_TRACE_SETUP_FILES_OVERRIDE:-}"
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

# ----------------------------
# ROS APT repo/key fix helpers (kept from your previous script) :contentReference[oaicite:2]{index=2}
# ----------------------------
ROS_APT_URL="http://packages.ros.org/ros2/ubuntu"
ROS_APT_DIST="jammy"
ROS_KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
ROS_LEGACY_FPR="F42ED6FBAB17C654"

fix_ros_repo() {
  echo "[fix-ros-repo] Resetting ROS 2 apt repo + keyring..."

  sudo mkdir -p /etc/apt/keyrings

  sudo rm -f /etc/apt/sources.list.d/ros2.list \
             /etc/apt/sources.list.d/ros2-latest.list \
             /etc/apt/sources.list.d/ros2.sources \
             /etc/apt/sources.list.d/ros2-*.list || true

  sudo rm -f /etc/apt/keyrings/ros2-archive-keyring.gpg \
             /etc/apt/keyrings/ros-archive-keyring.gpg \
             /etc/apt/trusted.gpg.d/ros2-archive-keyring.gpg || true

  sudo apt-key del "${ROS_LEGACY_FPR}" >/dev/null 2>&1 || true

  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor \
    | sudo tee "${ROS_KEYRING}" > /dev/null

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
    echo "$out" | tail -n 60

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
# MoveIt consistency helpers (new)
# ----------------------------
moveit_pkg_list() {
  cat <<EOF
ros-${ROS_DISTRO}-moveit-core
ros-${ROS_DISTRO}-moveit-ros
ros-${ROS_DISTRO}-moveit-ros-move-group
ros-${ROS_DISTRO}-moveit-ros-planning
ros-${ROS_DISTRO}-moveit-ros-planning-interface
ros-${ROS_DISTRO}-moveit-plugins
ros-${ROS_DISTRO}-moveit-planners-ompl
EOF
}

force_moveit_candidate_versions() {
  echo "[moveit] Installing/aligning MoveIt packages to APT candidate versions (targeted, no full-upgrade)..."

  # Critical: do NOT swallow errors here. We want a hard fail if alignment is impossible.
  sudo apt-get install -y --no-install-recommends $(moveit_pkg_list)

  # Fix any partial dependency state (still targeted)
  sudo apt-get -f install -y

  echo "[moveit] Verifying OMPL plugin dependencies..."
  if ldd /opt/ros/${ROS_DISTRO}/lib/libmoveit_ompl_planner_plugin.so | grep -q "not found"; then
    echo "ERROR: MoveIt OMPL plugin still has missing shared libraries:"
    ldd /opt/ros/${ROS_DISTRO}/lib/libmoveit_ompl_planner_plugin.so | grep "not found" || true
    echo ""
    echo "Installed MoveIt package versions:"
    dpkg -l | grep -E "ros-${ROS_DISTRO}-moveit" | sed -n '1,160p' || true
    exit 1
  fi

  echo "[moveit] OK: OMPL plugin has no missing libs."
}

pin_moveit_minor_best_effort() {
  local pin="$1"
  echo "[moveit] Pinning MoveIt packages to version ${pin}* (best-effort)..."

  # Best-effort pin; some repos may not provide all exact versions.
  # We keep it strict enough to help, but if it fails, we fall back to candidate alignment.
  set +e
  sudo apt-get install -y --no-install-recommends \
    ros-"${ROS_DISTRO}"-moveit-core="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-ros="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-ros-move-group="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-ros-planning="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-ros-planning-interface="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-plugins="${pin}"* \
    ros-"${ROS_DISTRO}"-moveit-planners-ompl="${pin}"*
  rc=$?
  set -e

  if [[ $rc -ne 0 ]]; then
    echo "[moveit] Pin failed or incomplete. Falling back to candidate alignment."
    force_moveit_candidate_versions
  else
    sudo apt-get -f install -y
  fi
}

# ----------------------------
# Optional explicit repo fix
# ----------------------------
if [[ "${FIX_ROS_REPO}" == "true" ]]; then
  echo ""
  echo "[0/8] Forcing ROS repo/key fix (requested)..."
  fix_ros_repo
fi

# ============================================================
# [1/8] APT health
# ============================================================
echo ""
echo "[1/8] APT health: update + fix-broken..."
apt_update_with_auto_fix
sudo apt --fix-broken install -y || true

# Optional full-upgrade (avoid on TheConstruct)
if [[ "${DO_FULL_UPGRADE}" == "true" ]]; then
  echo ""
  echo "[1.1/8] Full upgrade requested..."
  sudo apt-get upgrade -y
  sudo apt-get -f install -y
fi

# ============================================================
# [2/8] Base tooling (APT)
# ============================================================
echo ""
echo "[2/8] Installing base tooling..."
sudo apt-get install -y --no-install-recommends \
  python3-pip python3-setuptools python3-venv \
  curl gnupg lsb-release ca-certificates \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-vcstool || true

# ============================================================
# [3/8] Install ROS packages (UR + Gazebo Classic + ros2_control)
# ============================================================
echo ""
echo "[3/8] Installing ROS packages (UR + Gazebo Classic + ros2_control)..."

sudo apt-get install -y --no-install-recommends \
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
  ros-"${ROS_DISTRO}"-xacro \
  ros-"${ROS_DISTRO}"-joint-state-publisher \
  ros-"${ROS_DISTRO}"-joint-state-publisher-gui \
  ros-"${ROS_DISTRO}"-robot-state-publisher \
  ros-"${ROS_DISTRO}"-tf2-ros \
  ros-"${ROS_DISTRO}"-tf-transformations \
  ros-"${ROS_DISTRO}"-rviz2

# ============================================================
# [4/8] MoveIt2 install + consistency fix (critical)
# ============================================================
echo ""
echo "[4/8] Installing MoveIt2 (targeted) and enforcing version consistency..."

# Important: do NOT install only meta packages and ignore failures.
# Either pin to a minor (best-effort) or align to candidate versions.
if [[ -n "${PIN_MOVEIT_MINOR}" ]]; then
  pin_moveit_minor_best_effort "${PIN_MOVEIT_MINOR}"
else
  force_moveit_candidate_versions
fi

# ============================================================
# [5/8] ROS environment + rosdep
# ============================================================
echo ""
echo "[5/8] rosdep install for workspace (ignore-src)..."

safe_source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -d "${WS}" ]]; then
  echo "ERROR: Workspace folder does not exist: ${WS}"
  exit 1
fi

sudo rosdep init >/dev/null 2>&1 || true
rosdep update

if [[ -d "${WS}/src" ]]; then
  rosdep install --from-paths "${WS}/src" --ignore-src -y || true
else
  echo "WARNING: ${WS}/src not found. Skipping rosdep install."
fi

# ============================================================
# [6/8] Build workspace (and fix AMENT warnings)
# ============================================================
echo ""
echo "[6/8] Building workspace..."

cd "${WS}"
rm -rf build install log || true

# Simple env cleanup (your preferred approach)
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

# Re-source underlay (keeps environment clean for colcon)
safe_source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"

colcon build --symlink-install

# ============================================================
# [7/8] Python user-space deps (safe on TheConstruct)
# ============================================================
echo ""
echo "[7/8] Installing Python tools in user-space (pip --user)..."

#python3 -m pip install --user --upgrade pip
#python3 -m pip install --user spatialmath-python

# ============================================================
# [8/8] Final verification
# ============================================================
echo ""
echo "[8/8] Final verification checks..."

echo ""
echo "MoveIt planning interface libraries:"
ls -l /opt/ros/${ROS_DISTRO}/lib/libmoveit_planning_interface.so* || true

echo ""
echo "OMPL plugin dependency check:"
ldd /opt/ros/${ROS_DISTRO}/lib/libmoveit_ompl_planner_plugin.so | grep "not found" || echo "OK: no missing libs"

echo ""
echo "Done."
echo "Next:"
echo "  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
echo ""
echo "If your combo launch still fails, immediately check move_group output for:"
echo "  - Failed to load any planning pipelines"
echo "  - dlopen / not found errors"
