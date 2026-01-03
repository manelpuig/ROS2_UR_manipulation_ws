#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./install_ur_moveit_tc_humble.sh --ws /path/to/workspace [--rosdistro humble]
                                  [--fix-ros-repo]
                                  [--pin-moveit-minor 2.5.9]

Default behavior (TheConstruct-friendly):
  - Does NOT touch ROS apt keys/sources unless --fix-ros-repo is used.
  - Repairs apt state (fix-broken) + full-upgrade.
  - Installs UR + MoveIt + ros2_control + Gazebo Classic integration packages.
  - Makes MoveIt consistent via --reinstall (no timestamp pinning).
  - rosdep best-effort + colcon build --symlink-install.

Examples:
  ./install_ur_moveit_tc_humble.sh --ws ~/ROS2_UR_manipulation_ws
  ./install_ur_moveit_tc_humble.sh --ws ~/ROS2_UR_manipulation_ws --pin-moveit-minor 2.5.9
  ./install_ur_moveit_tc_humble.sh --ws ~/ROS2_UR_manipulation_ws --fix-ros-repo
EOF
}

WS=""
ROS_DISTRO="humble"
FIX_ROS_REPO="false"
PIN_MOVEIT_MINOR=""   # e.g. "2.5.9" -> pins to "2.5.9-*"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws) WS="${2:-}"; shift 2 ;;
    --rosdistro) ROS_DISTRO="${2:-}"; shift 2 ;;
    --fix-ros-repo) FIX_ROS_REPO="true"; shift 1 ;;
    --pin-moveit-minor) PIN_MOVEIT_MINOR="${2:-}"; shift 2 ;;
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
WS="$(readlink -f "${WS}")"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Fix ROS repo: ${FIX_ROS_REPO}"
echo "Pin MoveIt minor: ${PIN_MOVEIT_MINOR:-<none>}"

if [[ ! -d "${WS}" ]]; then
  echo "ERROR: Workspace does not exist: ${WS}"
  exit 1
fi

# ------------------------------------------------------------
# 0) (Optional) Fix ROS 2 apt repo Signed-By conflicts
#     Use ONLY if you have repo/key errors.
# ------------------------------------------------------------
if [[ "${FIX_ROS_REPO}" == "true" ]]; then
  echo ""
  echo "[0/7] Fixing ROS 2 apt repository (Signed-By conflicts)..."

  UBUNTU_CODENAME="jammy"
  KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
  ROS_LIST="/etc/apt/sources.list.d/ros2.list"

  sudo mkdir -p /etc/apt/keyrings

  # Remove only entries referencing packages.ros.org/ros2/ubuntu to avoid duplicates
  sudo bash -c '
set -e
for f in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
  [ -f "$f" ] || continue
  if grep -q "packages.ros.org/ros2/ubuntu" "$f"; then
    rm -f "$f"
  fi
done
if [ -f /etc/apt/sources.list ]; then
  sed -i "\|packages.ros.org/ros2/ubuntu|d" /etc/apt/sources.list
fi
'

  sudo apt-get update -y || true
  sudo apt-get install -y curl gnupg ca-certificates

  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor | sudo tee "${KEYRING}" > /dev/null

  echo "deb [arch=$(dpkg --print-architecture) signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
    sudo tee "${ROS_LIST}" > /dev/null
fi

# ------------------------------------------------------------
# 1) APT health (important in preconfigured images)
# ------------------------------------------------------------
echo ""
echo "[1/7] APT health: update + fix-broken + full-upgrade..."
sudo apt update
sudo apt --fix-broken install -y
sudo apt full-upgrade -y
sudo apt autoremove -y

# ------------------------------------------------------------
# 2) (Optional) Pin MoveIt to a minor version (safe glob, no timestamps)
# ------------------------------------------------------------
if [[ -n "${PIN_MOVEIT_MINOR}" ]]; then
  echo ""
  echo "[2/7] Pinning MoveIt to ${PIN_MOVEIT_MINOR}-* (safe glob)..."
  sudo tee /etc/apt/preferences.d/moveit-pin.pref >/dev/null <<EOF
Package: ros-${ROS_DISTRO}-moveit*
Pin: version ${PIN_MOVEIT_MINOR}-*
Pin-Priority: 1001
EOF
  sudo apt update
else
  echo ""
  echo "[2/7] MoveIt pinning skipped."
fi

# ------------------------------------------------------------
# 3) Install packages (your original list + build tools)
#     This should NOT conflict with TheConstruct if repos are consistent.
# ------------------------------------------------------------
echo ""
echo "[3/7] Installing UR + MoveIt + ros2_control + Gazebo Classic integration packages..."

sudo apt install -y \
  python3-pip \
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

# ------------------------------------------------------------
# 4) Make MoveIt consistent (this is what fixed your mixed versions)
# ------------------------------------------------------------
echo ""
echo "[4/7] Reinstalling MoveIt meta packages for consistency..."
sudo apt install -y --reinstall \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-ros \
  ros-${ROS_DISTRO}-moveit-planners-ompl

# ------------------------------------------------------------
# 5) Source ROS (avoid set -u issues) + clean stale prefix paths
# ------------------------------------------------------------
echo ""
echo "[5/7] Sourcing ROS env + cleaning stale prefix paths..."

set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u

clean_prefix_var() {
  local var="$1"
  local value="${!var-}"
  [[ -z "${value}" ]] && return 0
  local new=""
  IFS=':' read -r -a parts <<< "${value}"
  for p in "${parts[@]}"; do
    [[ -d "${p}" ]] && new="${new:+${new}:}${p}"
  done
  export "${var}=${new}"
}
clean_prefix_var AMENT_PREFIX_PATH
clean_prefix_var COLCON_PREFIX_PATH
clean_prefix_var CMAKE_PREFIX_PATH

# ------------------------------------------------------------
# 6) rosdep (best-effort)
# ------------------------------------------------------------
echo ""
echo "[6/7] rosdep install (best-effort)..."

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update || true
  if [[ -d "${WS}/src" ]]; then
    rosdep install --from-paths "${WS}/src" --ignore-src -r -y --rosdistro "${ROS_DISTRO}" || true
  fi
else
  echo "rosdep not found; skipping."
fi

# ------------------------------------------------------------
# 7) Build workspace
# ------------------------------------------------------------
echo ""
echo "[7/7] Building workspace (symlink install)..."
cd "${WS}"
rm -rf build install log
colcon build --symlink-install

echo ""
echo "DONE."
echo "New terminal:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
