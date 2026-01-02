#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./install_ur_moveit_keep_desktop_full.sh --ws /path/to/workspace [--pin-exact <apt_version_string>]

Examples:
  ./install_ur_moveit_keep_desktop_full.sh --ws ~/ROS2_UR_manipulation_ws
  ./install_ur_moveit_keep_desktop_full.sh --ws ~/ROS2_UR_manipulation_ws \
      --pin-exact 2.5.9-1jammy.20251119.004651

Behavior:
  - Keeps ros-humble-desktop-full (does NOT remove ros-gz/gz/ign packages).
  - Repairs APT (fix-broken) and prevents Signed-By conflicts for ROS2 repo.
  - Installs UR + MoveIt + Gazebo Classic packages if needed.
  - Pins MoveIt to ONE version:
      * Default: pins to the current apt "Candidate" version (stable & reproducible per repo snapshot).
      * Optional: --pin-exact pins to an exact apt version string.
EOF
}

WS=""
PIN_EXACT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws) WS="${2:-}"; shift 2 ;;
    --pin-exact) PIN_EXACT="${2:-}"; shift 2 ;;
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

# Canonical ROS2 keyring + list (avoid Signed-By conflicts)
ROS_KEYRING="/etc/apt/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

echo "Workspace: ${WS}"
echo "ROS distro: ${ROS_DISTRO}"
echo "Pin exact MoveIt version: ${PIN_EXACT:-<none>}"
echo "Note: This script keeps ros-humble-desktop-full and does NOT uninstall ros-gz / gz / ign packages."

###############################################################################
# 1) Fix ROS2 repo Signed-By conflicts (without removing gz packages)
###############################################################################
echo "[1/7] Repairing ROS 2 APT sources (Signed-By conflicts)..."

sudo mkdir -p /etc/apt/keyrings

# Remove ONLY entries that point to packages.ros.org/ros2/ubuntu to avoid duplicates.
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

# Minimal tools
sudo apt-get update -y || true
sudo apt-get install -y curl gnupg ca-certificates

# Key + single repo entry
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  gpg --dearmor | sudo tee "${ROS_KEYRING}" > /dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
  sudo tee "${ROS_LIST}" > /dev/null

###############################################################################
# 2) Repair APT state (your PC had fix-broken issues)
###############################################################################
echo "[2/7] Repairing APT dependency state (fix-broken) + full upgrade..."
sudo apt-get update -y
sudo apt-get --fix-broken install -y
sudo apt-get full-upgrade -y
sudo apt-get autoremove -y

###############################################################################
# 3) Install required packages (without removing desktop-full)
###############################################################################
echo "[3/7] Installing UR + MoveIt + Gazebo Classic packages (no removals)..."

sudo apt-get install -y \
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
  ros-${ROS_DISTRO}-gazebo-ros \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-gazebo-ros2-control \
  gazebo

###############################################################################
# 4) Pin MoveIt to ONE version (default: pin to Candidate)
###############################################################################
echo "[4/7] Pinning MoveIt to a single version..."

MOVEIT_PKGS=(
  ros-${ROS_DISTRO}-moveit-core
  ros-${ROS_DISTRO}-moveit-common
  ros-${ROS_DISTRO}-moveit-ros-move-group
  ros-${ROS_DISTRO}-moveit-ros-planning-interface
  ros-${ROS_DISTRO}-moveit-ros-planning
  ros-${ROS_DISTRO}-moveit-planners-ompl
  ros-${ROS_DISTRO}-moveit-ros
)

get_candidate_version() {
  local pkg="$1"
  apt-cache policy "$pkg" 2>/dev/null | awk '/Candidate:/{print $2; exit}'
}

if [[ -n "${PIN_EXACT}" ]]; then
  MOVEIT_VER="${PIN_EXACT}"
else
  MOVEIT_VER="$(get_candidate_version "ros-${ROS_DISTRO}-moveit-core")"
fi

if [[ -z "${MOVEIT_VER}" || "${MOVEIT_VER}" == "(none)" ]]; then
  echo "ERROR: Could not determine MoveIt version from apt."
  echo "Check: apt-cache policy ros-${ROS_DISTRO}-moveit-core"
  exit 1
fi

echo "Using MoveIt version: ${MOVEIT_VER}"

# Install exactly that version for the main coupling set.
# If some subpackages do not exist with exactly that version, apt will fail early (good).
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-moveit-core=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-common=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-ros-move-group=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-ros-planning-interface=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-ros-planning=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-planners-ompl=${MOVEIT_VER}" \
  "ros-${ROS_DISTRO}-moveit-ros=${MOVEIT_VER}"

# Hold to prevent drifting back to mixed versions
sudo apt-mark hold "${MOVEIT_PKGS[@]}"

sudo ldconfig

###############################################################################
# 5) rosdep for workspace
###############################################################################
echo "[5/7] rosdep install for workspace..."

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# source ROS setup (avoid set -u unbound var issues)
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u

if [ -d "${WS}/src" ]; then
  rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${ROS_DISTRO}" || true
else
  echo "WARNING: ${WS}/src not found; skipping rosdep."
fi

###############################################################################
# 6) Build workspace
###############################################################################
echo "[6/7] Building workspace..."
if [ -d "${WS}" ]; then
  rm -rf "${WS}/build" "${WS}/install" "${WS}/log"
  cd "${WS}"
  # ROS already sourced above; keep it consistent
  colcon build --symlink-install
fi

###############################################################################
# 7) Post-check
###############################################################################
echo "[7/7] Post-check: MoveIt versions installed (should match) + hold status"

dpkg -l | grep -E "ros-${ROS_DISTRO}-moveit-(core|common|ros-move-group|ros-planning-interface|planners-ompl)" || true
echo ""
echo "Held MoveIt packages:"
apt-mark showhold | grep -E "ros-${ROS_DISTRO}-moveit-" || true

echo ""
echo "DONE. In a NEW terminal:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
