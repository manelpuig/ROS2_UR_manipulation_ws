#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if present
if [ -f /root/ROS2_UR_manipulation_ws/install/setup.bash ]; then
  source /root/ROS2_UR_manipulation_ws/install/setup.bash
fi

# DDS / ROS 2 networking (clear & explicit)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Teaching banner
echo "=============================================="
echo " ROS 2 Humble - Docker PC"
echo "----------------------------------------------"
echo " ROS_DOMAIN_ID                  = $ROS_DOMAIN_ID"
echo " ROS_AUTOMATIC_DISCOVERY_RANGE  = $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo " ROS_STATIC_PEERS               = $ROS_STATIC_PEERS"
echo " CYCLONEDDS_URI                 = $CYCLONEDDS_URI"
echo "=============================================="

exec "$@"
