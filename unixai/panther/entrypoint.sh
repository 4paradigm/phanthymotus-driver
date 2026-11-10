#!/usr/bin/env bash
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
source /panther_ws/install/setup.bash

export ROS_DOMAIN_ID="${PANTHER_ROS_DOMAIN_ID:-0}"
export AGENT_CORE_ROS_DOMAIN_ID="${AGENT_CORE_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export PYTHONUNBUFFERED=1
export RCUTILS_COLORIZED_OUTPUT=0

exec python3 /work/main.py

