#!/bin/bash
set -e

# Keep the image usable with ROS installations that expose setup.sh rather
# than setup.bash, while failing clearly if the configured base image has no
# ROS installation at all.
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ros_setup}" && -f /opt/ros/humble/setup.sh ]]; then
    ros_setup=/opt/ros/humble/setup.sh
fi
if [[ ! -f "${ros_setup}" ]]; then
    echo "[as2w] ROS Humble setup file is missing in the image: ${ros_setup}" >&2
    echo "[as2w] Rebuild with the ROS base image (ROS_BASE_IMAGE must provide /opt/ros/humble)." >&2
    exit 127
fi

source "${ros_setup}"
exec "$@" "${NETWORK_INTERFACE:-}"
