#!/usr/bin/env bash
set -euo pipefail

# Install the host-side Odin2 ROS services shipped with this driver.  This is
# intentionally idempotent: it rebuilds the vendor depth node only when the
# workspace is missing and refreshes the calibration for the currently seen
# Odin2 device before enabling the services.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPTH_WS="${ODIN2_DEPTH_WS:-/home/ubuntu/odin-depth-ws}"
DEPTH_SRC="${DEPTH_WS}/src/odin_ros_driver"
# odin_ros_driver v0.14.4 — the revision validated against the on-board Odin2
# firmware: provides the `pcd2depth_ros2_node` the depth service runs and the
# `config/control_command.yaml` key this script rewrites.  Pinned so an OS
# recovery always reproduces the same depth service instead of silently
# tracking upstream's moving default branch.
ODIN_ROS_DRIVER_REV="v0.14.4"
CALIB_SRC="$(find /home/ubuntu/odin/src/ros_driver/config -maxdepth 1 -type f -name 'camera_calib_*.yaml' -print | sort | tail -n 1)"

if [[ -z "${CALIB_SRC}" ]]; then
    echo "No Odin2 camera calibration found under /home/ubuntu/odin/src/ros_driver/config" >&2
    exit 1
fi

mkdir -p "${DEPTH_WS}/src"
if [[ ! -d "${DEPTH_SRC}/.git" ]]; then
    git clone --branch "${ODIN_ROS_DRIVER_REV}" --depth 1 \
        https://github.com/manifoldsdk/odin_ros_driver.git "${DEPTH_SRC}"
fi
if [[ "$(git -C "${DEPTH_SRC}" rev-parse --short HEAD 2>/dev/null)" != \
      "$(git ls-remote https://github.com/manifoldsdk/odin_ros_driver.git \
            "refs/tags/${ODIN_ROS_DRIVER_REV}" 2>/dev/null | cut -c1-7)" ]]; then
    echo "odin_ros_driver checkout does not match pinned tag ${ODIN_ROS_DRIVER_REV}" >&2
    exit 1
fi
sed -i -E 's/^  senddepth: 0/  senddepth: 1/' \
    "${DEPTH_SRC}/config/control_command.yaml"

# pcd2depth_ros2_node expects the legacy cam_0 schema.  The current Odin2
# driver writes camera_calib_<SN>.yaml with resolution-specific cam_0_* keys.
calib_tmp="$(mktemp "${DEPTH_WS}/calib.yaml.XXXXXX")"
trap 'rm -f "${calib_tmp}"' EXIT
sed -E \
    -e 's/^img_topic:/img_topic_0:/' \
    -e 's/^cam_0_[0-9]+_[0-9]+:/cam_0:/' \
    "${CALIB_SRC}" > "${calib_tmp}"
install -m 0644 "${calib_tmp}" "${DEPTH_WS}/calib.yaml"
trap - EXIT
rm -f "${calib_tmp}"

source /opt/ros/humble/setup.bash
# Build from the workspace so colcon puts build/install/log under
# ${DEPTH_WS}, matching the path the depth service sources unconditionally.
cd "${DEPTH_WS}"
colcon build --packages-select odin_ros_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

sudo install -m 0644 "${SCRIPT_DIR}/engineai-odin2.service" \
    /etc/systemd/system/engineai-odin2.service
sudo install -m 0644 "${SCRIPT_DIR}/engineai-odin2-depth.service" \
    /etc/systemd/system/engineai-odin2-depth.service
sudo systemctl daemon-reload
sudo systemctl enable --now engineai-odin2.service
sudo systemctl enable --now engineai-odin2-depth.service

echo "Odin2 services installed and enabled"
