#!/usr/bin/env bash
set -euo pipefail

# Install the host-side Odin2 ROS services shipped with this driver.  This is
# intentionally idempotent: it rebuilds the vendor depth node only when the
# workspace is missing and refreshes the calibration for the currently seen
# Odin2 device before enabling the services.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPTH_WS="${ODIN2_DEPTH_WS:-/home/ubuntu/odin-depth-ws}"
DEPTH_SRC="${DEPTH_WS}/src/odin_ros_driver"
CALIB_SRC="$(find /home/ubuntu/odin/src/ros_driver/config -maxdepth 1 -type f -name 'camera_calib_*.yaml' -print | sort | tail -n 1)"

if [[ -z "${CALIB_SRC}" ]]; then
    echo "No Odin2 camera calibration found under /home/ubuntu/odin/src/ros_driver/config" >&2
    exit 1
fi

mkdir -p "${DEPTH_WS}/src"
if [[ ! -d "${DEPTH_SRC}/.git" ]]; then
    git clone --depth 1 https://github.com/manifoldsdk/odin_ros_driver.git "${DEPTH_SRC}"
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
colcon build --base-paths "${DEPTH_WS}" --packages-select odin_ros_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

sudo install -m 0644 "${SCRIPT_DIR}/engineai-odin2.service" \
    /etc/systemd/system/engineai-odin2.service
sudo install -m 0644 "${SCRIPT_DIR}/engineai-odin2-depth.service" \
    /etc/systemd/system/engineai-odin2-depth.service
sudo systemctl daemon-reload
sudo systemctl enable --now engineai-odin2.service
sudo systemctl enable --now engineai-odin2-depth.service

echo "Odin2 services installed and enabled"
