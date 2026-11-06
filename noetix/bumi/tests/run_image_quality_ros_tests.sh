#!/usr/bin/env bash
# Local synthetic ROS2 transport tests. Never starts a robot/camera/SDK.
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${IQG_TEST_ROS_DOMAIN_ID:-219}"
export PYTHONDONTWRITEBYTECODE=1
test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d /tmp/iqg-test-deps ]]; then
    export PYTHONPATH="/tmp/iqg-test-deps:${PYTHONPATH:-}"
fi
exec python3 -B -m unittest discover -s "$test_dir" -p "${1:-test_image_quality_ros.py}" -v
