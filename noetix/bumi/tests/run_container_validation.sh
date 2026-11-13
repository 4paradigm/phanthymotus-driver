#!/usr/bin/env bash
# Run only isolated validation, never /work/main.py or a camera/SDK controller.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
source /bumi_ros_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=219
export PYTHONDONTWRITEBYTECODE=1
validation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--serve" ]]; then
    exec python3 -B "$validation_dir/container_image_quality_smoke.py" --serve
fi
python3 -B "$validation_dir/container_image_quality_smoke.py" -v
exec python3 -B -m unittest discover -s "$validation_dir" -p "${1:-test_image_quality_ros.py}" -v
