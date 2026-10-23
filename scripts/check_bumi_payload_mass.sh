#!/usr/bin/env bash
# Local only: no SDK/ROS, network, motor writes, image push or marketplace calls.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 noetix/bumi/payload_mass_cli.py mock
python3 -m pytest noetix/bumi/tests tests/test_bumi_bundle.py \
  tests/test_bumi_loco_servo.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py \
  tests/test_dockerfile_copies.py tests/test_file_upload_endpoint.py -q -p no:cacheprovider
