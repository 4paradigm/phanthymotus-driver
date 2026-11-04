#!/usr/bin/env bash
# Local fake-backend checks only. Does not SSH, deploy, or contact a robot.
set -euo pipefail
cd "$(dirname "$0")/.."
FR3_PYTHON="${FR3_PYTHON:-python3}"
export PYTHONPATH="${PWD}${PYTHONPATH:+:${PYTHONPATH}}"
"${FR3_PYTHON}" scripts/check_service_yml.py franka/fr3
"${FR3_PYTHON}" -m pytest -q \
  tests/test_franka_joint_control.py tests/test_franka_control_session.py \
  tests/test_franka_driver.py tests/test_franka_hardware.py \
  tests/test_franka_ext_camera.py tests/test_franka_realsense.py \
  tests/test_franka_camera_contract.py tests/test_franka_discovery.py \
  tests/test_franka_joint_http.py tests/test_franka_remote_backend.py tests/test_vendor_runtime_bundle.py
"${FR3_PYTHON}" -m pytest -q tests/test_dockerfile_copies.py -k franka
"${FR3_PYTHON}" -m unittest discover -s franka/fr3/native -p test_session.py
FR3_TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$FR3_TMP_DIR"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wpedantic \
  franka/fr3/native/test_joint_motion.cpp -o "$FR3_TMP_DIR/joint-motion-test"
"$FR3_TMP_DIR/joint-motion-test"
