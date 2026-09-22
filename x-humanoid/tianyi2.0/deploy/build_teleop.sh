#!/usr/bin/env bash
# Candidate build only. Match build.sh's context extras layout.
set -euo pipefail
if [[ $# != 1 ]]; then
  echo 'Usage: build_teleop.sh LOCAL_IMAGE_TAG (set TMPDIR to your task directory on a robot)' >&2
  exit 2
fi
driver_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd "$driver_dir/../.." && pwd)"
test -d "$repo_root/common"
stage="$(mktemp -d "${TMPDIR:-/tmp}/tianyi-teleop-build.XXXXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
cp -R "$driver_dir/." "$stage/"
cp -R "$repo_root/common" "$stage/common"
cp -R "$repo_root/robotera/q5_bundle/vendor/audio_msgs" "$stage/audio_msgs"
docker build --platform linux/arm64 --progress plain \
  --build-arg ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f \
  --build-arg PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple \
  --build-arg UBUNTU_PORTS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports \
  -t "$1" "$stage"
