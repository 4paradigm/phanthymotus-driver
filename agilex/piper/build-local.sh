#!/usr/bin/env bash
# Standalone build for the x86_64 controller; repository build.sh targets arm64.
set -euo pipefail
driver_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${driver_dir}/../.." && pwd)"
stage_dir="$(mktemp -d "${TMPDIR:-/tmp}/piper-build.XXXXXX")"
trap 'rm -rf "${stage_dir}"' EXIT
cp -R "${driver_dir}/." "${stage_dir}/"
cp -R "${repo_dir}/common" "${stage_dir}/common"
docker buildx build --platform "${PIPER_BUILD_PLATFORM:-linux/amd64}" \
  --load --tag "${1:-agilex-piper:local}" "${stage_dir}"
