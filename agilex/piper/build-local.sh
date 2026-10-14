#!/usr/bin/env bash
# Default amd64 build; use build-arm64.sh for an ARM64 development machine.
# Usage: build-local.sh [IMAGE] [additional docker build arguments...]
set -euo pipefail
driver_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${driver_dir}/../.." && pwd)"
stage_dir="$(mktemp -d "${TMPDIR:-/tmp}/piper-build.XXXXXX")"
trap 'rm -rf "${stage_dir}"' EXIT
cp -R "${driver_dir}/." "${stage_dir}/"
cp -R "${repo_dir}/common" "${stage_dir}/common"
platform="${PIPER_BUILD_PLATFORM:-linux/amd64}"
case "$platform" in
  linux/amd64|linux/arm64) ;;
  *) echo "Unsupported PiPER build platform: $platform" >&2; exit 1 ;;
esac
if docker buildx version >/dev/null 2>&1; then
  docker buildx build --platform "$platform" \
    --load --tag "${1:-agilex-piper:local}" "${@:2}" "${stage_dir}"
else
  # Older Jetson installations may have Docker without the buildx CLI plugin.
  # Use native builds only; never install an emulator/plugin or change Docker.
  case "$(uname -m)" in
    aarch64|arm64) native_platform=linux/arm64 ;;
    x86_64|amd64) native_platform=linux/amd64 ;;
    *) echo "Unknown native architecture; buildx is required." >&2; exit 1 ;;
  esac
  if [[ "$platform" != "$native_platform" ]]; then
    echo "Cross-platform build requires buildx; native platform is $native_platform." >&2
    exit 1
  fi
  DOCKER_BUILDKIT=0 docker build --platform "$platform" \
    --tag "${1:-agilex-piper:local}" "${@:2}" "${stage_dir}"
fi
