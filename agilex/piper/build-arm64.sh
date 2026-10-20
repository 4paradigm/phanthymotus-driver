#!/usr/bin/env bash
# Build only. Does not start services, access hardware or modify the canvas.
set -euo pipefail
driver_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PIPER_BUILD_PLATFORM=linux/arm64
exec bash "${driver_dir}/build-local.sh" "${1:-agilex-piper:arm64-local}" "${@:2}"
