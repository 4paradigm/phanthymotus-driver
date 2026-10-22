#!/usr/bin/env bash
# Inspect an already pulled image without starting a container or touching CAN.
set -euo pipefail
image="${1:?Usage: bash check-image.sh IMAGE}"
architecture="$(docker image inspect "$image" --format '{{.Architecture}}')"
if [[ "$architecture" != amd64 ]]; then
    echo "拒绝部署：目标 Ubuntu 为 amd64，镜像为 $architecture。" >&2
    echo "请使用 agilex/piper/build-local.sh 构建 linux/amd64，或由 Bot 生成 amd64 镜像。" >&2
    exit 1
fi
docker image inspect "$image" --format 'image={{.Id}} architecture={{.Architecture}} size_bytes={{.Size}}'
echo '架构检查通过。部署前确认旧控制程序已停止，保持 PIPER_MOTION_ENABLED=0。'
