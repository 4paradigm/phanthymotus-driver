#!/usr/bin/env bash
# build.sh — 构建 DJI PSDK 基础镜像 (所有 DJI 机型共享)
#
# 用法：
#   ./build.sh [--push] [--mirror tuna|tencent|none]
#
# 仅在 PSDK 版本更新时需要重新构建。

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 配置 ──────────────────────────────────────────────────────────────
REGISTRY="${REGISTRY:-bj-warehouse.tencentcloudcr.com}"
IMAGE_NAMESPACE="${IMAGE_NAMESPACE:-phanthy-motus}"
IMAGE_NAME="psdk-base"
PSDK_VERSION="3.16.0"
TAG="${IMAGE_NAME}:${PSDK_VERSION}"
TAG_LATEST="${IMAGE_NAME}:latest"

PUSH=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --push) PUSH="--push"; shift ;;
        *) shift ;;
    esac
done

FULL_TAG="${REGISTRY}/${IMAGE_NAMESPACE}/${TAG}"
FULL_TAG_LATEST="${REGISTRY}/${IMAGE_NAMESPACE}/${TAG_LATEST}"

echo "=== Building DJI PSDK Base Image ==="
echo "  Image: ${FULL_TAG}"
echo "  PSDK:  v${PSDK_VERSION}"
echo ""

docker buildx build \
    --platform linux/arm64 \
    --file "${SCRIPT_DIR}/Dockerfile" \
    --tag "${FULL_TAG}" \
    --tag "${FULL_TAG_LATEST}" \
    ${PUSH} \
    "${SCRIPT_DIR}"

echo ""
echo "Done: ${FULL_TAG}"
