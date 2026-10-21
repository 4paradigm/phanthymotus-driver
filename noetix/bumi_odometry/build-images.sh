#!/usr/bin/env bash
# Publish an independently reusable ARM64 dependencies image, then a small app.
# Usage: build-images.sh REGISTRY/deps:tag REGISTRY/app:tag
set -euo pipefail
if [[ $# != 2 ]]; then echo "usage: $0 DEPS_IMAGE APP_IMAGE" >&2; exit 2; fi
fusion_deps_tag="$1"
fusion_app_tag="$2"
fusion_root="$(cd "$(dirname "$0")/../.." && pwd)"
fusion_context="$(mktemp -d)"
trap 'rm -rf "$fusion_context"' EXIT
cp -R "$fusion_root/noetix/bumi_odometry/." "$fusion_context/"
cp -R "$fusion_root/common" "$fusion_context/common"
# Cold dependencies build is explicitly separate; application changes do not
# invalidate apt. Registry cache is reusable across CI workers.
docker buildx build --platform linux/arm64 --target dependencies \
  --cache-from "type=registry,ref=${fusion_deps_tag}-cache" \
  --cache-to "type=registry,ref=${fusion_deps_tag}-cache,mode=max" \
  --metadata-file "$fusion_context/deps-metadata.json" -t "$fusion_deps_tag" --push "$fusion_context"
fusion_digest="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["containerimage.digest"])' "$fusion_context/deps-metadata.json")"
[[ "$fusion_digest" == sha256:* ]] || { echo 'missing dependency digest' >&2; exit 1; }
docker buildx build --platform linux/arm64 --target runtime \
  --build-arg "FUSION_BASE_IMAGE=${fusion_deps_tag}@${fusion_digest}" \
  --cache-from "type=registry,ref=${fusion_app_tag}-cache" \
  --cache-to "type=registry,ref=${fusion_app_tag}-cache,mode=max" \
  --metadata-file "$fusion_context/app-metadata.json" -t "$fusion_app_tag" --push "$fusion_context"
printf 'Dependencies: %s@%s\n' "$fusion_deps_tag" "$fusion_digest"
cat "$fusion_context/app-metadata.json"
