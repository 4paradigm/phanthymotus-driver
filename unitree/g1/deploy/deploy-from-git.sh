#!/usr/bin/env bash
set -euo pipefail

TARGET="${1:-g1-bj-wifi}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_REPO="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
SOURCE_REF="${SOURCE_REF:-$(git -C "$LOCAL_REPO" branch --show-current)}"
EXPECTED_COMMIT="${EXPECTED_COMMIT:-$(git -C "$LOCAL_REPO" rev-parse HEAD)}"
ROS_BASE_IMAGE="${ROS_BASE_IMAGE:-bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest}"
IMAGE="${IMAGE:-phanthy-g1-driver:git-${EXPECTED_COMMIT:0:7}}"
REMOTE_REPO="${REMOTE_REPO:-~/hanzebei/phanthymotus-driver}"
COMPOSE_BASE="${COMPOSE_BASE:-/opt/phanthy-motus/docker-compose.yml}"

if [[ -z "$SOURCE_REF" ]]; then
  echo "SOURCE_REF is required when the local checkout is detached" >&2
  exit 1
fi
git -C "$LOCAL_REPO" cat-file -e "$EXPECTED_COMMIT^{commit}"
if [[ -n "$(git -C "$LOCAL_REPO" status --porcelain)" ]]; then
  echo "Local checkout must be clean before deployment: $LOCAL_REPO" >&2
  exit 1
fi

if [[ -z "${REPO_URL:-}" ]]; then
  if git -C "$LOCAL_REPO" remote get-url fork >/dev/null 2>&1; then
    REPO_URL="$(git -C "$LOCAL_REPO" remote get-url fork)"
  else
    REPO_URL="$(git -C "$LOCAL_REPO" remote get-url origin)"
  fi
fi
case "$REPO_URL" in
  https://github.com/*)
    REPO_URL="git@github.com:${REPO_URL#https://github.com/}"
    ;;
esac

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  echo "DRY_RUN=PASS target=$TARGET repo=$REPO_URL ref=$SOURCE_REF commit=$EXPECTED_COMMIT base=$ROS_BASE_IMAGE image=$IMAGE remote_repo=$REMOTE_REPO"
  exit 0
fi

ssh "$TARGET" bash -s -- \
  "$REPO_URL" \
  "$SOURCE_REF" \
  "$EXPECTED_COMMIT" \
  "$ROS_BASE_IMAGE" \
  "$IMAGE" \
  "$REMOTE_REPO" \
  "$COMPOSE_BASE" <<'REMOTE_DEPLOY'
set -euo pipefail

repo_url="$1"
source_ref="$2"
expected_commit="$3"
ros_base_image="$4"
image="$5"
remote_repo_arg="$6"
compose_base="$7"

if [[ "$remote_repo_arg" == "~/"* ]]; then
  remote_repo="$HOME/${remote_repo_arg:2}"
else
  remote_repo="$remote_repo_arg"
fi
remote_compose="${remote_repo}.compose.${expected_commit:0:7}.yml"

command -v git >/dev/null
command -v docker >/dev/null
docker image inspect "$ros_base_image" >/dev/null

if [[ -e "$remote_repo" && ! -d "$remote_repo/.git" ]]; then
  echo "Remote checkout path exists but is not a Git repository: $remote_repo" >&2
  exit 1
fi

if [[ -d "$remote_repo/.git" ]]; then
  [[ -z "$(git -C "$remote_repo" status --porcelain)" ]] || {
    echo "Remote checkout is dirty: $remote_repo" >&2
    exit 1
  }
  if [[ "$(git -C "$remote_repo" remote get-url origin)" == "$repo_url" ]]; then
    source_remote="origin"
  else
    source_remote="deploy-source"
    if git -C "$remote_repo" remote get-url "$source_remote" >/dev/null 2>&1; then
      git -C "$remote_repo" remote set-url "$source_remote" "$repo_url"
    else
      git -C "$remote_repo" remote add "$source_remote" "$repo_url"
    fi
  fi
else
  git clone --filter=blob:none --no-checkout "$repo_url" "$remote_repo"
  source_remote="origin"
fi

git -C "$remote_repo" fetch --prune --no-tags "$source_remote" \
  "refs/heads/$source_ref:refs/remotes/$source_remote/$source_ref"
remote_commit="$(git -C "$remote_repo" rev-parse "refs/remotes/$source_remote/$source_ref")"
[[ "$remote_commit" == "$expected_commit" ]] || {
  echo "Remote source mismatch: expected=$expected_commit actual=$remote_commit" >&2
  exit 1
}
git -C "$remote_repo" checkout --detach "$expected_commit"

docker build --pull=false \
  --build-arg "ROS_BASE_IMAGE=$ros_base_image" \
  --label "phanthy.source_commit=$expected_commit" \
  --label "phanthy.source_ref=$source_ref" \
  -f "$remote_repo/unitree/g1/Dockerfile" \
  -t "$image" \
  "$remote_repo/unitree/g1"

built_commit="$(docker image inspect "$image" --format '{{ index .Config.Labels "phanthy.source_commit" }}')"
[[ "$built_commit" == "$expected_commit" ]]

service_template="$remote_repo/unitree/g1/deploy/service.yml"
[[ -f "$service_template" ]] || {
  echo "Driver Compose service template is missing: $service_template" >&2
  exit 1
}
{
  echo "services:"
  sed "s|__IMAGE__|$image|" "$service_template" | sed 's/^/  /'
} > "$remote_compose"
grep -Fq "image: $image" "$remote_compose"
! grep -Fq '__IMAGE__' "$remote_compose"

docker compose -p phanthy-motus \
  -f "$compose_base" -f "$remote_compose" \
  up -d --no-deps --force-recreate unitree-g1

container_id="$(docker compose -p phanthy-motus \
  -f "$compose_base" -f "$remote_compose" \
  ps -q unitree-g1)"
[[ -n "$container_id" ]] || {
  echo "Compose did not return a container for service unitree-g1" >&2
  exit 1
}

actual_image="$(docker inspect "$container_id" --format '{{.Config.Image}}')"
status="$(docker inspect "$container_id" --format '{{.State.Status}}')"
[[ "$status" == "running" && "$actual_image" == "$image" ]]

for _ in $(seq 1 30); do
  response="$(curl -fsS -H 'Content-Type: application/json' \
    --data '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}' \
    http://127.0.0.1:15701 2>/dev/null || true)"
  if grep -Fq '"navigation_lidar"' <<<"$response" \
    && grep -Fq '"navigation_imu"' <<<"$response" \
    && grep -Fq '"camera_rgb_frame"' <<<"$response" \
    && grep -Fq '"camera_depth_frame"' <<<"$response" \
    && grep -Fq '/ubuntu/camera/rgb_frame' <<<"$response" \
    && grep -Fq '/ubuntu/camera/depth_frame' <<<"$response" \
    && grep -Fq 'phanthy.sensor.camera_rgb_frame.v1' <<<"$response" \
    && grep -Fq 'phanthy.sensor.camera_depth_frame.v1' <<<"$response" \
    && grep -Fq '/ubuntu/navigation/nav2/velocity_proposal' <<<"$response"; then
    docker exec -w /work "$container_id" python3 -c '
from velocity_proposal import ProposalLimits

limits = ProposalLimits()
assert limits.min_x == -1.0
assert limits.max_x == 1.0
assert limits.max_abs_y == 1.0
assert limits.max_abs_yaw == 2.0
print("VELOCITY_CONTRACT=PASS vx=[-1,1] vy=[-1,1] vyaw=[-2,2]")
'
    docker exec -w /work "$container_id" python3 -c '
from camera_frame import DEPTH_SCHEMA, ENVELOPE_FORMAT, ENVELOPE_MAGIC, RGB_SCHEMA

assert ENVELOPE_MAGIC == b"PSE1"
assert ENVELOPE_FORMAT == "application/vnd.phanthy.sensor-envelope.v1"
assert RGB_SCHEMA == "phanthy.sensor.camera_rgb_frame.v1"
assert DEPTH_SCHEMA == "phanthy.sensor.camera_depth_frame.v1"
print("CAMERA_FRAME_CONTRACT=PASS envelope=PSE1 schemas=frame.v1")
'
    echo "DEPLOYMENT=PASS status=$status image=$actual_image source_commit=$built_commit"
    echo "TOOLS=PASS navigation_lidar navigation_imu camera_rgb_frame camera_depth_frame loco.velocity_proposal"
    exit 0
  fi
  sleep 1
done

docker logs --tail 80 "$container_id" >&2
echo "Driver started but expected G1 navigation tools were not ready" >&2
exit 1
REMOTE_DEPLOY
