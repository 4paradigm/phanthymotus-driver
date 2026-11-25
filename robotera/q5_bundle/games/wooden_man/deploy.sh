#!/usr/bin/env bash
# Usage on the robot: bash deploy.sh IMAGE /path/to/robotera-q5-wooden-man.tar
set -Eeuo pipefail
image="${1:?Usage: deploy.sh IMAGE IMAGE_TAR}"
archive="${2:?Usage: deploy.sh IMAGE IMAGE_TAR}"
cd /opt/phanthy-motus
test -f docker-compose.yml
test -f "$archive"
docker load --input "$archive"
test "$(docker image inspect "$image" --format '{{.Architecture}}')" = arm64
docker compose -f docker-compose.yml config --services | grep -qx robotera-q5
# Compose merges this override with the current, robot-specific device/network config.
cat > docker-compose.wooden-man.yml <<EOF
services:
  robotera-q5:
    image: ${image}
EOF
docker compose -f docker-compose.yml -f docker-compose.wooden-man.yml config --quiet
docker compose -f docker-compose.yml -f docker-compose.wooden-man.yml up -d --no-deps --pull never --force-recreate robotera-q5
docker inspect embodied-robotera-q5 --format '{{.Config.Image}} {{.State.Status}}'
docker logs --tail 40 embodied-robotera-q5
