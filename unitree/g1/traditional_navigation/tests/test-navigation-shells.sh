#!/usr/bin/env bash
set -euo pipefail

readonly TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly NAV_DIR="$(cd -- "${TEST_DIR}/.." && pwd -P)"
readonly DEPLOY_SCRIPT="${NAV_DIR}/deploy-g1-navigation-shadow.sh"
readonly ACCEPT_SCRIPT="${NAV_DIR}/accept-g1-navigation-shadow.sh"
readonly TIME_PROBE_SCRIPT="${NAV_DIR}/probe-g1-rgb-time-offset.sh"
readonly RECOVER_SCRIPT="${NAV_DIR}/recover-g1-rgb-preview.sh"
readonly RGB_CONFIG_TEST="${TEST_DIR}/test_rgb_livo_config.py"
readonly RGB_PCD_RECOVERY_TEST="${TEST_DIR}/test_rgb_pcd_recovery.py"
readonly FAKE_COMMAND="${TEST_DIR}/fake-navigation-command.sh"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

expect_status() {
  expected="$1"
  shift
  set +e
  "$@" >/dev/null 2>&1
  actual="$?"
  set -e
  test "${actual}" -eq "${expected}" || {
    fail "expected status ${expected}, got ${actual}: $*"
  }
}

bash -n "${DEPLOY_SCRIPT}"
bash -n "${ACCEPT_SCRIPT}"
bash -n "${TIME_PROBE_SCRIPT}"
bash -n "${RECOVER_SCRIPT}"
bash -n "${FAKE_COMMAND}"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v \
  "${RGB_CONFIG_TEST}" "${RGB_PCD_RECOVERY_TEST}"
rgb_patch_expected="$(sed -n 's/^FAST_LIVO2_RGB_QOS_PATCH_SHA256=//p' "${NAV_DIR}/source-lock.rgb.env")"
rgb_patch_actual="$(shasum -a 256 "${NAV_DIR}/fast-livo2-rgb-qos.patch" | awk '{print $1}')"
test "${rgb_patch_expected}" = "${rgb_patch_actual}" || fail "RGB QoS patch checksum drifted"
grep -Fq 'FAST_LIVO2_RGB_BASE_IMAGE=phanthy-fast-livo2:g1-1fcd0d0-n3save1' \
  "${NAV_DIR}/source-lock.rgb.env"
grep -Fq 'FAST_LIVO2_RGB_IMAGE=phanthy-fast-livo2:g1-1fcd0d0-n3rgbpreview2' \
  "${NAV_DIR}/source-lock.rgb.env"
grep -Fq 'rclcpp::SensorDataQoS image_qos' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'kMaxBufferedLidarFrames = 8' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'kMaxBufferedImuSamples = 400' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'kMaxBufferedImages = 4' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'lid_header_time_buffer.pop_front()' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'prepareFrameForColorization' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'do not grow a visual feature map' "${NAV_DIR}/fast-livo2-rgb-qos.patch"
grep -Fq 'rgb_pipeline: lio_colorize_only' "${NAV_DIR}/compose.rgb-preview.yml"

expect_status 2 "${DEPLOY_SCRIPT}"
expect_status 2 "${DEPLOY_SCRIPT}" preflight "bad target" ubuntu eth0
expect_status 2 env G1_MAP_NAME=../bad "${DEPLOY_SCRIPT}" start_mapping g1-sh-wifi ubuntu eth0
expect_status 2 env G1_MAP_NAME=sh_n3_smoke G1_PCD_SAVE_INTERVAL=0 \
  "${DEPLOY_SCRIPT}" start_mapping g1-sh-wifi ubuntu eth0
expect_status 2 env G1_MAP_NAME=../bad \
  "${DEPLOY_SCRIPT}" start_rgb_preview g1-sh-wifi ubuntu eth0
expect_status 2 env G1_MAP_NAME=sh_rgb_static G1_PCD_SAVE_INTERVAL=0 \
  "${DEPLOY_SCRIPT}" start_rgb_preview g1-sh-wifi ubuntu eth0
expect_status 2 "${TIME_PROBE_SCRIPT}" g1-sh-wifi
expect_status 2 env CONFIRM_G1_SHADOW_WRITE=YES \
  "${TIME_PROBE_SCRIPT}" g1-sh-wifi 29
expect_status 2 "${RECOVER_SCRIPT}"
expect_status 2 env G1_MAP_NAME=../bad CONFIRM_G1_SHADOW_WRITE=YES \
  "${RECOVER_SCRIPT}" g1-sh-wifi
expect_status 2 env G1_MAP_NAME=sh_rgb_interrupted \
  "${RECOVER_SCRIPT}" g1-sh-wifi
expect_status 2 "${ACCEPT_SCRIPT}" map g1-sh-wifi ubuntu eth0

readonly TEST_ROOT="$(mktemp -d /private/tmp/g1-navigation-shells.XXXXXX)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT
mkdir -p "${TEST_ROOT}/bin"
ln -s "${FAKE_COMMAND}" "${TEST_ROOT}/bin/docker"
ln -s "${FAKE_COMMAND}" "${TEST_ROOT}/bin/ssh"
readonly FAKE_PATH="${TEST_ROOT}/bin:${PATH}"

readonly TIME_PROBE_LOG="${TEST_ROOT}/time-probe.log"
readonly TIME_PROBE_OUTPUT="${TEST_ROOT}/time-probe.json"
readonly TIME_PROBE_STDOUT="${TEST_ROOT}/time-probe.output"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${TIME_PROBE_LOG}" \
G1_RGB_TIME_PROBE_OUTPUT="${TIME_PROBE_OUTPUT}" CONFIRM_G1_SHADOW_WRITE=YES \
  "${TIME_PROBE_SCRIPT}" g1-sh-wifi 120 >"${TIME_PROBE_STDOUT}"
grep -Fq 'rgb_time_probe=PASS' "${TIME_PROBE_STDOUT}"
grep -Fq 'offset_s=-0.032500000' "${TIME_PROBE_STDOUT}"
grep -Fq 'docker stop -t 20' "${TIME_PROBE_LOG}"
grep -Fq 'docker start' "${TIME_PROBE_LOG}"
python3 "${NAV_DIR}/probe-g1-rgb-time-offset.py" \
  --validate "${TIME_PROBE_OUTPUT}" >/dev/null

readonly RECOVER_LOG="${TEST_ROOT}/recover.log"
readonly RECOVER_OUTPUT="${TEST_ROOT}/recover.output"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${RECOVER_LOG}" \
G1_MAP_NAME=sh_rgb_interrupted CONFIRM_G1_SHADOW_WRITE=YES \
  "${RECOVER_SCRIPT}" g1-sh-wifi ubuntu eth0 >"${RECOVER_OUTPUT}"
grep -Fq 'rgb_pcd_recovery=PASS action=merge' "${RECOVER_OUTPUT}"
grep -Fq 'rgb_pcd_recovery=PASS action=validate' "${RECOVER_OUTPUT}"
grep -Fq 'skipped_zero_filled=10' "${RECOVER_OUTPUT}"
grep -Fq 'clean_shutdown=false' "${RECOVER_OUTPUT}"
grep -Fq 'test "${actual_exit}" = 255' "${RECOVER_LOG}"
grep -Fq 'refusing RGB recovery identity mismatch' "${RECOVER_LOG}"
grep -Fq 'all_rgb_points.recovered.pcd' "${RECOVER_LOG}"
grep -Fq -- '--skip-zero-filled-checkpoints' "${RECOVER_LOG}"
grep -Fq 'up -d --no-build --force-recreate --wait' "${RECOVER_LOG}"

readonly PREFLIGHT_LOG="${TEST_ROOT}/preflight.log"
(
  cd /private/tmp
  PATH="${FAKE_PATH}" G1_FAKE_LOG="${PREFLIGHT_LOG}" \
    "${DEPLOY_SCRIPT}" preflight g1-sh-wifi ubuntu eth0
)
grep -Fq "PREFLIGHT OK: no robot state changed" <(
  cd /private/tmp
  PATH="${FAKE_PATH}" G1_FAKE_LOG="${TEST_ROOT}/preflight-repeat.log" \
    "${DEPLOY_SCRIPT}" preflight g1-sh-wifi ubuntu eth0
)
if grep -Eq 'mkdir -p|docker (compose|build|load).*(up|down|build|load)' "${PREFLIGHT_LOG}"; then
  fail "preflight emitted a robot write command"
fi

set +e
start_output="$(
  PATH="${FAKE_PATH}" G1_FAKE_LOG="${TEST_ROOT}/start-denied.log" \
  G1_MAP_NAME=sh_n3_smoke \
    "${DEPLOY_SCRIPT}" start_mapping g1-sh-wifi ubuntu eth0 2>&1
)"
start_status="$?"
set -e
test "${start_status}" -eq 2 || fail "start_mapping without confirmation was not denied"
grep -Fq "Refusing robot write" <<<"${start_output}"

readonly START_LOG="${TEST_ROOT}/start.log"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${START_LOG}" \
G1_MAP_NAME=sh_n3_smoke G1_PCD_SAVE_INTERVAL=20 CONFIRM_G1_SHADOW_WRITE=YES \
  "${DEPLOY_SCRIPT}" start_mapping g1-sh-wifi ubuntu eth0 >/dev/null
grep -Fq "stat -c '%u'" "${START_LOG}"
grep -Fq 'G1_MAP_UID="${map_uid}"' "${START_LOG}"
grep -Fq 'G1_MAP_GID="${map_gid}"' "${START_LOG}"
grep -Fq '{{json .HostConfig.GroupAdd}}' "${START_LOG}"
grep -Fq 'test -z "$(docker inspect phanthy-fast-livo2-shadow' "${START_LOG}"
grep -Fq 'test -w /opt/fast_livo_ws/src/fast_livo/Log/pcd' "${START_LOG}"
grep -Fq 'group_add:' "${NAV_DIR}/compose.mapping.yml"
grep -Fq -- '- "${G1_MAP_GID:-1000}"' "${NAV_DIR}/compose.mapping.yml"
if grep -Eq '^    user:' "${NAV_DIR}/compose.mapping.yml"; then
  fail "mapping override changes the validated FAST-LIVO2 user"
fi

readonly STOP_LOG="${TEST_ROOT}/stop.log"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${STOP_LOG}" \
G1_MAP_NAME=sh_n3_smoke CONFIRM_G1_SHADOW_WRITE=YES \
  "${DEPLOY_SCRIPT}" stop_mapping g1-sh-wifi ubuntu eth0 >/dev/null

grep -Fq 'total += $1' "${STOP_LOG}"
grep -Fq 'stop fast-livo2' "${STOP_LOG}"
grep -Fq 'mapping_exit_code=' "${STOP_LOG}"
grep -Fq 'test "${mapping_exit_code}" = 0' "${STOP_LOG}"
grep -Fq 'mapping_restore_status=0' "${STOP_LOG}"
grep -Fq 'test "${mapping_restore_status}" -eq 0' "${STOP_LOG}"
grep -Fq 'refusing stop_mapping identity mismatch' "${STOP_LOG}"
identity_guard_line="$(grep -n 'refusing stop_mapping identity mismatch' "${STOP_LOG}" | head -n 1 | cut -d: -f1)"
stop_line="$(grep -n 'stop fast-livo2' "${STOP_LOG}" | head -n 1 | cut -d: -f1)"
restore_line="$(grep -n 'up -d --no-build --wait' "${STOP_LOG}" | tail -n 1 | cut -d: -f1)"
report_line="$(grep -n 'pcd_count=' "${STOP_LOG}" | tail -n 1 | cut -d: -f1)"
test "${identity_guard_line}" -lt "${stop_line}" || fail "mapping identity guard occurs after stop"
test -n "${restore_line}" || fail "ordinary shadow restore command missing"
test -n "${report_line}" || fail "PCD report command missing"
test "${restore_line}" -lt "${report_line}" || fail "map reporting occurs before shadow restore"
if grep -Fq 'docker logs' "${DEPLOY_SCRIPT}"; then
  fail "deployment script reads Docker logs on the G1"
fi

set +e
rgb_start_output="$(
  PATH="${FAKE_PATH}" G1_FAKE_LOG="${TEST_ROOT}/rgb-start-denied.log" \
  G1_MAP_NAME=sh_rgb_static \
    "${DEPLOY_SCRIPT}" start_rgb_preview g1-sh-wifi ubuntu eth0 2>&1
)"
rgb_start_status="$?"
set -e
test "${rgb_start_status}" -eq 2 || fail "start_rgb_preview without confirmation was not denied"
grep -Fq "Refusing robot write" <<<"${rgb_start_output}"

readonly RGB_START_LOG="${TEST_ROOT}/rgb-start.log"
readonly RGB_START_OUTPUT="${TEST_ROOT}/rgb-start.output"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${RGB_START_LOG}" \
G1_FAKE_EMPTY_PROBE_ONCE=1 G1_MAP_NAME=sh_rgb_static G1_PCD_SAVE_INTERVAL=20 \
G1_RGB_TIME_PROBE="${TIME_PROBE_OUTPUT}" CONFIRM_G1_SHADOW_WRITE=YES \
  "${DEPLOY_SCRIPT}" start_rgb_preview g1-sh-wifi ubuntu eth0 \
  >"${RGB_START_OUTPUT}" 2>&1
test "$(grep -Fc 'docker exec -i embodied-unitree-g1 python3 -' "${RGB_START_LOG}")" -eq 2 \
  || fail "RGB probe did not retry after a successful-but-empty response"
grep -Fq 'RGB probe attempt=1 invalid ssh_status=0 derive_status=1 bytes=0' \
  "${RGB_START_OUTPUT}"
grep -Fq 'docker build' "${RGB_START_LOG}"
grep -Fq 'compose.rgb-preview.yml' "${RGB_START_LOG}"
grep -Fq 'nominal_public_urdf' "${RGB_START_LOG}"
grep -Fq 'measured_callback_latency' "${RGB_START_LOG}"
grep -Fq 'slow_manual_preview' "${RGB_START_LOG}"
grep -Fq 'img_time_offset_s=-0.032500000' "${RGB_START_OUTPUT}"
grep -Fq '/navigation/cloud_registered_rgb' "${NAV_DIR}/compose.rgb-preview.yml"
grep -Fq '/camera/rgb' "${NAV_DIR}/compose.rgb-preview.yml"
grep -Fq 'healthcheck:' "${NAV_DIR}/compose.rgb-preview.yml"
grep -Fq "awk '/^[1-9][0-9]*\$\$/" "${NAV_DIR}/compose.rgb-preview.yml"

readonly RGB_STOP_LOG="${TEST_ROOT}/rgb-stop.log"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${RGB_STOP_LOG}" \
G1_MAP_NAME=sh_rgb_static CONFIRM_G1_SHADOW_WRITE=YES \
  "${DEPLOY_SCRIPT}" stop_rgb_preview g1-sh-wifi ubuntu eth0 >/dev/null
grep -Fq 'refusing stop_rgb_preview identity mismatch' "${RGB_STOP_LOG}"
grep -Fq 'stop fast-livo2' "${RGB_STOP_LOG}"
grep -Fq 'preview_restore_status=0' "${RGB_STOP_LOG}"
grep -Fq '^FIELDS x y z rgb' "${RGB_STOP_LOG}"
rgb_identity_line="$(grep -n 'refusing stop_rgb_preview identity mismatch' "${RGB_STOP_LOG}" | head -n 1 | cut -d: -f1)"
rgb_stop_line="$(grep -n 'stop fast-livo2' "${RGB_STOP_LOG}" | head -n 1 | cut -d: -f1)"
rgb_restore_line="$(grep -n 'up -d --no-build --wait' "${RGB_STOP_LOG}" | tail -n 1 | cut -d: -f1)"
rgb_report_line="$(grep -n 'rgb_pcd_count=' "${RGB_STOP_LOG}" | tail -n 1 | cut -d: -f1)"
test "${rgb_identity_line}" -lt "${rgb_stop_line}" || fail "RGB preview identity guard occurs after stop"
test "${rgb_restore_line}" -lt "${rgb_report_line}" || fail "RGB preview reports artifacts before LIO restore"

readonly ACCEPT_PREFLIGHT_LOG="${TEST_ROOT}/accept-preflight.log"
PATH="${FAKE_PATH}" G1_FAKE_LOG="${ACCEPT_PREFLIGHT_LOG}" \
  "${ACCEPT_SCRIPT}" preflight g1-sh-wifi ubuntu eth0 >/dev/null
if grep -Eq 'mkdir -p|docker (compose|build|load).*(up|down|build|load)' "${ACCEPT_PREFLIGHT_LOG}"; then
  fail "acceptance preflight emitted a robot write command"
fi

PATH="${FAKE_PATH}" G1_FAKE_LOG="${TEST_ROOT}/accept-lio.log" \
  "${ACCEPT_SCRIPT}" lio g1-sh-wifi ubuntu eth0 >/dev/null
readonly ACCEPT_RGB_LOG="${TEST_ROOT}/accept-rgb.log"
expect_status 1 env PATH="${FAKE_PATH}" G1_FAKE_LOG="${ACCEPT_RGB_LOG}" \
  "${ACCEPT_SCRIPT}" rgb g1-sh-wifi ubuntu eth0
grep -Fq '8086:0b3a' "${ACCEPT_RGB_LOG}"
grep -Fq '/ubuntu/camera/rgb' "${ACCEPT_RGB_LOG}"
grep -Fq '/home/unitree/.sensor-collector/calibration.json' "${ACCEPT_RGB_LOG}"
if grep -Eq 'mkdir -p|docker (compose|build|load).*(up|down|build|load)' "${ACCEPT_RGB_LOG}"; then
  fail "RGB acceptance emitted a robot write command"
fi
G1_MAP_NAME=sh_n3_smoke PATH="${FAKE_PATH}" G1_FAKE_LOG="${TEST_ROOT}/accept-map.log" \
  "${ACCEPT_SCRIPT}" map g1-sh-wifi ubuntu eth0 >/dev/null
grep -Fq '/tmp/nav_result.json' "${TEST_ROOT}/accept-map.log"
grep -Fq "^POINTS [1-9][0-9]*" "${TEST_ROOT}/accept-map.log"

echo "navigation_shell_tests=PASS"
