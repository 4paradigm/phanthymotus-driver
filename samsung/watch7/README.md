# Samsung Galaxy Watch7 — read-only Driver MVP

This is **PR 1**: a standard `phanthymotus-driver` bundle with two Sensor
cards: `heart_rate` and `device_state`. Both publish `std_msgs/String`
JSON on local ROS2 domain 42 using `data/json`:

- `/{ros_namespace}/watch7/heart_rate`
- `/{ros_namespace}/watch7/device_state`

The ROS namespace defaults to the sanitized hostname. Sensor `start` and
`stop` only control the plugin's ROS2 publishing lifecycle; **they do not
start or stop a Watch Health Services measurement**.

## Required companion and ingress

The existing research companion APK sends `POST /v1/telemetry`, `POST
/v2/telemetry` and polls commands. The Driver uses the original v1/v2
protocol validators verbatim and exposes an authenticated watch-only HTTP
endpoint on port **15751**. The existing app can continue to upload data
without Windows Relay or ADB. The first PR intentionally returns empty
`/v1/commands` and `/v2/commands` queues and refuses all execution
and task commands. No vibration, remote sampling, ECG/BIA or autonomous
robot action is exposed by this bundle.

MCP binds to **127.0.0.1:15750**. The Watch ingress binds to 0.0.0.0:15751
by default and requires `X-Wearable-Token`. Set `WATCH7_DEVICE_TOKEN`
to a unique random secret of at least 16 characters in the deployment
environment; the service deliberately **refuses to start** without it.
The secret must be configured manually in the Watch app.

**Security boundary:** the current companion transmits HTTP on Wi-Fi,
not authenticated TLS. Only operate on a controlled, isolated/trusted WLAN
or behind a TLS-protected reverse proxy; do not expose port 15751 to the
internet or an untrusted shared LAN. Before wider release, implement
TLS/pairing and secure token rotation. Do not include secret values in
Solutions, git commits, logs or documentation.

## Build & deploy

Company build entry point (from repository root):

```bash
./build.sh --mirror none samsung/watch7
```

**Warning:** the company's build script may push an image when registry
credentials are present. Do not execute it against a configured registry
merely to perform a local acceptance test. For no-push S5 tests, use the
adjacent project's `scripts/s5_build_local.py` (Docker Buildx `--load`);
the ARM64 build has passed against the company's ROS base image.

Its Driver image contains only Python, ROS2, and the two Card plugins.
There is no Android SDK or Samsung proprietary SDK inside the Docker image.
The host needs the company's `/opt/phanthy-motus/dds-local.xml` loopback
DDS profile, ROS domain 42 and host network mode.

Deploying through the Dashboard uses `deploy/service.yml`; configure
the Watch's URL as `http://<robot-LAN-IP>:15751` and its device token to
match `WATCH7_DEVICE_TOKEN` on the robot. APK v1 and v2 telemetry are
accepted; local advanced Samsung measurement builds remain separate.

The Android companion source is maintained in the adjacent
`phanthymotus_wearable_card/heart-rate-wear-test` project, **not** this Driver PR.
Its **Standard 0.2.0** flavor builds without proprietary Samsung SDK dependencies
and installs alongside the legacy research app; **Research 0.2.0** includes
advanced Samsung Health Sensor SDK features and requires an independently
licensed local AAR. Both debug APK builds and manifest checks passed in S4.
Debug APK signing, HTTP device transport, and unpublished research SDK
license terms are not substitutes for an approved production distribution.

## Offline contract verification

```bash
python3 -m pytest samsung/watch7/tests -q
python3 scripts/check_service_yml.py samsung/watch7
```

No Watch is needed for these checks. True end-to-end verification requires
Watch7 on the same trusted network, an ARM64 Driver running on Linux and
a ROS2 subscriber. A successful stub test does not constitute live-device
acceptance.

## Disabled functionality (separate, reviewed PRs)

- Remote start/stop/configure, haptics, ACP acknowledgements
- Task proposals, consent/revocation, TTS and robots
- ECG/BIA/SpO2/PPG recording manifests and high-rate data
- Access from an untrusted network; automated APK distribution

Failing health permissions, stale samples, off-wrist state, expired sessions,
duplicate sequences and absent device telemetry never generate fake readings.
