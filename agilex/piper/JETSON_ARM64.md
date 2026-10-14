# PiPER ARM64 development on Jetson

The same production Python driver supports amd64 and arm64. The arm64 developer
build keeps the English Arm control interface, background PiPER skeleton and
parameters, optional live camera, fixed base-frame direction actions and the
existing pinned CAN SDK. Core code and firmware are unchanged.

## Build without deployment

```bash
bash agilex/piper/build-arm64.sh agilex-piper:arm64-local
```

This stages the component and common package and builds the production
Dockerfile. It does not start a driver or touch a canvas. If buildx is absent,
the helper permits a native ARM64 Docker build without installing a CLI plugin,
emulator or changing the Docker daemon. A cross-platform build without buildx
fails explicitly. The amd64 helper retains its existing default.

For offline development, after importing the already-built ARM64 dependency
image, rebuild only source layers:

```bash
bash agilex/piper/build-arm64.sh piper-arm64-dev:local \
  -f "$PWD/agilex/piper/Dockerfile.arm64-dev" \
  --build-arg PIPER_SOURCE_REV="$(git rev-parse HEAD)" \
  --memory 1g --cpu-period 100000 --cpu-quota 50000 --network none
```

The default developer base is the ARM64 Bot image release.261014.4f9cd06.
This development overlay is not a new Bot Version or a clean release build.
The normal Dockerfile and SDK commit/SHA256/source are retained; COS publication
remains a separate paused task. No CUDA/GPU dependency is introduced.

## Isolated native verification

Mount a source checkout read-only at /src in a temporary container with
`--network none`, no device mounts, no published ports and CPU/memory limits.
Use `PYTHONPATH=/work:/src/tests:/src` to exercise the built production package
with its recording hardware fixtures. Run the PiPER unittest discovery suite.

For ROS2 transport verification, source /opt/ros/humble/setup.bash and run
`python3 /src/scripts/piper_arm64_smoke.py`. This imports the actual ARM64 SDK,
NumPy/OpenCV and ROS2 libraries, checks a synthetic JPEG roundtrip, and receives
six measured fixture angles in radians and English parameters over actual DDS.
It uses domains 210/211 and namespace piper_arm64_dev, records zero hardware
control commands, and never registers with an existing Core. CAN and the camera
are fake; this is not physical or Jetson camera/USB acceptance.

## Before an operator-approved Docker deployment

- Preserve the running robot/Core containers and their project directories.
  Use a separate PiPER Compose project, service/container names, data/config
  directories and topic namespace. Do not run compose down or remove-orphans
  against another robot's project.
- If the existing Core/canvas belongs to another robot, prepare a separate Core
  instance with its own data, ports and DDS configuration before integration.
  Do not change the other robot's canvas or register the development test there.
- Reidentify only the target slave USB-CAN and color camera on the Jetson;
  adapter indices and aliases from another host are not device identities.
  Verify single-source feedback and color-interface binding before controls.
- Keep PIPER_MOTION_ENABLED=0 for passive checks. Physical movement requires the
  operator's explicit deployment gate and separate motion acceptance. Do not
  treat an ARM64 build or simulated control result as physical acceptance.

Teaching exit now receives the runtime's permission callback and checks it
immediately before every mode/target send, followup hold and recovery send.
Revocation during firmware lookup/stability waiting prevents controls;
revocation after a mode/target submission prevents subsequent targets/recovery.
This correction does not reset, home or release motor torque.

## Development verification — 2026-10-14

- Native aarch64 on Jetson Orin NX, Ubuntu 22.04.5/L4T R36.4.3/Docker 27.5.1:
  225 PiPER unittest tests passed in an isolated development container.
- Native ROS2/DDS smoke received eight skeleton and eight English-parameter
  frames; six joint names/radians, the model resource, SDK import and synthetic
  JPEG encode/decode passed. Recorded hardware commands: zero.
- Mac software/contracts regression: 304 passed, 243 subtests passed, including
  permission revocation during firmware/stability waits, mode/target sends,
  followup/recovery sends and native build-helper fallback/mismatch checks.
- Existing Franka driver and Core image IDs/start times were preserved.
  No PiPER daemon, canvas registration, host ports, hardware mappings or Docker
  Compose deployment were started. Physical acceptance remains pending.

The developer image/source changes are not yet a new PR Bot release. The base
dependency image Version is release.261014.4f9cd06; it must not be reported as a
Bot Version for this newer development source.
