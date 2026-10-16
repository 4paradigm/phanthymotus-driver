# PiPER Arm control, Joints and On camera

The current bundle provides three canvas cards: **Arm control** (`arm_control`),
**Joints** (`joints`) and **On camera** (`piper_camera_live`). Joints and On camera
are independent read-only sensors. They share the existing driver lifecycle and
keep monitoring when the control card stops. No Parameters or
`state/parameters` stream is published; **Get status** returns feedback, limits,
motor state, faults, capabilities and actual task outcomes.

| Action | Input and meaning |
| --- | --- |
| Move relative | `translation_mm=[dx,dy,dz]`, along the initial measured SDK end axes; pure translation while TCP is uncalibrated |
| Move to pose | Calibrated-only `pose_mm_deg=[x,y,z,roll,pitch,yaw]`, absolute base-frame tip target |
| Set joints | Six `joints_deg`, with absolute or relative degree mode |
| Reset | Configured `intent_control.reset_joints_deg`; otherwise the official six-zero target; joint trajectory, gripper unchanged |
| Set gripper | `set_gripper_opening`, `width_mm`, effective SDK/deployment bounds, normally 0–70 mm, default 20 mm adjusted to bounds |
| Stop | Cancel current preparation/movement and attempt a measured hold; retain motor torque |
| Get status | Passive actual feedback and per-action task/completion-delivery records |

Absolute arrays permit `null` to retain a fresh measured component and require
at least one numeric target. Relative arrays require numbers; use 0 for no
increment. Zero relative requests complete without preparation or movement.
Default speed is 1%, permitted range 1–10%; relative Move is limited to a total
10 mm translation. Inputs may be native JSON arrays or JSON-array text.
Set position, Open gripper and Close gripper are removed; old requests return an
interface-refresh error. Preserved legacy saved poses are not used for Reset.

Move remains linear. Absolute tip Pose/rotation stay hidden and rejected until
the installed TCP is physically calibrated and verified. Candidate point-to-point
stays closed until device/firmware/source/configuration-bound acceptance passes.
See [UNIFIED_CONTROL.md](UNIFIED_CONTROL.md) and
[PATH_VALIDATION.md](PATH_VALIDATION.md) for the complete contract.

Every nonzero control request uses guarded stationary-teaching exit and measured
hold before target resolution. Freshness, faults, limits, mode, enabling,
cancellation and one-task-at-a-time checks apply throughout preparation and
execution. Target rejection does not automatically clear faults, retry or change
path. Move, Pose, Set joints and Reset wait at most **60 seconds** after target
submission for actual stable arrival. Low-speed large moves can still exceed this
limit. Reset is a trajectory, not firmware reset, zero calibration or error
clearing; its target and source appear in Get status. Physical return paths and
stopping latency require separate supervised acceptance.

## Monitoring and ACP

Joints publishes the official PiPER chain at 10 Hz, with measured radians and
stale-feedback freezing. `model` returns `{"urdf":"..."}`. Finger translation is
not animated by the current Core; real opening width is available in Get status.
On camera publishes the existing JPEG topic when its configured live stream and
camera permission are enabled. Optional photo capture/import remains available.

Long-running requests return a unique `action_id`; their real completed, error
or cancelled result is POSTed to Core. Receipt timeout is 90 seconds, allowing
preparation and delivery around the 60-second movement wait. Delivery checks
`ok` and matching ID, retries at most four times and records Core receipt state.
Delivery retries never repeat hardware commands; older deliveries cannot replace
newer task outcomes. Fast actions return their terminal result synchronously.
Records are bounded and in memory, not durable across driver replacement.

[Core PR #330](https://github.com/4paradigm/phanthymotus/pull/330) supplies correct
model routing, early-receipt buffering, manual-result updates and activity
presentation. The driver remains independently buildable. Use a Core containing
those fixes for the complete canvas experience. A `trigger` activity label is
presentation; Get status is a synchronous query and its completion does not
change the previous movement result.

## Build and packaging

```bash
# Native/cross ARM64 build; does not start a service or contact hardware.
bash agilex/piper/build-arm64.sh agilex-piper:arm64-local
# Existing amd64 build remains available.
bash agilex/piper/build-local.sh agilex-piper:amd64-local
```

The ARM64 production base reuses the project's pinned ROS base. The amd64 helper
selects the pinned official multi-architecture Humble base. Existing ROS/APT
packages are reused; explicit build assertions/fallback installation prevent
silently assuming dependencies exist. See [IMAGE_ACCOUNTING.md](IMAGE_ACCOUNTING.md)
for exact base digests, dependency inventory, image-layer sizes and reproducible
measurement commands. `Dockerfile.arm64-dev` is a digest-pinned operator-only
source overlay; it is not the clean production release build.

PiPER SDK commit `8d4dc29bd6e753066dedee235c008e90ecb2ab4a` and SHA-256
`89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29`
remain fixed. **SDK COS publication is deferred at the operator's request**;
[SDK_ARTIFACT.md](SDK_ARTIFACT.md) records that remaining merge finding. No ZIP,
credential or site configuration is committed.

## Device access and optional camera

`deploy/service.yml` uses host networking for host SocketCAN and loopback DDS,
with all Linux capabilities dropped and `no-new-privileges`. It grants no host
`/dev` mount, privileged mode, device wildcard or NET_ADMIN. Configure and bring up
the CAN interface on the host before starting the driver. The standard fragment
keeps camera permission at 0 and needs no camera node to exist.

For camera-enabled deployments, merge `deploy/camera.compose.yml` into the site's
Compose configuration and set `PIPER_CAMERA_DEVICE` to the explicitly bound color
V4L2 path, matching `camera.device` in the driver configuration. For example,
a site already configured for `/dev/piper-camera-2` can use:

```bash
PIPER_CAMERA_DEVICE=/dev/piper-camera-2 docker compose \
  -f /opt/phanthy-motus/docker-compose.yml \
  -f /path/to/piper/deploy/camera.compose.yml config
```

This command only renders the configuration. Apply the reviewed configuration
through the site's normal deployment process. The override enables camera access
and maps exactly that selected path with read/write device permission. It does
not expose USB buses, other cameras or host device directories. Keep the color
serial/interface binding; generic `video-index0` may select a depth interface.
After re-enumeration to a new device number, recreate only the PiPER service to
refresh the device mapping. Broad hotplug access is deliberately not granted.

Image-only upgrades preserve old Compose grants. When adopting this fragment,
remove old `privileged` and `/dev:/dev` entries from the site service as well.
Existing deployments are not rewritten by a source/PR update.

On the ARM64 Jetson, an isolated container with all capabilities dropped received
a CAN frame without sending one. A second isolated container with only the exact
camera node opened it and completed V4L2 QUERYCAP without starting capture. CAN TX
was unchanged. These passive probes establish minimum access; they do not certify
camera streaming/replug, physical motion or full deployment acceptance.

## Verification and operational records

```bash
python -m pytest -q tests/test_piper_deployment.py tests/test_piper_arm64_build.py
python scripts/check_service_yml.py agilex/piper
python3 agilex/piper/deploy/diagnose.py --output readonly.json
```

The passive diagnostic reads current status/model/camera metadata; it does not
start, reconnect, prepare or move the robot. Optional photos require an explicit
request. Keep physical trajectory and Stop-latency acceptance separate from
simulated tests, image builds and passive deployment checks. Current motion
behavior, firmware limits and hardware acceptance remain documented in
[UNIFIED_CONTROL.md](UNIFIED_CONTROL.md) and the existing acceptance records.

Earlier ten-card menus, Set position and Parameters instructions are preserved
in [LEGACY_OPERATIONS.md](LEGACY_OPERATIONS.md). They are not the current public
interface. Existing legacy code and its regression coverage are retained.
