# Franka FR3 state driver

One ARM64 container exposes one read-only `joint_states` sensor card, plus the
`model` resource automatically fetched for visualization. Each sensor instance
has its own persistent `robot_ip` setting in the canvas gear menu. Multiple cards
using the same IP share one state reader; different IPs have independent readers
and retry timers. No IP, credentials or installation-specific configuration is
embedded in the image.

| Card | Type | Configuration | Output |
| --- | --- | --- | --- |
| `joint_states` | multi-instance sensor | `robot_ip`, `publish_hz` (1–30, default 10 Hz) | `sensor/skeleton`, seven actual joint angles in radians |
| `model` | resource | none | FR3 visualization URDF |

## Requirements

- ARM64 Ubuntu 22.04 / ROS 2 Humble container environment.
- Supported FR3 robot on a wired, reachable network, with a unique IP per robot.
- FCI installed and activated by the operator in Desk. The driver never takes a
  control token, unlocks brakes, activates FCI, performs recovery or commands motion.
- The image pins libfranka **0.18.1**, FCI protocol 10, for robot system **5.9+**
  per the [official compatibility matrix](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html).
  Firmware 5.10.0 is the initial deployment target. Older protocols require a
  separately validated SDK build; Panda and mixed robot models are not advertised
  as supported by this image.

This release only observes state. It does not validate the host for 1 kHz motion
control. The 10 Hz card setting is a display rate, not a control-loop rate.

## Local build and deployment

The repository build context contains the driver directory plus `common/`.
For a local image without the registry push behavior of `build.sh`:

```bash
# Run from the repository root.
context=$(mktemp -d)
cp -a franka/fr3/. "$context/"
cp -a common "$context/common"
docker build -t franka-fr3:local-review "$context"
# Optional: --build-arg ROS_BASE_IMAGE=ros:humble
```

The Dockerfile defaults to the public TUNA Ubuntu mirror and the official ROS
repository. Override `APT_MIRROR` and `ROS_APT_MIRROR` with `docker build
--build-arg` when needed. The repository's `build.sh --mirror` option currently
selects the Python package mirror only; it does not override these APT sources.

The SDK and its `common` submodule are built from checksum-verified official
source archives included in `vendor/`, following the repository's existing SDK
packaging pattern. Docker builds do not clone or download SDK sources from
GitHub. The archives preserve their upstream licenses and notices; a small
CMake patch supplies the pinned release version for package metadata without
requiring `.git`. Source revisions, checksums and the patch are documented in
[`vendor/SOURCE.md`](vendor/SOURCE.md). Base-image and APT downloads still require
network access.

`deploy/service.yml` follows the resource-center service-fragment convention
(`__IMAGE__` is replaced by the selected image). For standalone local Compose,
wrap that fragment in a top-level `services:` mapping and replace `__IMAGE__`
with `franka-fr3:local-review`. Keep the host networking, domain 42 and
read-only `/opt/phanthy-motus/dds-local.xml` mount; confirm that file exists on
the host before deploying. No privileged mode, robot
device mounts or real-time capabilities are required for this reader.

The driver ID is **franka-fr3-driver** and its MCP port is **15723**. It registers
as **Franka FR3** with Core, which assigns its own MCP record ID. Starting the
container only registers cards; it does not connect to robots until a
`joint_states` card is started.

## Canvas workflow

1. Add a `joint_states` card and set its robot IP with the card gear button.
2. Start the project. The card reports `loading` until a valid sample arrives,
   then `running`. Its info response includes `connected`, `readiness`,
   `data_age_ms` and `error` for diagnosis without a separate status card.
3. In the monitoring dashboard, view the `joints` stream for the FR3 skeleton.
   The renderer fetches the driver's `model` resource automatically; it is not
   a separate sensor card and needs no IP or canvas wiring.
4. Repeat with another IP for another FR3.

Core persists per-card IPs. Its sidebar can also save shared defaults; an
explicit card IP overrides them. Stop a card before changing its IP or frequency.
Deleting/stopping one card releases only its own lease. The last consumer closes
that IP's reader. One disconnected robot does not prevent other robots being read.

Topics use `/franka/<card_id>/state/joints`, following the `state/joints`
suffix used by the RealMan and other robot drivers. The card ID provides an
instance namespace so several arms/cards cannot overwrite each other's poses.
For example, `card-franka-joints` publishes to
`/franka/card_franka_joints/state/joints`. The topic is available before IP
configuration or connection. Hyphens in card IDs become underscores; collisions
are rejected. IP settings are marked `x-sensitive` for Solution export redaction.

The reader rejects malformed/nonfinite samples and stops publishing joint poses
after the 2-second freshness deadline. A renderer may retain its last pose;
check the joint card's info response for freshness and errors instead of
interpreting that pose as a live reading.
If DDS publication fails, the card reports `state: error` and
`publication_error`, retries on the next tick, and leaves other cards running.
Reader connection attempts time out after 5 seconds and retry with bounded
backoff. The SDK runs in a child process per IP so blocked I/O can be cancelled
without blocking other devices or killing the driver.

## Current Core visualization limitation

Core `release.261011.46afd50` passes the placeholder driver ID `detail` from the
canvas's “查看数据流” panel to the skeleton renderer. That panel consequently
cannot fetch this driver's URDF. The monitoring dashboard passes the real driver
ID and is the supported validation path for this release. No Core files are
modified by this driver. Core currently fetches one model per driver, so all
instances in this release use the same FR3 chain. The startup dialog also groups
progress by tool name rather than instance ID; verify each card using its own
IP, info response and topic when using several robots.

## Development checks

```bash
python3 -m pytest tests/test_franka_hardware.py tests/test_franka_driver.py
python3 -m pytest tests/test_dockerfile_copies.py tests/test_common_lifecycle.py
python3 scripts/check_service_yml.py franka/fr3
```

Tests use process and DDS fixtures. Hardware validation must additionally confirm
real FCI samples and DDS reception inside Core; successful HTTP registration is
not sufficient. Only one real arm is available in the initial setup: multi-IP
isolation is tested with fixtures and an unreachable second IP, not claimed as a
multi-arm hardware test.


## Hardware validation scope

Validated on an ARM64 Jetson running Ubuntu 22.04.5, an FR3 with system 5.10.0
and operator-activated FCI, libfranka 0.18.1, and Core
`release.261011.46afd50` (with a separate local fix for the canvas detail panel).
Acceptance covers live DDS samples, matching URDF joint names and units,
same-IP reader sharing, an unavailable second IP, SDK process recovery,
stop/restart cleanup, and both Web visualization entry points. No motion
commands are used in these checks.

Only `joint_states` needs to be placed on the canvas. If the monitoring dashboard
was opened before project startup, reopen it after startup so Core refreshes
the topic formats. Tests cover multiple sessions with fixtures; multiple physical
arms and their network capacity still require hardware validation.
