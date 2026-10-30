# Current running-driver source

Infrastructure rationale, the pinned ROS base and image-size measurement
requirements are documented in [build reproducibility](deploy/BUILD_REPRODUCIBILITY.md).

This revision imports the running Docker driver on 2026-10-30, including automatic
preparation, the hidden manual Prepare entry, bounded/relative movement, and the
read-only joint skeleton. Public monitor cards are battery, joints, lidar_cloud
and imu, plus device availability. The historical sections below describe earlier
versions. See [automatic preparation](README_AUTOMATIC_PREPARE.md) for current
startup behavior. Existing PR deployment hardening is retained.

# DEEP Robotics S10 Pro Driver

S10 Pro is one driver bundle with one MCP endpoint at `127.0.0.1:15745/mcp`.
The `phanthy-motus-agent-core-1` uses the standard platform container name. Its target
implementation is the shared platform Core, without S10-specific image layers
or source patches; use the approved standard baseline for comparison. The target application
has exactly that Core and one `embodied-deep-robotics-s10-pro` driver container.
This local candidate has not been built or deployed to the robot. Existing
site storage, configuration, certificates and canvas data require a separate
local inventory before any future migration; do not replace them with defaults.

## Pinned G1 development reference

The reference is repository commit
`4ccfdebf938e69f98c40a84554c538224a4a6859`, with `unitree/g1` tree
`af218e219107d4b951950f54ff30658a9afd75ec`. The active S10 checkout starts at
`7385f170922853988e5ccfce14a1ef582a12bdfa` and contains uncommitted work.
[The reference lock](deploy/G1_REFERENCE.json) records hashes of the reviewed
G1 files. This pins the available local reference; it is not a claim about the
latest remote branch. The G1 reference itself remains unchanged.

Follow `README_dev.md` and the pinned G1 implementations of `main.py`,
`Dockerfile`, `deploy/service.yml`, `loco_relative_move.py` and
`relative_feedback.py`: aggregate capabilities in one driver, use `/mcp`, return
plain dictionaries from dispatch, implement start/stop explicitly, declare
per-action parameters and completion contracts, and isolate platform DDS on
domain 42. Driver code, comments, tools and documentation use English.

G1 contains Unitree-specific SDK, privilege, IPC and motion behavior. Do not copy
those into S10. S10 uses Jazzy/Python 3.12 and its own verified vendor message
contracts. Retain the S10 transport and conservative motion limits. The pinned
G1 service and the development guide require host networking for loopback DDS;
retain it despite an older conflicting note about injected network settings.

## Container packaging

`deploy/service.yml` is a **single service fragment**, as in pinned G1. It is
not a standalone Compose file: the platform installs it below `services` in the
existing application document after replacing `__IMAGE__`. Never pass this
fragment or `service.supervised.yml` directly to `docker compose -f`.
The supervised file is an overlay for the same driver service, not another
container. Both definitions must be merged into that one service.

The image copies `common/` into `/work/common/` and `deploy/` into `/deploy/`.
The latter lets the existing Core installer extract the service definition.
Local log rotation uses `local`, 10 MB and three files; `restart: always` follows
the pinned G1 service. These changes affect the driver only.

The build context is the S10 directory plus `common/`, as specified by
`driver.yaml` build_context_extras. To stage it and build a local ARM64 image:

```bash
mkdir -p work/s10-container-context
cp -R deep_robotics/s10_pro/. work/s10-container-context/
cp -R common work/s10-container-context/common
# Run only when a local Docker engine is available. No push is requested.
docker build --platform linux/arm64 -t s10-pro:local work/s10-container-context
```

Do not use the repository build.sh for a local-only run with unknown registry
settings: it can push and register images when credentials are configured.
Do not copy host ROS libraries into a container using another ROS distribution.

## DDS and persistent state

The platform parent uses domain 42 and the existing loopback-only
`/opt/phanthy-motus/dds-local.xml`. Independent reader and command children use
vendor domain 0 and `deploy/s10-reader-udp.xml`. Set external configuration:

```yaml
reader:
  domain_id: 0
  vendor_profile: /opt/phanthy-motus/s10-reader-udp.xml
```

The service fragment mounts both XML files and the external driver config
read-only. Check that mount sources are real files before deployment; Docker
can create a directory for a missing source. Do not change vendor host profiles
or run the vendor setup script as a connectivity probe.

`/work/state` is driver-only persistent storage for `motion-tasks.sqlite3` and
`acp-outbox.sqlite3`. Preserve it across driver replacement. It must never mount
the Core database. Reboot does not resume physical tasks or operator sessions.
Vendor sources and message provenance are recorded in `vendor/SOURCE.md`;
keep their upstream license files and verify field compatibility separately.

## Cards and motion boundaries

The manifest lists state, joints, IMU, odometry, LiDAR, motion feedback, device
health, hardware e-stop, errors, motion events, capabilities, ROS graph and
restricted device-management cards. The configured supervised mode adds
`motion`, `timed_move` and `relative_move` through the same driver bundle.
Cards are idle at process start; start only enables their defined observations.

Default output paths are `/{namespace}/s10_pro/{card}`, with separate
`posture_status|posture_events`, `timed_move_status|timed_move_events` and
`relative_move_status|relative_move_events` outputs. LiDAR keeps its point-cloud
port and a separate JSON metadata port. Preserve raw timestamps and unknown
units. Freshness uses monotonic receive time; missing and stale data must not
appear live. Stopping a sensor releases only its own leases.

Motion is disabled by default. Explicit deployment policy and a successful
prepare operation are required for a supervised session of up to 60 minutes.
A card or DDS subscriber does not establish vendor control ownership.
`stand` requests state 1, `lie` requests state 4 and waits for final state 0,
and `move_mode` requests state 17 from stationary state 1. To leave state 17,
use lie before stand. Navigation mode on the vendor remote remains a site
prerequisite; it is not inferred from feedback.

Timed movement supports forward/backward and in-place left/right turns for
0.5-5 seconds. Translation limits are 0.06-0.12 m/s; turns use 0.08-0.20 rad/s.
Completion requires direction-consistent feedback and confirmed stopping.
Speed multiplied by duration is not measured displacement.

Relative movement uses /ODOM for one axis per task: translation 0.3-0.6 m or
30/60-degree turns. Translation limits are 0.06-0.12 m/s; turns use
0.12-0.20 rad/s. The schema speed default is 0.12, valid for either direction
class. Position/yaw tolerances are 8 cm/5 degrees, with settling and a bounded
stop budget. The controller decelerates and stops on excessive drift; it does
not provide complete lateral or heading correction or navigation. Stale,
invalid, jumping or inconsistent feedback fails closed. Physical relative
movement and lateral motion have not been accepted for this candidate.

All motion cards share authorization, mutual exclusion and persisted request
identity. Duplicate IDs cannot create a second physical task. Completion is
reported through ACP only after the relevant feedback conditions; publishing
DDS alone does not prove completion. Cancellation is application stopping,
not a hardware e-stop or an independently verified robot watchdog.

## ACP and authorization

ACP uses `AGENT_CORE_URL=https://localhost:15678`. Use system trust or
`S10_ACP_CA_FILE`, or a previously verified single leaf certificate through
`S10_ACP_PINNED_LEAF_FILE`; do not configure both custom trust modes.
Never disable TLS verification or follow redirects with credentials.
Read `ACCESS_TOKEN` from protected environment or the read-only file selected
by `S10_ACP_ENV_FILE`; never store credentials in the image, repository or logs.

[Motion setup](deploy/MOTION_CARDS.md), [runtime binding](deploy/CONTROL_RUNTIME.md)
and [operator test boundaries](deploy/OPERATOR_TEST.md) document the separate
authorization paths. Do not enable conflicting execution modes. Pending ACP
records retain their original IDs and finite delivery window. An undelivered
completion is not acknowledged or physically successful.

## Local checks and remaining work

```bash
python3 -m pytest -q tests/test_s10_*.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
python3 deep_robotics/s10_pro/deploy/check_container_contract.py
```

The container checker validates the install fragment by default. When local,
fully resolved standard-baseline and candidate Compose documents are available, run:

```bash
python3 deep_robotics/s10_pro/deploy/check_container_contract.py \
  --baseline /path/to/original-compose.yml --candidate /path/to/candidate-compose.yml
```

It rejects a third service, a renamed Core or driver, changes from the approved standard Core service,
shared project-resource changes and replica counts other than one. It reads
files only and never starts, stops or removes a container. Synthetic test
fixtures are not proof about a deployed Compose file or running containers.

The historical separate monitor container depends on `overview.py` and
`groups.py`, which are absent locally. Its four existing aggregate cards have
not been ported or verified in this candidate. Do not add a monitor container
to bypass the two-container constraint. Obtain those sources and preserve their
tool IDs, schemas, topics and lifecycle inside the one driver before migrating
the existing canvas. The standard Core source is copied from the pinned shared platform commit;
this driver does not patch it. This work does not edit the robot or its canvas.

See [acceptance](deploy/ACCEPTANCE.md) for evidence requirements. Historical
robot observations and older test totals do not validate the present candidate.
