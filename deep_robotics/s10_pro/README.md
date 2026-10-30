# DEEP Robotics S10 Pro

One Driver beside the standard Agent Core. Public tools are `battery`, `joints`,
`model`, `lidar_cloud`, `imu`, `device`, `loco_move` and `relative_move`.
The device-management placeholder sends no hardware commands.

## Motion and canvas lifecycle

`loco_move` supports `stand_up`, `sit_down`, finite `move` and `stop_move`.
Saved `sitdown` / `stand_down` calls remain aliases. `relative_move` uses odometry
to reach a distance or angle. Both tools share one controller and `x-resource: base`;
a second action is rejected as busy while the first is running.

- `start` activates read-only feedback and returns `{"state":"ready"}`.
  This is a lifecycle marker, not proof that a specific movement is available.
- `stop` cancels the current action, returns `{"state":"idle"}` immediately and
  releases observation once the action worker finishes stopping.
- `info`/`status` read current blockers. Inspect `readiness.actions.move` for
  movement, separately from posture readiness. Starting a card sends no command.
- `stand_up` requests state 1 only when idle, confirms standing, then requests
  move mode 17. An already-ready state 17 needs no posture publication.
- `sit_down` requests state 4 and waits for final idle state 0.
- `move` sends `/NAV_CMD` at 10 Hz for 0.5–5 seconds, with configured speed caps.
  It must observe velocity in the requested direction and confirm stopping.
- Relative movement accepts 0.3–0.6 m or 30/60 degrees. Odometry controls progress,
  settling and drift/stall detection. Translation is capped at 0.12 m/s and
  turning at 0.20 rad/s.

Motion requires `motion.enabled: true` in config plus `S10_MOTION_ENABLED=1`
(default). Setting the host variable to `0` disables motion even if canvas
settings enable it. Sensors remain available. An old
`S10_EXECUTION_AUTHORIZATION_FILE` disables motion with a warning instead of
crashing/restarting the entire Driver. Legacy Prepare policies, identity grants,
manual unlocks and automatic permission renewal are no longer used.

Every action checks fresh motion, joint, fault and emergency-stop feedback.
Move additionally requires stationary state 17. Velocity actions reserve a
bounded stop window and send repeated zero frames, then require fresh stationary
feedback. Small residual velocity can use independently stationary odometry;
see [stationary confirmation](deploy/STOP_DIAGNOSTICS.md).
Application cancellation is not a hardware emergency stop. Failed tasks are not
automatically replayed; later requests use current conditions.

## ACP

The implementation follows [README_dev — ACP](../../README_dev.md#action-completion-protocol-acp):

1. Long-running motion transitions declare `x-completion` and `x-resource: base`.
2. Dispatch accepts the action with a unique `action_id`; it does not wait for
   physical completion. Core owns the pending-action barrier.
3. The action worker posts one terminal `completed`, `error` or `cancelled`
   payload to `${AGENT_CORE_URL}/api/acp/complete`, including `action_id`, `result`,
   `tool` and `ts`, after the action and required stopping checks.
4. Local Core HTTPS accepts the platform self-signed certificate as in the
   README example. Non-loopback HTTPS still validates its CA and hostname.
   `AGENT_CORE_URL` defaults to `https://localhost:15678`.

There is no custom receipt handshake, SQLite outbox, manual completion trigger,
SSE endpoint or driver callback retry loop. A failed POST is logged; Core's
20-second completion timeout is the fallback. No special CA/pin/credential mount
is required by this callback. Existing Core authentication/role configuration
remains platform-owned.

Request IDs are deduplicated in bounded in-memory history. A terminal replay
returns its saved result without `action_id`, so Core does not register another
pending action. History does not survive a Driver restart, and interrupted
motions are never replayed. Existing site databases are not deleted or migrated.

## Deployment and DDS

Fast DDS uses process-global profiles. The platform process stays on domain 42
with `/opt/phanthy-motus/dds-local.xml` restricted to loopback. The read-only
reader and command worker use separate domain-0 processes with the vendor UDP
profile; they never inherit the platform profile. Preserve this separation.

Before creating the Driver, install the shipped `/deploy/s10-reader-udp.xml`
as the read-only host mount `/opt/phanthy-motus/s10-reader-udp.xml` and provide
host `config.yaml`. Both mounts must be files, not auto-created directories.
Use `deploy/service.yml`; retain the existing Core service and site paths.
The image contains generated `drdds` interfaces from licensed text sources;
message generators stay in the build stage. No Core code is patched.

The Driver starts with cards idle. Follow the canvas lifecycle to start
observation; no startup posture or velocity is sent. During upgrades preserve
old containers/images and current site configuration for rollback. Removal of
source-side policies/outboxes does not authorize deleting their site backups.

## Local verification

Use Python 3.12+ outside the runtime image:

```bash
python3 -m pip install -r scripts/s10-review-requirements.txt
python3 -m pytest -q tests/test_s10_motion.py
python3 scripts/check_s10_release.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
python3 deep_robotics/s10_pro/deploy/check_container_contract.py
```

The compact motion tests use fake telemetry/transport and a local self-signed
HTTPS server. They cover gates, posture, motion, cancellation, stopping,
relative progress, message encoding and ACP; they never connect to a robot.
Passing these checks does not establish a full ARM64 build or hardware acceptance.
