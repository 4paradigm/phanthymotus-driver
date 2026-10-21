# PR #408 / #425 integration contract

Audited remote heads on 2026-10-21: #408 `7d5b2673a4ca9e384d56656f74a4046d9c79bb3d`,
#425 `c48692f81ace94bd5e73d3986cf01422c74f68d2`; common ancestor
`40ae7fe80382f02085cb9e7bf55133cdc2c62151`. Neither local checkout was assumed current.
The original checkouts (including the directions worktree and platform repository)
were clean and have not been edited. Work occurs in isolated detached worktrees.

## Ownership and independent use

| Owner | Runtime responsibility |
| --- | --- |
| #408 | Single supervised `pyrealsense2` owner, media cache, raw RGB-D publisher, body IMU bridge, saved-binding consumer, both distance controllers, rotation/arbitration/emergency stop/ACP |
| #408 shared prerequisite | `common/fusion_odometry.py`: ROS-free numeric/clock/geometry/quality contract, originally introduced in #425; identical shared implementation, no second estimator or ROS installation |
| #425 | Independent `rgbd_imu_odometry` MCP sensor; RTAB-Map/EKF processes, calibration/initialization/segment management, separate Docker image and deployment, recordings/evaluation |
| Platform repository | Authoritative saved-canvas binding issuer and renewal/confirmation; delivered here as patch files only |

#408 defaults to standalone IMU distance control without #425, a ROS fusion
process or the platform extension. It retains four directions and the control-loop
IMU trace. The starting stationary window and estimated-distance caveats still
apply; the existing IMU conventions are assumptions awaiting measurement.
`plugins.loco.require_feedback_binding: true` remains an optional stricter mode.

#425 can run and publish as an independent read-only sensor without any destination
card or motion request. It consumes the shared Bumi inputs (included on its branch)
and never calls a chassis command or opens a camera. Unconfigured calibration
keeps it initializing. Its stop/reset/crash cannot stop the Bumi camera or audio.

## Exact interfaces

- One Bumi camera pipeline produces the existing JPEG/Z16 compressed topics and
  photo/video cache. The optional raw branch publishes `/<namespace>/odometry_input/`
  `rgb` (`bgr8`), `depth` (aligned **32FC1 metres**, SDK depth scale), `info`
  (`CameraInfo`) and `camera_meta` (`String` JSON). Raw RGB and depth retain their
  distinct device timestamps/domain/frame numbers, mapping metadata and epoch.
  Raw-branch errors are isolated from media; no RealSense ROS driver is started.
- `BodyImuBridge` publishes `odometry_input/imu` JSON with raw orientation/gyro/
  acceleration, read-begin/end wall times, sequence and epoch. `source_stamp_ns`
  is **null**, `stamp_source` is **host_read**; SDK reads are not source sampling
  timestamps. Source freshness cannot be proved from receipt/read times.
- #425 outputs `/bumi_rgbd_imu_odometry/state/odom`: `std_msgs/msg/String`, format
  `state/odom`, schema `motus.odom/1`, body twist in SI, pose `odom -> base_link`.
  Atomic `vendor` status is `initializing/tracking/lost`, with `segment_id`,
  `calibration_id`, publication time and matched visual-evidence time. The older
  motion-state gyro-only `state/odom` output is **not** a fusion source.
- `loco_target` declares one `state/odom` input and `x-feedback-binding: 1`.
  Platform bindings carry version, source/edge-derived revision, resolved flag,
  input topic, issued/expiry nanoseconds. Empty saved edges explicitly select
  IMU; unresolved/multiple/stale edges reject. Live source declaration must match
  the selected port, format and topic. Discovery alone never selects feedback.
- `start/config` applies the authoritative binding; the ready response echoes the
  complete binding and actual input topic. Same-revision renewal preserves the
  subscription; replacements invalidate its callback generation and action.
  Once any binding is received, including empty, its lease is mandatory.
  A legacy connected-topic request without binding is rejected and cannot then
  start an unbound IMU action. Restart the driver to return to unmanaged mode.
- Acceptance freezes feedback source, revision, segment, calibration and a copy of
  the relative origin. Fusion progress projects onto the selected starting body
  axis while holding starting yaw. No connected-but-unready fallback exists.
  Lost/stale/rewired/reset/expired feedback brakes, errors and finishes ACP once.
  Expired-then-renewed leases cannot rescue an old action even if its watchdog
  was delayed; old watchdog contexts cannot cancel replacement actions.

No installed transform, gyro unit, sensor-to-world quaternion direction, gravity
convention or host-read timing qualification is inferred as calibrated. #425's
Calibration and stationary/visual initialization gates remain unchanged. Tests
use explicit synthetic calibration fixtures, not production defaults.

## Platform patch (separate ownership)

The original #425 platform patch targets platform commit
`6df601f612818716bb0dc7b180a9467e2c43172c`. For the original #425 `c48692f8` artifact,
apply its `noetix/bumi_odometry/platform/saved-feedback-binding.patch` first, then
[this supplemental patch](platform/lease-confirmation.patch). The overlay here
is the resulting graph module and is tested directly. Platform files in other
workspaces have not been modified or deployed.

The supplement makes both start and config resolve the saved graph server-side,
requires the exact accepted lease and topic in the ready ACK (revision alone is
insufficient), rejects missing/changed live sensor ports, and requests stop on a
failed ACK. The graph heartbeat runs every 100ms with 500ms leases, dispatch timeout
200ms; the Bumi watchdog checks every 20ms. These are software budgets, not measured
hardware stopping latencies. If stop delivery also fails, the accepted lease must
expire; repeated failed ACKs issue stop rather than silently prolonging motion.

The synchronized #425 branch incorporates these changes into its full platform
patch/overlay. Use that full patch **alone**, rather than applying this supplement
again. Full browser/backend/remote-peer integration remains unverified.

## Integration and merge order

1. Merge #408 first. It supplies the current shared adapters and retains standalone
   IMU/directions/traces and the official Ubuntu archive. No fusion ROS packages
   are added to the Bumi image; only two small Python modules are copied and the
   import-only check is extended.
2. Synchronize #408's compatibility commit into #425, then merge #425 second.
   Do not cherry-pick #425 wholesale into #408. Its historical inherited changes
   must not overwrite the current consumer, direction schema, traces or mirror.
   The isolated integration merge keeps #408 versions for shared paths and #425
   versions for the independent sensor paths; add/add test/adapter conflicts
   need the #408 superset, not a blind automatic merge.
3. Deploy the reviewed full platform patch, refresh tool declarations and save
   the real sensor-to-destination connection before attempting fusion motion.
   Deploy and calibrate the independent sensor; tracking is required to move.
   #408 standalone IMU can be used without this step, with its stated limits.

Both PRs remain separate drafts. No PR is merged by this work, no force push is
used, and no platform checkout is changed. See [validation](VALIDATION.md) for
commands, limitations and actual results; Git mergeability is not runtime proof.
