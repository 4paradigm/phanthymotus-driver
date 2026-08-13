# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus one higher-level motion-state card backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

## New cards

### `motion_state`

Passive, read-only whole-body motion telemetry from `HighController`. The card has no action parameters or execute button. Its single JSON output combines the former summary and joint views:

- current activity, workmode, protection flag, body orientation, angular velocity, linear acceleration and whole-body joint activity statistics;
- active motor faults whose codes are documented by Noetix;
- position, velocity, torque, temperature and raw error value for all 21 joints.
- ROS2 output: `/<namespace>/motion/state`, JSON.

The default polling and topic publication rate is 2 Hz (`poll_interval_s: 0.5`).
Motion activity uses a configurable joint-speed threshold, defaulting to
`0.15 rad/s`. Only motor error codes explicitly documented by Noetix are
classified as faults. Undocumented non-zero raw values are not shown in the
fault summary and remain available only in each joint's raw `error` field for device-side verification.

Every published state identifies `Noetix HighController/CycloneDDS` as its source and includes a freshness flag. It deliberately excludes battery data, which belongs to the existing `battery` card. The SDK does not expose world-frame position or translational velocity, so the card reports only documented IMU and joint measurements and does not invent odometry.

## `switch_mode` behavior

The card follows the vendor's documented main transition chain:
`disabled -> enabled -> ready -> walking`. It validates the immediate
prerequisite but never executes missing transitions automatically. A rejected
request returns `required_sequence` so the user can check robot pose, ground,
support and clearance before executing each step. `fall_to_stand` starts from
ready; `stand_to_fall`, gestures, dances and teach entry/play start from walking.

`enable` and `disable` are separate idempotent `mode` values even though the
SDK exposes only the state-dependent `START` toggle. `enable` sends `START`
only from workmode 30; `disable` sends it only from a non-disabled mode. The
card waits for workmode feedback and reports `completed` only after
confirmation. The unavailable `RUN` command and deprecated `ENDTEACH` command
are not exposed.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
