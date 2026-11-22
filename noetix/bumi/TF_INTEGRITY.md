# Bumi TF Integrity Monitor

TF Integrity Monitor is a read-only Sensor Card in the Bumi bundle. It subscribes to
`/tf` (volatile, best effort) and `/tf_static` (reliable, transient local).
It publishes `std_msgs/msg/String` containing JSON on
`/<driver_namespace>/tf_integrity_monitor`, advertised as `data/json`.

## Dashboard

Add **TF Integrity Monitor** from the Bumi driver and connect its JSON output to a
**Monitor**. Start the TF source and the sensor. A separate `tf_tree` card may
provide the TF source via ROS, so no canvas wire is needed between them. The sensor
starts idle, and `start` and `stop` are idempotent. `info` returns lifecycle,
output port, and the current diagnostics while running.

## Configuration

Use `plugins.tf_integrity_monitor` in the development driver's `config.yaml`.
List expected names in `required_frames` and directional ancestor-to-descendant
pairs in `required_chains`. The default requires only `base_link`. For production
coverage, enumerate the chains relevant to the task using actual TF names.

`stale_s`, `future_s`, and `skew_s` are seconds. Run replay with
`use_sim_time: true` and `ros2 bag play <bag> --clock`. In simulated time,
paused ROS time pauses freshness aging. Live time uses both stamp age and monotonic
receive age. Static stamps do not age. A backward ROS clock jump clears dynamic
history and increments `clock_resets`. A late, older message within one clock
epoch is reported as a regression and leaves the newer sample in place.

Translation and rotation jump counters require exceeding both the configured
minimum change (meters or radians) and the configured rate (meters or radians
per second). Long gaps reset the comparison baseline. Rotation uses the shortest
quaternion angle. Treat the rate defaults as diagnostic settings and tune them against
the expected Bumi motion and publisher rate.

## Output

- `state` is one of `healthy`, `degraded`, `inconsistent`,
  `insufficient_evidence`, or `stale`. Healthy describes the evidence in the
  monitored topics and configured chains.
- `timestamp` is the ROS clock as integer `sec` and `nanosec`.
- `checked_frames`, `missing_frames`, `stale_frames`, `conflicting_frames`
  and `cyclic_frames` are lists of names. `disconnected_frames` and
  `skewed_chains` list configured pairs.
- `translation_jump_count` and `rotation_jump_count` count observations since
  start. `motion` contains per-edge changes, rates, and consecutive normal
  comparisons. `reason` lists diagnostic codes. `issues` associates codes with
  frame names, and `capacity_drops` records declarations rejected at the capacity cap.
- `visual_residual` is null without valid evidence. `visual_state` reports
  disabled, insufficient evidence, consistent, or inconsistent.

The topology view retains latest declarations from up to 2048 edge/source keys.
It checks frame naming and connectivity without resolving malformed declarations
into a single selected tf2 parent. Humble runtimes that omit subscription message
metadata use an unknown publisher ID. Same-timestamp disagreement, incompatible
parent declarations and static disagreement remain detectable. Different dynamic
stamps on a single edge are compared for continuity, rather than presumed to be
simultaneous measurements.

## Optional visual check

Set `visual_check: true` and provide the following `vision` keys:

| Key | Meaning |
| --- | --- |
| `observation_topic` | ROS `geometry_msgs/msg/PoseStamped` of one identified, fixed ArUco marker in optical coordinates, at capture time |
| `camera_frame` | Actual optical frame in the observation header |
| `surveyed_marker_frame` | Existing static TF frame with independently measured marker pose and matching marker axis convention |
| `calibration_id` | Reference to the measured intrinsics and extrinsics used for this setup |
| `survey_id` | Reference to the physical marker survey |
| `time_sync_id` | Reference to time synchronization evidence |
| `translation_tolerance_m`, `rotation_tolerance_rad` | Residual thresholds, default 0.05 m and 0.15 rad |

Supply the surveyed frame from an independent source and keep it distinct from
any marker frame derived from the visual detector. The configuration identifiers
are operator attestations. Verify them against the calibration, survey and
capture clock before interpreting residuals. The monitor looks up the prediction
at the observation timestamp with tf2. It expresses the observed pose in the
surveyed marker axes and reports absolute residuals and successive observation
changes. Absent evidence, unavailable transforms, and stale or invalid observations
yield `insufficient_evidence`. The #386 JSON output needs a PoseStamped adapter
that preserves capture time and marker identity.

## Development and acceptance

The development branch layers this sensor on driver PR #560 (`76647a06`), which
supplies `vision_tf` and `tf_tree`. Driver #562 proposes model camera frames.
Platform #386 proposes the calibrated ArUco pose processor. The sensor can also
consume TF from existing standard ROS broadcasters.

```sh
python -m pytest -q tests/test_bumi_tf_integrity.py tests/test_bumi_bundle.py
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 python3 tests/bumi_tf_integrity_ros.py
```

The ROS test uses the Bumi MCP HTTP handler, actual TF broadcasters, a JSON
subscriber, and a rosbag2
SQLite fixture read and republished with its recorded clock. Run it on a
separate ROS domain or in a container with networking disabled. On Bumi, validate
normal motion envelopes, measured optical and surveyed marker transforms, replay
from the target recorder, DDS visibility from Dashboard Core, and Monitor rendering.
