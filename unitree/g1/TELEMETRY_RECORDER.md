# G1 telemetry recorder (experimental, opt-in)

`telemetry_recorder` consumes existing Motus ROS2 `std_msgs/String` JSON topics.
It never creates a Unitree control client or publishes robot commands. It is
disabled by default; no additional Python dependencies are required.

## Enable and configure

Merge the `plugins.telemetry_recorder` block into the deployed config (respect
`CONFIG_PATH`). Confirm topic mapping, one writer per output directory, actual
storage mount/capacity, and overlap with any existing recorder before enabling.

```yaml
plugins:
  telemetry_recorder:
    enabled: true
    output_dir: /opt/phanthy-motus/data/telemetry_recorder
    buffer_s: 30
    pre_s: 10
    post_s: 5
    max_buffer_bytes: 4000000
    max_clip_bytes: 4000000
    max_message_bytes: 65536
    max_samples: 10000
    max_pending: 2
    max_disk_bytes: 64000000
    max_clips: 100
    auto_events: []
    cooldown_s: 10
```

Defaults subscribe to `/<resolved namespace>/state/imu`, `state/joints`,
`state/battery`, and `safety/motion_events`. These are Motus relay topics, not
the original Unitree DDS names. Override with `imu_topic`, `joints_topic`,
`battery_topic`, `motion_events_topic`. No canvas input rewiring is implemented.
The existing state/event publishers must be enabled independently.

Freshness thresholds are `imu_stale_s=.5`, `joints_stale_s=.5`,
`battery_stale_s=3`. Event streams have no periodic-heartbeat assumption. These
are configurable receiving-data thresholds, not robot safety thresholds.

`auto_events` is empty by default. Confirmed event names in the current
SmartMotion implementation include `safety_stop`, `safety_stop_failed`,
`motion_stop_failed`, and `joint_overheat`; configure only the events needed.

## Actions and state

- `capture(label="manual", pre_s=10, post_s=5)` returns `capturing` and a
  `clip_id` immediately. It is not an ACP completion notification.
- `get_clip(clip_id)` checks `capturing`, `writing`, `saved`, or `failed`.
- `list_clips(limit=10)` lists at most 20 saved metadata entries.
- `status`/`info` reports actual per-source receive ages, pending tasks and the
  last storage/worker error. Lifecycle `ready` does not establish stream health.
- `start`/`stop` implement platform lifecycle. Stop flushes incomplete clips
  with `interrupted=true`; a slow writer blocks restart until it has stopped.
- Stop before changing card config. YAML exposes all bounds; the card's
  configSchema exposes the common topic/time/storage options.

Each saved directory contains `clip.json` (`g1.telemetry_clip/1`) and a
self-contained `report.html`. Paths are local to the driver host; no network
upload, front-end download or automatic retention deletion is implemented.
Quota or write failure is explicit and not automatically retried.

## Interpretation and validation

Alignment uses receiving-process monotonic time, plus receive UTC for humans.
Original payloads are retained without inferred clock synchronization or unit
conversion. `post_window_elapsed` only establishes elapsed wall duration;
inspect source coverage, gaps, truncation and interruption independently.
Ring evictions/rejections in `receiver_totals_at_export` are process cumulative.

The current IMU relay is hip IMU only. Motor-array slots are not asserted to be
actual robot DoF. No physical fault diagnosis, safety certification, action
replay or motion intervention is provided. Budgeted payload bytes are not an
RSS measurement. Real ROS transport, Agent Core rendering, disk performance,
power-loss durability, firmware/units/joint mapping and robot load remain
hardware acceptance items.

Offline, without ROS/SDK initialization:

```bash
python3 -m unittest discover -s unitree/g1/tests -p test_telemetry_recorder.py -v
python3 unitree/g1/demo_telemetry_recorder.py --output-dir ./recorder-demo
```

The demo generates simulated IMU/joints/battery/event data and a deliberate
receive gap. Its output is explicitly marked simulated and is not a hardware
test. Tests can use `G1_RECORDER_TEST_TMP` to choose a writable temporary root.
