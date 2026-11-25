# Spatial SLAM Input Contract (v1)

This is the **cross-robot** contract for the generic Perception
`pointcloud_slam` card. The contract is based on observable sensor facts, not
robot names. The normative ROS-free checks are in
[`common/spatial_slam.py`](../common/spatial_slam.py); the separately deployed
Perception image mirrors those checks in
`perception/plugins/pointcloud_slam_contract.py`.

This contract specifies the **producer's assertions**. Passing structural
validation does not prove geometric calibration or timing synchronization.
Producers must never invent a transform, pose, clock proof, or capability just
to make an input pass.

## Wire types and capability

Exactly two connected native ROS 2 topics are used by the current card:

| Port format | ROS message | Contents |
| --- | --- | --- |
| `state/odom` | `std_msgs/msg/String` | JSON `motus.odom/1` with `vendor.motus_slam` |
| `sensor/pointcloud2` | `sensor_msgs/msg/PointCloud2` | Original metric XYZ scan or depth cloud |

`sensor/pointcloud` is an `UInt8MultiArray` **dashboard rendering format**,
not a native PointCloud2 input; `sensor_msgs/msg/LaserScan` requires a
separate adapter. Do not relabel a visualization packet as PointCloud2.

A state card that actually supports SLAM announces, in `info()`:

```json
{
  "odom_interface": {
    "schema": "motus.odom/1",
    "frame": "body",
    "provides": [],
    "rate_hz": 20,
    "pose_drift": "unbounded",
    "slam_pose": {"available": true, "profile": "motus_slam/1"}
  }
}
```

`provides` is the available **twist** axes; it may be empty for a
pose-only sensor. An ordinary `motus.odom/1` sample, a valid velocity, or a
`pose_drift` hint alone does **not** imply the SLAM pose capability.

For each tracking reading, `motus.odom/1` must carry positive integer
`stamp_ms` (the measurement's time in a documented clock), `units.length=m`,
`pose.position` (finite 3-vector, meters), normalized xyzw quaternion,
`pose.frame_id`, `pose.child_frame_id`, covariance (36 finite row-major
values or `null`), plus `vendor.motus_slam`:

| Key | Requirement |
| --- | --- |
| `status` | `tracking` for a usable pose |
| `segment_id` | Nonempty identifier; change it on odometry restart/reset |
| `stamp_source` | `measurement` for valid tracking |
| `clock_id` | Nonempty, stable clock label |
| `extrinsic` | Optional *actual* producer mount declaration, as below |
| `source_id`, `clock_mapping`, `clock_verified` | Optional producer provenance; do not assert verification without evidence |

A lost/uninitialized sensor publishes `pose: null` and an honest status.
SLAM must not extrapolate that into a tracking pose. The pose's child frame
defines the reference point of the odometry, not automatically the robot's
center; reset or tracking loss requires a new segment where appropriate.

## Optional explicit input declaration: `motus.slam_input/1`

Provide one declaration **per topic**, keyed by actual ROS topic path, with
these common required fields:

| Key | Type / permitted values |
| --- | --- |
| `schema` | Literal `motus.slam_input/1` |
| `topic` | Exact connected ROS topic |
| `format`, `kind` | `state/odom` + `pose` or `sensor/pointcloud2` + `cloud` |
| `source_id` | Nonempty sensor/producer source ID, not `unknown` |
| `clock_id` | Nonempty shared clock label, not `unknown` |
| `reference_point` | Nonempty description of what position is reported |
| `stamp_source` | `measurement` |
| `length_unit` | `m` |

Pose declarations additionally require `child_frame_id` (same reference
frame as each tracking sample), with optional `pose_reference`, `frame_id`
and clock provenance.

Cloud declarations additionally require:

| Key | Type / permitted values |
| --- | --- |
| `frame_id` | Exact `PointCloud2.header.frame_id` |
| `cloud_semantics` | `depth_frame` or `scan`; never a pre-accumulated map |
| `observation_model` | `pinhole` or `lidar` |
| `timing_model` | `instantaneous` or `deskewed` |
| `frame_metadata_topic` | Separate `std_msgs/msg/String` provenance sidecar |
| `clock.mapping` | Nonempty mapping method identifier |
| `extrinsic` | Complete static or dynamic mount declaration, on cloud or pose |

Cloud and pose `clock_id` must match. When `source_id` differs, both
producers must provide a **verified** relationship with the same nonempty
`clock.relation_id`. For the strict `motus.slam_input/1` path, both
`clock.verified` must be `true`, and `extrinsic.calibrated` must be
`true`; a clock name or arrival-time approximation is not proof.

Example (identical sensor frame and odometry child frame, static identity mount):

```json
{
  "/robot/state/odom": {
    "schema": "motus.slam_input/1",
    "topic": "/robot/state/odom",
    "format": "state/odom",
    "kind": "pose",
    "source_id": "sensor-a",
    "clock_id": "clock-a",
    "reference_point": "sensor_origin",
    "stamp_source": "measurement",
    "length_unit": "m",
    "child_frame_id": "sensor",
    "clock": {"mapping": "device_to_ros", "verified": true}
  },
  "/robot/lidar/points": {
    "schema": "motus.slam_input/1",
    "topic": "/robot/lidar/points",
    "format": "sensor/pointcloud2",
    "kind": "cloud",
    "source_id": "sensor-a",
    "clock_id": "clock-a",
    "reference_point": "sensor_origin",
    "stamp_source": "measurement",
    "length_unit": "m",
    "frame_id": "sensor",
    "cloud_semantics": "scan",
    "observation_model": "lidar",
    "timing_model": "deskewed",
    "frame_metadata_topic": "/robot/lidar/points/metadata",
    "clock": {"mapping": "device_to_ros", "verified": true},
    "extrinsic": {
      "parent_frame": "sensor",
      "child_frame": "sensor",
      "mode": "static",
      "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
      "source": "survey",
      "version": "installation/1",
      "calibrated": true
    }
  }
}
```

This is an *example of a genuinely verified installation*, not a template
whose `verified` and `calibrated` flags should be copied to unverified
hardware. Clock claims need underlying measurement evidence.

## Extrinsic transform and coordinate frames

`extrinsic` maps **cloud coordinates into the odometry child coordinates**:

`p_parent = T_parent_child @ p_child`.

- `parent_frame` equals `pose.child_frame_id`; `child_frame` equals
  `PointCloud2.header.frame_id`.
- Static mode requires a rigid 4×4 matrix with meters for translation,
  an orthonormal right-handed 3×3 rotation, and last row `[0,0,0,1]`.
  Also include nonempty `source` and `version`.
- Dynamic mode requires `mode: dynamic`, `source: tf`, and **no**
  static matrix; the time-aligned transform is supplied at capture time.
- `calibrated: true` is a *producer assertion* requiring real installation
  calibration. An axis convention or guessed translation must remain
  `calibrated: false`.
- If a user supplies `child_from_cloud` on the SLAM card and a producer also
  declares a mount, the matrices must agree; a mismatch is an error.
- When the cloud frame equals the pose child frame **and no competing mount is
  declared**, identity may be used; do not invent identity across different frames.

## Per-frame provenance: `motus.slam_frame/1`

The cloud-side `frame_metadata_topic` carries a JSON `std_msgs/msg/String`
corresponding to the *exact* PointCloud2 payload, not a nearby frame:

| Field | Requirement |
| --- | --- |
| `schema` | `motus.slam_frame/1` |
| `stamp_ns` | Positive integer capture time, same as cloud header |
| `frame_id`, `clock_id` | Match cloud input declaration |
| `clock_epoch`, `source_generation` | Nonempty IDs, change on clock/source reset |
| `source_stamp` | Finite nonnegative original device timestamp |
| `source_unit` | `ms` or `ns` |
| `source_clock` | Nonempty original device clock name |
| `mapping.method`, `mapping.version` | Nonempty |
| `mapping.offset_ns` | Finite number |
| `mapping.verified`, `mapping.uncertainty_ns` | Evidence of a verified mapping in strict mode |
| `mapping.relation_id` | Required when independently sourced clocks are related |
| `data_sha256` | Lowercase SHA-256 of the **native PointCloud2 data bytes** |

The check reproduces normalized time exactly:

```text
stamp_ns = int(source_stamp * (1_000_000 if source_unit == 'ms' else 1)) + offset_ns
```

The current SLAM parser uses the tuple
`(clock_id, clock_epoch, source_generation)` to guard against stale or
restarted streams. Cloud headers, odometry measurements, sidecar
timestamps, hash, frame IDs and segment state are cross-checked before
admission. An unverified mapping, new generation, lost tracking or conflicting
mount must not be silently promoted to valid SLAM input.

## Core main (topic-only) compatibility

Agent Core's current `main` project-start path forwards `input_topics` and
some camera optics, but **not** the complete
`input_metadata`/`odom_interfaces` declarations. A connected graph therefore
uses the Perception card's *legacy-Core runtime discovery* unless a caller
explicitly supplies the strict contract:

1. Discover native publisher types from the two ROS topic names.
2. On `start_mapping`, observe a fresh tracking `motus.odom/1` sample and
   PointCloud2 capture stamp / frame (default 5-second probe).
3. Require overlapping measurement timestamps (default `max_gap_ms=200`);
   resolve the actual frame, odometry child frame and clock label.
4. Accept a producer mount only if calibrated; otherwise require a compatible
   manually configured 4×4 `child_from_cloud` matrix. Fail closed when
   frames/time/mount are inconsistent.
5. Report `input_provenance` in `info()` and the map revision metadata.
   The fallback reports `timestamp_overlap_only_not_independent_clock_proof`.

**This fallback is not a verified clock-sync protocol.** It must not be used to
assert that independent LiDAR and odometry clocks are synchronized.

The SLAM card's backend and input contracts are platform-wide; no
`robot: Bumi|G1|Tianyi` selector is needed. The current card retains one
fixed `sensor/mapping` visualization output. RTAB-Map's native database is
the stored map for future localization/navigation, not that preview packet.

## Driver author checklist

1. Produce actual `sensor_msgs/msg/PointCloud2` (finite float32 XYZ, metric,
   valid `header.stamp` and `frame_id`), not only visualization bytes.
2. Publish `motus.odom/1` tracking poses and additive `motus_slam/1` metadata.
   Advertise `odom_interface.slam_pose` only if available.
3. Keep pose and cloud frame names consistent with an actual measured or
   explicitly assumed `T_parent_child`.
4. Describe measurement clock origins, epochs, generation, uncertainty and
   any verified cross-source clock relationship; never replace capture time
   with arrival time without saying so.
5. Keep unavailable/unverified `calibrated` and `verified` flags false.
6. Test tracking, loss, reset, unequal source clocks, invalid mounts and
   payload/metadata mismatch using the pure-Python helpers and negative tests.

**Bumi example:** Driver PR #471's default optical-to-body transform is an
axis assumption (`calibrated: false`), and its RealSense arrival-offset
mapping is approximate (`clock_verified: false`). The driver exposes both
facts; it does not claim strict producer-clock or extrinsic calibration.
