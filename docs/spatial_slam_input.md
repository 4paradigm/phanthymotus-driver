# Spatial SLAM Input Contract (v1)

Drivers that advertise the optional `motus_slam/1` pose profile use the
[producer validators](../common/spatial_slam.py) for the declarations and
evidence described below. Producers own the physical alignment of poses and
clouds and should publish only clock and calibration claims they can support.

## Minimum input for Perception PR #371

The ordinary point-cloud mapping path reads a `motus.odom/1` object with a
numeric `pose`: three finite position values in meters and four finite xyzw
quaternion components with nonzero norm. The decoder normalizes the quaternion.
It uses the integer `pose.stamp_ms` if present, otherwise the integer top-level
`stamp_ms`. Covariance can be omitted or null; when supplied it has 36 finite
entries. Missing `info().odom_interface.slam_pose` produces a diagnostic.
The ordinary decoder accepts samples without frame names or `vendor.motus_slam`,
including its status, segment and clock fields. Publish `pose: null` when
there is no usable pose; a loss label alone leaves a numeric pose usable.

See the [consumer pose decoder](https://github.com/4paradigm/phanthymotus/blob/8d38150c2d3be9e7074b5aa9b8b67f806daaf888/perception/plugins/pointcloud_slam_odom.py)
and [numeric input tests](https://github.com/4paradigm/phanthymotus/blob/8d38150c2d3be9e7074b5aa9b8b67f806daaf888/perception/tests/test_pointcloud_slam_numeric_input.py).

## Wire types and capability

The pose and cloud connections use these ROS 2 wire types:

| Port format | ROS message | Contents |
| --- | --- | --- |
| `state/odom` | `std_msgs/msg/String` | JSON `motus.odom/1` (profile metadata in `vendor.motus_slam` is optional for PR #371) |
| `sensor/pointcloud2` | `sensor_msgs/msg/PointCloud2` | Original metric XYZ scan or depth cloud |

`sensor/pointcloud` is an `UInt8MultiArray` **dashboard rendering format**,
not a native PointCloud2 input; `sensor_msgs/msg/LaserScan` requires a
separate adapter. Do not relabel a visualization packet as PointCloud2.

A state card implementing the full driver profile announces, in `info()`:

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

For each tracking reading in the full driver profile, `motus.odom/1` must carry positive integer
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
| `length_unit` | `m` |

The declaration may include `source_id` (producer identity), `clock_id`
(clock label), `reference_point` (the point whose position is reported), and
`stamp_source` (timestamp provenance). These fields are optional port
metadata. Tracking samples and frame packets have their own timestamp checks.

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

To declare a verified pose/cloud clock relationship, supply matching
`clock_id` values. When the producers are different, both declarations must
provide the same nonempty verified `clock.relation_id`. A strict setup uses
measurement timestamps, sets both `clock.verified` values to `true`, and
requires `extrinsic.calibrated` to be `true`.

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

The driver validator can check these provenance claims. PR #371 can consume a
static cloud before, after, or without the sidecar. The pose and cloud still
need timestamps that allow pairing. The producer is responsible for their
physical timing and coordinate relationship.

## Runtime geometry in PR #371

When producer declarations are missing, runtime discovery can resolve geometry
from the connected samples. For ordinary static input, an absent mount means
the cloud is assumed to be in pose-child coordinates. Different frame labels
alone do not block that calculation. A dynamic mount needs a numeric transform
for the cloud. See [geometry resolution](https://github.com/4paradigm/phanthymotus/blob/8d38150c2d3be9e7074b5aa9b8b67f806daaf888/perception/plugins/pointcloud_slam_config.py)
and the numeric input tests above.

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
