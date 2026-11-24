# Traffic light processor Card

`traffic_light` classifies vehicle traffic lights using VOP detections and the RGB images that produced them. It publishes per-housing and per-bulb evidence and temporally confirmed decisions. Downstream consumers can read those results through the output topic or the `latest` action.

## Inputs and exact frame matching

Bind each Card instance to one VOP `std_msgs/String` JSON output ending in `/objects`. For `/robot/camera/color/objects`, the Card subscribes to `/robot/camera/color` as `sensor_msgs/CompressedImage` and publishes JSON on `/robot/camera/color/objects/traffic_light`.

The VOP JSON must contain an `objects` array and the following source provenance fields.

| Field | Required value or meaning |
| --- | --- |
| `source_stamp` | Original RGB `header.stamp` as nonnegative integer `sec` and `nanosec`. `nanosec` must be below one billion and the pair must be nonzero. |
| `stream_id` | Nonempty identifier for the current VOP stream. A stream restart starts a new identity. |
| `frame_seq` | Nonnegative integer that increases within a stream. Gaps are allowed. |
| `source_mode` | `stream` |
| `source_clock_status` | `ok` |
| `temporal_eligible` | `true` |

When present, `source_topic` must equal the inferred RGB topic and `source_stamp_semantics` must be `ros_header_stamp`. The processor selects detections named `traffic light` and uses their VOP `bbox` (`[x1, y1, x2, y2]` in source-image pixels) and `confidence`. Keep VOP `publish_bbox` enabled so the Card can locate the bulbs within each detection. An optional normalized VOP `position` pair is passed through as `{x, y}`.

The join uses exact integer nanosecond equality between `source_stamp` and RGB `CompressedImage.header.stamp`. A bounded cache handles either arrival order within the freshness window. The VOP result `timestamp` and local clocks are never used to substitute for the source header or to find a nearby frame. ROS time checks source freshness, and a monotonic clock bounds cache and track lifetimes.

## Output

| Field | Meaning |
| --- | --- |
| `state` | `ok`, `no_signal`, `waiting`, `stale`, or `error`. `ok` means the frame was processed and can include unknown decisions. `no_signal` means a matched frame had no traffic light detections. |
| `reason` | Diagnostic reason when available. |
| `timestamp` | Card result creation time, in wall-clock seconds. |
| `frame_stamp`, `capture_stamp` | Equal `{sec, nanosec}` values for the matched RGB ROS source-frame header, or `null` when unavailable. Use the integer fields for exact equality. |
| `frame_timestamp`, `capture_timestamp` | Equal numeric seconds for the same stamp, or `null` when unavailable. |
| `frame_timestamp_semantics` | `ros_header_stamp` when the matched header is available, otherwise `unavailable`. The timestamp describes the image message header. Equivalence to RealSense sensor exposure time has not been established. |
| `stream_id`, `frame_seq` | VOP source identity on successfully processed frames. |
| `signals` | Per-housing results with `signal_id`, `bbox`, `position`, `detection_confidence`, `observations`, and `bulbs`. Each bulb has its own box and evidence. |

Each housing or bulb reports the current frame's evidence separately from the confirmed decision.

| Field | Meaning |
| --- | --- |
| `raw_color`, `raw_color_confidence` | `red`, `yellow`, `green`, or `unknown`, with a score based on HSV saturation and brightness in emitting regions. |
| `light_type`, `light_type_confidence` | `circle`, `arrow`, or `unknown`, with a score based on shape fit, resolution, coverage, and edge quality. |
| `direction`, `direction_confidence`, `direction_scores` | `left`, `straight`, `right`, or `unknown`. The direction score is arrow-template overlap. `direction_scores` reports the individual template similarities when available. |
| `decision` | Temporally confirmed `color`, `light_type`, and `direction` plus `stable`. Individual dimensions can remain `unknown`. A confirmed circle has `direction: unknown`. `stable` requires a confirmed color and a confirmed symbol. |
| `dimension_stability` | Stability of `color`, `light_type`, and `direction` individually. |
| `color`, `color_confidence`, `symbol`, `symbol_confidence`, `color_stability`, `symbol_stability`, `stability` | Compatibility fields for the confirmed result. `symbol` uses `circle`, `arrow_left`, `arrow_straight`, `arrow_right`, or `unknown`. |

The Card's confidence values are algorithm evidence scores in `[0, 1]`. They have no statistical probability calibration. Raw shape and direction hypotheses can appear even when the quality gates leave the corresponding decision `unknown`.

## Temporal confirmation and multiple lights

The current label needs at least three supporting observations in a window of up to five distinct source frames. Replays add no votes and do not refresh freshness. An arrow type can be confirmed across different accepted orientations. A direction needs three observations of that same accepted direction. The existing symbol gates require a minimum symbol size of 16 pixels and a median edge gradient of at least 160. Circle solidity must be at least 0.94, with circularity and aspect ratio checks. Arrow solidity must be in `[0.50, 0.90]`, the best template score must be at least 0.82, and its lead over the runner-up must be at least 0.15.

Each housing has its own track, and bulbs within a housing retain independent evidence. Consult `bulbs` when several lamps are illuminated. For example, mixed colors may leave the housing color `unknown` even when individual bulb colors are clear. Ambiguous housing associations retire the affected tracks, missing targets clear their votes, and a new stream starts with fresh history.

While an exact frame is pending, the result is `waiting` with unknown decisions. Missing or invalid provenance, conflicting stamps, retired streams, and out-of-order frames cannot contribute votes. Expired results are `stale` and their evidence and decisions become `unknown`. A stale result can retain its previously matched source timestamp for diagnostics. Invalid input or a failed image decode can produce `error` with no signals. Consumers can use `state`, `reason`, and each `decision` to distinguish these cases.

## Bumi registration

The Bumi bundle registers `common.traffic_light.TrafficLightPlugin` as a `processor` when `plugins.traffic_light.enabled` is true. The shipped configuration enables it with a 64-frame cache, a 1-second freshness window, up to 32 tracks, a 1-second track TTL, and a minimum VOP detection confidence of 0.3. The bundle image includes the shared `common` package, NumPy, OpenCV, and the ROS message dependencies.

Registration exposes the Card through MCP. A bound instance starts its own subscriptions when invoked with `start` and releases them on `stop`. `info` reports the topics and synchronization status, and `latest` returns the current result with runtime metadata. Each instance has its own cache and track history.
