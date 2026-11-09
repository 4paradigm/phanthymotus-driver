# Planar region rectification

`planar_rectify` is one optional, single-instance `processor` Card in the existing
Bumi bundle. It converts a perspective-skewed quadrilateral into a configured
rectangle. The output can feed the existing OCR Card for slanted document
reading, exhibition-board reading or whiteboard notes. OCR, speech, task logic
and any robot movement remain outside this Card.

## Setup

The existing Bumi camera must publish JPEG images. There is no second camera
pipeline, model download, new Python package, container, port or mount.
Enable the Card in `config.yaml` before deploying:

```yaml
plugins:
  planar_rectify:
    enabled: true
    mode: auto
    corners: []
    output_width: 640
    output_height: 905  # example portrait A4 target, approximately 1:sqrt(2)
    max_fps: 2.0
```

The default source is `/<namespace>/camera/color`; a `start` call can supply one
`input_topic`, or the Core can inject one `input_topics` connection. Other
sources are allowed, but the Card never opens a camera or enables its producer.
Input must not be this Card's own output. A restart replaces the input baseline
and releases the previous subscription, timer and worker.

| Setting | Default | Meaning |
| --- | --- | --- |
| `mode` | `auto` | `auto`: geometric quadrilateral proposal; `manual`: configured corners |
| `corners` | `[]` | Four `[x,y]` pairs in `[0,1]`, clockwise **TL, TR, BR, BL**; required in manual mode |
| `output_width` | `640` | Output pixels, integer 64-1280 |
| `output_height` | `480` | Output pixels, integer 64-1280 |
| `max_fps` | `2` | Processing cap, 1-5 Hz; latest frame only, no queue |

Set output width/height to the intended surface's aspect ratio. A homography from
four corners alone does **not** establish the physical dimensions or true aspect
ratio of an unknown rectangle. The default 4:3 output is not an A4 detector.
There is no text-orientation inference or automatic portrait rotation. Manual
corner order selects orientation; automatic ordering is a geometric heuristic.

Automatic mode uses contrast edges, closed contours and convex quadrilaterals,
bounded to a 640-pixel detection image. Candidates cover 15%-95% of the image;
cropped borders, tiny shapes and a substantial distinct second candidate are
rejected. Success means a **geometric candidate**, not a claim that paper or a
whiteboard was semantically recognized. Manual mode is the fallback when the
boundary is unclear. This is planar rectification; it cannot uncurve a book,
remove lens distortion, recover hidden corners or restore blurred text.

## MCP calls and lifecycle

The bundle starts the configured processor; when disabled, it is not loaded.
`start`/`stop` actually acquire/release this processor's resources. They never
start/stop the camera or alter its configuration. `config` validates first, then
restarts a running processor with the new settings; invalid settings preserve
the existing running instance. Settings changed by MCP are in-memory only.

```bash
curl -sS http://127.0.0.1:15704/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"planar_rectify","arguments":{"action":"info"}}}'
```

Example manual start (replace the topic and corners with actual observations):

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "planar_rectify",
    "arguments": {
      "action": "start",
      "input_topic": "/bumi/camera/color",
      "mode": "manual",
      "corners": [[0.23,0.11],[0.67,0.21],[0.74,0.88],[0.16,0.81]],
      "output_width": 640,
      "output_height": 905
    }
  }
}
```

`info`/`robot_info` are immediate snapshots. Unknown actions return `None` to the
existing MCP handler. There are no long-running physical actions and no ACP job;
processing runs continuously in a bounded latest-frame worker. No files are
written and the latest input is cleared on stop.

## Output and geometry

| Topic | Format | Content |
| --- | --- | --- |
| `/<namespace>/planar_rectify/image` | `image/jpeg` (`sensor_msgs/CompressedImage`) | Rectified JPEG, emitted only for a valid, recent result |
| `/<namespace>/planar_rectify/state` | `data/json` (`std_msgs/String`) | 2 Hz status, input age, selected corners and homography |

The status includes:

- `state`: `running`, `idle`, or `error` (unexpected worker failure).
- `status`: `no_input`, `ok`, `no_boundary`, `ambiguous_boundary`, `invalid_image`,
  `replayed_input`, `stale_input`, `stale_result`, `processing_error`, `worker_error`
  or `idle`.
- `valid`: authoritative result validity. `frame_sequence` identifies the input
  message processed within this process; it is not a hardware frame counter.
- `source_size`, `output_size`, `corners_px` and `homography`: pixel geometry.
  `H` maps source pixel `[x,y,1]` into output pixels after homogeneous division.
- `selection`: `configured_corners` or `geometric_candidate`; no calibrated
  probability or OCR accuracy is claimed.
- `input_received_monotonic_s`, `input_age_s`: host receipt and result age.
  The raw `input_ros_stamp` is preserved, with `capture_time_verified: false`.
  Missing stamps are permitted; nonzero repeated/regressing stamps invalidate
  the baseline until a newer stamp or explicit Card restart arrives.
- `camera_info: []`, `metric_geometry: false`, and
  `output_geometry: planar_projective_pixels`: the flattened image is **not** the
  original pinhole-camera image. Do not reuse the source intrinsics, depth map,
  odometry or obstacle-detection geometry on these pixels.

The outgoing JPEG retains the input ROS stamp and uses frame ID
`planar_rectified_pixels`. There is no timestamp conversion that invents a
hardware capture time. Host receipt/result ages above two seconds invalidate
the report and suppress images; incoming packets cannot keep an old processed
result valid. JPEGs are limited to 2 MiB and 1920x1080 total pixels, checked from
the JPEG header before decoding. Input and output memory are bounded.

On failure, **no replacement JPEG is emitted**. A viewer may still display its
last image; consult `valid`, status and age before acting on it. Status delivery
and the final idle notification are DDS best effort, not an exactly-once event
or image/JSON transaction. A downstream consumer must also detect missing data.
This Card does not prove OCR improved; that requires actual documents and the
deployed OCR model.

## Verification

Offline tests use real OpenCV on synthetic, known perspective transforms and ROS
test doubles. They verify pixel reconstruction, corner geometry, false-success
rejection, JPEG bounds, replay/stale handling, cancellation, cleanup and bundle
registration. They are not a physical camera or ROS middleware acceptance test.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest tests/test_bumi_planar_rectify.py tests/test_bumi_bundle.py \
  -q -p no:cacheprovider
```

A separate smoke script exercises the actual local ROS 2/DDS middleware with a
synthetic image producer, never a camera or SDK. After sourcing ROS Humble, use
an unused isolated domain and localhost-only transport:

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
PYTHONDONTWRITEBYTECODE=1 python3 tests/bumi_planar_rectify_ros_smoke.py
```

It checks JPEG/status delivery, source-stamp preservation, stale state, final
idle notification and node/thread release. This still is not a robot acceptance
test. The script refuses the production domain 42 and the default domain 0.

After separate authorization to deploy on the robot:

1. Enable only this optional Card and the existing camera; leave the robot
   stationary. Verify `tools/list` advertises one processor and `info` lists the
   configured source and both output topics.
2. Present a flat page with all corners visible against a contrasting background.
   Set the target aspect ratio and verify corners, orientation, text and crop.
   Compare the source and rectified images; do not infer success from status alone.
3. Present no page, two comparable pages, a cropped page and a low-contrast page.
   Record failures and any wrong geometric selections. Use manual corners where
   auto selection is unreliable; do not claim universal page detection.
4. Stop image delivery: state must become stale within two seconds of the last
   received input. Stop the processor: a best-effort idle report is sent and the
   worker/node are removed. The camera and other Cards must remain running.
5. If OCR is desired, compare actual text recognition on the same content with
   and without rectification, then measure CPU, latency and dropped-input behavior
   on the Jetson. These measurements have not been performed locally.

No canvas was opened or edited for development. If a later canvas acceptance is
requested, connect `camera -> planar_rectify -> OCR`, select the **image** output
for OCR and inspect the JSON output separately. Expected behavior is the same
topic contract above; deployment and canvas editing require the operator's next
authorization.

## References

- [OpenCV geometric transforms](https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html)
  (`getPerspectiveTransform`, `warpPerspective`).
- [OpenCV homography tutorial](https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html).
- [OpenCV-Document-Scanner](https://github.com/andrewdcampbell/OpenCV-Document-Scanner)
  as a use-case reference. The implementation here is independently written;
  no source, model, test dataset or third-party UI was imported.
