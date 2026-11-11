# Automatic white-paper rectification

`planar_rectify` is an optional single-instance processor in the existing Bumi
bundle. It automatically proposes white-paper-like quadrilaterals, rectifies
the region and asks the existing perception OCR service for a reading direction.
There are no manual-corner or manual-rotation controls. No UVDoc, extra model,
text recognition engine, physical motion or camera capture is added here.

## Setup and migration

The existing camera must publish JPEG images; this Card never opens it.
Enable the Card in config.yaml:

    plugins:
      planar_rectify:
        enabled: true
        output_width: 640
        output_height: 905
        max_fps: 5.0
        preview: true
        ocr_port: 15720

| Setting | Default | Meaning |
| --- | --- | --- |
| output_width | 640 | Final pixels, integer 64-1280 |
| output_height | 905 | Final pixels, integer 64-1280 |
| max_fps | 5 | Processing cap, 1-15 Hz; actual throughput must be measured |
| preview | true | Labelled original-image detection preview |
| ocr_port | 15720 | YAML-only same-host perception port, integer 1-65535 |

Remove old manual mode/corners settings. Legacy mode=auto and corners=[] are
accepted for migration, but are not advertised. Manual settings fail explicitly.
The output size is retained after automatic quarter-turns by composing a new
source-to-final homography, not by stretching the already rotated result.
The requested aspect ratio is an operator-supplied target, not a measurement of
a sheet's real dimensions. Blank/uncertain documents do not establish a true
reading orientation or portrait/landscape aspect ratio.

The default input is /<namespace>/camera/color. start accepts one input_topic
or the Core's single input_topics connection. All three of this Card's own
output topics are rejected as feedback inputs.

## Automatic detection

Two edge passes and a light/neutral-region mask propose observed closed
quadrilaterals on an image at most 640 pixels across. Candidate area is 5%-95%
of the image. The neutral-color coverage, convexity, edge length, full-contour
border check and contour/quad area agreement reject unsuitable shapes.
Duplicate contours across passes are merged. A substantial distinct second
candidate produces ambiguous_boundary instead of an arbitrary selection.

This is a geometric/color heuristic, not semantic paper recognition. White
walls, screens and other rectangular objects can still be confused with paper.
Occluded/out-of-frame corners, strongly curved pages and indistinct boundaries
can fail. No hidden corner, missing text, real paper size or dewarping mesh is
invented. Enlarging the output does not restore missing source detail.

## Automatic direction: companion OCR API required

The matching change in the phanthymotus repository adds
ocr.classify_orientation to the existing perception service. That service
reuses its one shared OCR detector/classifier and inference locks; it does not
run text recognition or construct a second model set for the request. The
existing OCR bundle may load on first use, using its normal loader/cache.

An old perception image does NOT expose this action. Updating only the driver
therefore gives geometry and preview, with orientation explicitly unknown;
it does not silently provide automatic upright images. The two local changes
must be reviewed/built together before direction can be accepted on hardware.

The driver calls only http://127.0.0.1:<ocr_port>/mcp. JPEG data stays in memory
(base64, at most 2 MiB); proxies and redirects are disabled. There is one bounded
pending sample, one in-flight socket-timeout-limited request, and normally at
most one new request per second. Missing/unsupported OCR backs off for 10 s.
RPC runs outside the image worker so model loading/requests do not set its FPS.

Responses use schema motus.document-orientation/1 and a clockwise correction of
0, 90, 180 or 270 degrees. Direction is accepted only with sufficient line-level
classifier evidence and consensus. Empty, short/square or conflicting text
leaves direction unknown. A normalized appearance signature invalidates old
page results; a cached result expires after 3 s, reports its originating frame
and age, and is never described as a fresh per-frame OCR measurement. This
association is a heuristic, not a semantic document identity guarantee.

## Calls and lifecycle

    {"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"planar_rectify","arguments":{"action":"start","input_topic":"/ubuntu/camera/color"}}}

start/stop control only this processor's resources. info, robot_info and a bare
planar_rectify call return status. config validates first. Unchanged config is
a no-op; parameter changes retain the ROS node, invalidate in-flight results
and direction, and apply to new work. Only changing the input topic rebinds its
subscription. Invalid settings preserve the existing running instance.

The worker always processes the latest input, without an accumulating queue.
Its deadline counts processing time toward the requested frame period; the
configured wait is no longer added on top of every processing step. Stopping
wakes and joins image/direction workers and releases the ROS node. Changes made
by MCP remain in memory and do not change config.yaml.

## Outputs

| Topic suffix under /<namespace>/planar_rectify | Format | Meaning |
| --- | --- | --- |
| /image | image/jpeg | Newly rectified region, only while geometry/input is valid |
| /state | data/json | 2 Hz validity, age, homography, direction and measured processing statistics |
| /preview | image/jpeg | Labelled ORIGINAL image with current outline/status, including no_boundary |

The preview is NOT the rectified output. Its frame ID is
planar_preview_original_pixels; the rectified frame ID is
planar_rectified_pixels. Failure does not pass the original off as successful
rectification. Invalid JPEGs and lost input do not create invented live images.
Viewers may retain their last frame: always check status and input age.

State fields include:

- valid: geometric/input validity, not verified reading direction.
- orientation_verified: whether a sufficiently supported current cached OCR
  direction was applied; false on invalid geometry/input.
- orientation: ready/unknown, bounded reason, model support (not a calibrated
  probability), originating frame and cache age.
- rotation_cw_degrees: applied 0/90/180/270, or null when direction is unknown.
- corners_px, homography, source_size, output_size: final pixel geometry.
- selection: white_paper_candidate, not a semantic detection label.
- processing_ms and actual_output_fps: measured worker time/output rate,
  not the camera's capture rate or a promised Jetson performance result.
- input_ros_stamp, input_received_monotonic_s, input_age_s: source stamp plus
  host-receipt/result age; capture_time_verified remains false.
- state: running/idle/error; status includes no_input, ok, no_boundary,
  ambiguous_boundary, config_changed, invalid_image, replayed_input, stale_input,
  stale_result, processing_error, worker_error and idle.

Geometry may be valid while reading direction is unknown: in that case /image
contains an unrotated rectified crop and orientation_verified=false. Consumers
requiring an upright page must check both fields.

Images are limited to 2 MiB and 1920x1080 source pixels, checked before decoding.
Repeated/regressing nonzero source stamps invalidate the input baseline. Ages
above 2 s invalidate a processed result and suppress its image; new input alone
cannot make an old result valid. The JSON/image topics are best effort and are
not an atomic transaction or an exactly-once notification.

camera_info remains empty and metric_geometry=false. The rectified image is not
a pinhole-camera view: source intrinsics, raw depth and odometry must not be
reused on these pixels.

## Validation boundaries

Local tests use real OpenCV/synthetic known geometry, model/ROS doubles, and a
loopback HTTP contract server. They cover white-vs-colored candidates, absent
boundaries, quarter-turn homographies, API bounds, unknown direction, stale
responses, rate limiting, config lifecycle and worker shutdown. They do not
establish pretrained-model accuracy, Jetson FPS, real paper detection reliability
or deployed two-service compatibility. Hardware testing requires separate
authorization; no robot, canvas or container is changed by local development.
