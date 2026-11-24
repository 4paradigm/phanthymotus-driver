# Automatic white-paper geometry rectification

The optional single-instance `planar_rectify` Card automatically finds a
white-paper-like quadrilateral in the existing JPEG stream and flattens it.
It runs independently of perception/OCR: no OCR API, network client, new
model, new hardware resource or companion repository change is required.
It does not evaluate text direction, recognize text or turn an upside-down
page upright. Downstream OCR remains a separate existing capability.

## Setup

    plugins:
      planar_rectify:
        enabled: true
        output_width: 640
        output_height: 905
        max_fps: 5.0
        preview: true

| Setting | Default | Meaning |
| --- | --- | --- |
| output_width | 640 | Output pixels, integer 64-1280 |
| output_height | 905 | Output pixels, integer 64-1280 |
| max_fps | 5 | Processing cap, 1-15 Hz; actual throughput must be measured |
| preview | true | Publish labelled original-image detection preview |

There are no manual-corner or manual-rotation controls. Legacy mode=auto and
corners=[] are accepted for migration; manual settings fail explicitly.
Remove the former ocr_port setting: it is no longer used or advertised.

The configured aspect ratio is a target, not a measurement of a sheet's
physical size or reading direction. The default is approximately portrait A4;
this does not automatically establish which way text should face.

The existing camera must publish JPEGs. The default input topic is
/<namespace>/camera/color. start accepts input_topic or one input_topics
connection. The Card never opens or starts the camera, and rejects its own
image/state/preview topics as feedback inputs.

## Detection and limits

Two edge passes and a light/neutral-region mask propose observed closed
quadrilaterals on an image at most 640 pixels across. Candidate area is 5%-95%
of the image. Neutral-color coverage, convexity, edge length, full-contour
border checks and contour/quad area agreement reject unsuitable shapes.
Duplicate contours are merged. A substantial distinct second candidate
produces ambiguous_boundary instead of an arbitrary selection.

This is a geometric/color heuristic, not semantic paper recognition.
White walls, clothing, screens and other rectangular objects can still be
confused with paper. Occluded/out-of-frame corners, curved pages and
indistinct boundaries may fail. No hidden corner, missing text, true paper
size or dewarping mesh is invented. Enlarging the output cannot restore
source detail. Text reading direction is explicitly not evaluated.

## Calls and lifecycle

    {"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"planar_rectify","arguments":{"action":"start","input_topic":"/ubuntu/camera/color"}}}

start/stop control only this processor's resources. info, robot_info and a
bare planar_rectify call return status. config validates before changing
anything. Unchanged settings are a no-op. Parameter changes retain the ROS
node and invalidate old in-flight results; only changing the input topic
rebinds the subscription. Invalid settings preserve the running instance.

The worker takes only the latest frame and never accumulates a frame queue.
Its deadline includes processing time instead of adding the full configured
wait after every frame. Stop wakes/joins the worker and releases the node.
MCP configuration changes remain in memory and do not edit config.yaml.

## Outputs

| Topic suffix under /<namespace>/planar_rectify | Format | Meaning |
| --- | --- | --- |
| /image | image/jpeg | Newly rectified region, only for valid recent geometry |
| /state | data/json | 2 Hz validity, age, homography and processing statistics |
| /preview | image/jpeg | Labelled ORIGINAL image with outline/status, including no_boundary |

The preview is not the rectified output. Its frame ID is
planar_preview_original_pixels; the rectified output uses
planar_rectified_pixels. A failure never passes the original off as a
successful rectification. Invalid JPEGs/lost input do not create invented
live images. A viewer may retain its last frame, so inspect status and age.

State includes:

- valid: geometric/input validity, not verified text reading direction.
- reading_direction: not_evaluated; there is no automatic text rotation.
- corners_px, homography, source_size, output_size: pixel geometry. The matrix
  maps source pixels into the configured output rectangle.
- selection: white_paper_candidate, not a semantic recognition label.
- processing_ms and actual_output_fps: measured worker time/publication rate,
  not camera capture rate or a promised Jetson performance result.
- input_ros_stamp, input_received_monotonic_s and input_age_s: source stamp and
  host-receipt/result age. capture_time_verified remains false.
- state: running/idle/error. status includes no_input, ok, no_boundary,
  ambiguous_boundary, config_changed, invalid_image, replayed_input,
  stale_input, stale_result, processing_error, worker_error and idle.

JPEGs are limited to 2 MiB and 1920x1080 source pixels before decoding.
Repeated/regressing nonzero stamps invalidate the input baseline. Ages above
2 s invalidate a processed result and suppress its image; new input alone
cannot make an old processed result valid. Topics are best effort and not an
atomic image/JSON transaction or an exactly-once notification.

camera_info remains empty and metric_geometry=false. Rectified pixels are not
a pinhole-camera view: source intrinsics, raw depth and odometry must not be
reused on them.

## Local validation boundaries

Tests use real OpenCV/synthetic known geometry and ROS doubles. They cover
white-vs-colored candidates, missing boundaries, live original previews,
JPEG/parameter bounds, stale/replayed input, configuration races and worker
cleanup. A test also forbids service connections while geometry runs.

A separate smoke check uses synthetic JPEGs over isolated localhost-only
ROS2/DDS. These checks do not establish real paper detection accuracy, Jetson
FPS, OCR improvement, Docker build success or hardware acceptance.

## Scope correction after commit 697bf9b5

The earlier committed draft introduced an OCR direction dependency. This
follow-up removes its client, Docker COPY, config, tests and direction claims
while retaining white-paper detection, preview and frame/lifecycle changes.
The existing commit/PR can remain; the user can append a correction commit.
The unsubmitted companion OCR worktree is not needed for this Card and is not
part of its submission. No commit, push, deployment or comment is performed
by these code changes.
