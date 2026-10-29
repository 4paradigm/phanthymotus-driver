# TRON2 YOLO detection card

Standalone, read-only `yolo_detect` processor. Connect one existing `camera`,
`camera_left` or `camera_right` JPEG stream in Canvas. Latest frames are sent to
an existing YOLO-World GPU service; annotated JPEGs and structured detections
are published on `/limx_tron2/yolo/annotated` (`image/jpeg`) and
`/limx_tron2/yolo/detections` (`data/json`). No robot motion is sent.

The UI exposes View Data Stream without confidence/FPS/label tuning fields.
Internal lifecycle actions `start`, `stop`, `info` remain available. The source
is derived from Canvas wiring; callers can pass `input_topic` or exactly one
`input_topics` entry. Legacy tuning values are ignored. All valid model
results are retained; inference load is limited to 2 Hz with a latest-frame queue.

## Dependencies and integration

- ROS2 Humble (`rclpy`, `sensor_msgs`, `std_msgs`) and Pillow.
- The existing Core/camera FastDDS domain/profile. Set `ROS_DOMAIN_ID`,
  `YOLO_EXPECTED_DOMAIN` (default 42), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  and `FASTRTPS_DEFAULT_PROFILES_FILE` pointing to an existing readable profile.
- The existing deployment-approved YOLO service at `http://172.28.4.23:18080`.
  The adapter deliberately accepts only this endpoint; another site needs an
  explicit endpoint/configuration review, not an arbitrary tool-call URL.
  GPU service deployment and model weights are not included.

Backend contract: `GET /health` reports `status: ok`, `model_loaded: true`,
`model: yolov8l-worldv2.pt`. `POST /detect` accepts original JPEG bytes as
`application/octet-stream`, returning `image_width`, `image_height`,
`box_coordinate_system: original_image_pixels`, and `detections` entries with
`label`, `confidence`, `box_xyxy`. Dimensions, coordinates and freshness are
validated. See `ModelClient` and `validate_detections` for bounds.

The service binds loopback `15792`. Register
`http://127.0.0.1:15792/mcp` as an HTTP MCP service in Motus, connect a camera,
and enable intelligent control. Stopping intelligent control stops detection.
`/health` reports state/counters and `/latest.jpg` returns the image or status
slate. The sibling `../head_follow` uses this health/result contract and requires
the head camera as source; wrist input can still be used for detection alone.

## Worker deadline fix (0.1.2)

The old limiter read the clock twice around a deadline, occasionally producing
`time.sleep(negative)` outside the processing error handler. This killed
`yolo_latest_frame` while HTTP and DDS input could stay alive, leaving STARTING
on screen. The limiter now computes remaining time from one clock sample and
only waits for positive intervals. A deterministic regression exercises this
boundary. Health adds `worker_alive`; an active dead worker reports an error.
No model, vocabulary, inference rate or camera behavior changes.

## Build and validate

From the repository root:

```bash
docker build -t tron2-yolo-detect:0.1.2 limx/tron2/yolo_detect
```

Run as a separate host-network service with read-only root, non-root user, the
existing DDS profile mounted read-only, environment above, and a private writable
`/tmp` (such as a 128 MiB tmpfs). Do not start another instance if port 15792 is
already owned by the field service. The parent TRON2 image stays unchanged.

```bash
cd limx/tron2/yolo_detect
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
YOLO_HTTP_TEST=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tests -v
```

Unit tests use fake model/bus and do not contact hardware. Real loopback HTTP
can be enabled separately. `smoke_dds.py` exercises DDS and must run **only in a
network-none test container with a separate DDS domain/profile**. `probe.py`
is a read-only live receiver proving two matching image/result pairs by hash
and distinct frame sequence; neither sends robot commands.

Field repair passed 59 tests (including deployment-only tests outside this
contribution), isolated DDS checks, actual detection validation and continued
inference/output growth. Its receipt reported about 2 fps, 298–364 ms latency,
worker alive, and no robot motion. Personal deployment paths, update installers,
replacement journals and backups are excluded from this PR.
