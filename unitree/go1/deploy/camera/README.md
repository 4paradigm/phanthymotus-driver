# Go1 Nano camera streams

`camera.py` is the Pi-side visual-card aggregation file. It registers the
independent `camera_rgb`, `camera_depth`, and `camera_pointcloud` cards; each
instance selects one of the five positions below.

| Position | Nano | device_id | RGB | depth | point cloud |
|---|---:|---:|---:|---:|---:|
| front | 192.168.123.13 | 1 | 9201 | 9101 | 9401 |
| chin | 192.168.123.13 | 0 | 9202 | 9102 | 9402 |
| left | 192.168.123.14 | 0 | 9203 | 9103 | 9403 |
| right | 192.168.123.14 | 1 | 9204 | 9104 | 9404 |
| belly | 192.168.123.15 | 0 | 9205 | 9105 | 9405 |

All streamers are on-demand: the Nano process listens without opening the
camera, opens it only after the Pi connects, and exits on disconnect so systemd
returns it to idle. A physical camera can therefore serve only one of RGB,
depth, and point cloud at a time.

`vision_capture` connects directly to the selected RGB port; `camera_rgb` does
not need to be started. Use `{"action":"capture_photo","position":"front"}`
for one JPEG or `{"action":"record_video","position":"front","duration_s":5}`
for an MP4 (1–30 seconds, default 5). Both calls return promptly after acceptance,
with an `action_id`, a planned `file_path` and `channel_reply_path`, and
`state: capturing` / `recording`. `ok: true` means accepted and
`file_ready: false` means this response does not confirm a saved file.

The tool declares `inputSchema.x-completion` for `capture_photo` and
`record_video`, with a 120-second completion timeout. The background worker
reports one terminal outcome (`completed`, `error`, or `cancelled`) to the
existing local Agent Core `/api/acp/complete` endpoint. Successful completion
contains final file paths, MIME type, size and capture timing; failure or
cancellation contains an error code and message. Use this completion result to
determine file availability, rather than the initial MCP result.
`info.last_capture` and `info.last_recording` keep the latest
`{action_id, status, result}` for inspection.

ACP delivery uses `AGENT_CORE_URL` (default `https://localhost:15678`), no bearer
token, and at most three attempts with a 3-second request timeout and 0.5/1-second
backoff. Retries reuse the same action ID and terminal payload; an ambiguous
network failure may deliver the same payload again, so this is one logical
terminal outcome, not guaranteed exactly-once network delivery. Exhaustion is
logged; there is no durable retry queue across outages or process restarts.
The card does not send `/api/event` or require `ACCESS_TOKEN`. ACP completion
updates orchestration status; it does not implement a separate “录像已保存”
canvas message. Whether the canvas exposes final file paths has not been
verified with the disconnected robot.

`stop` reports lifecycle state `idle` and a separate `capture_active` flag;
an active photo continues while an active video is cancelled. Cancellation
and MP4 publication share a lock: cancellation before publication prevents a
successful MP4; once the file is published the recording is already complete.

As on Tianyi, `capture_photo` accepts an optional `image_name` and `record_video`
accepts an optional `video_name`. Use a filename stem without an extension;
otherwise the card generates one. `list` returns saved files with their full
container `path`, `channel_reply_path`, MIME type and size; `delete` accepts the
complete `.jpg` or `.mp4` filename. A file currently being written cannot be
deleted, and an existing name is never accepted for a new capture.

For `record_video`, the recording clock starts at the first valid camera frame.
The planned MP4 path becomes ready only after successful ACP completion.
Lifecycle `stop` cancels an active recording instead of saving it, including
when the encoder is being created or is finishing. Driver shutdown rejects new
captures, cancels active video, and waits for all accepted photo/video workers
and their bounded ACP delivery attempts, even after camera occupancy is released.
The service gives this drain up to 60 seconds before forced container exit.

Recording first writes the selected JPEGs into a temporary MJPEG stream, then
encodes that stream to MP4 with `ffmpeg`'s `ultrafast` H.264 preset. This keeps
slow encoding from blocking Nano frame reception and repeating the last image
for the rest of the requested duration. Unpublished temporary files are hidden from `list` and removed after
success, failure, or cancellation; allow disk space for the temporary stream
and final MP4 while encoding finishes. The completion result distinguishes
`recording_started_at` (first valid camera frame), `recording_ended_at` (capture
finished), and `file_ready_at` (MP4 published). Playback duration follows the
capture interval; file availability can be later because encoding runs after it.

`config.yaml` sets `vision_capture.output_dir` to
`/opt/phanthy-motus/data/vision_capture`. Photos go to `photos/*.jpg`, videos
to `videos/*.mp4`. `deploy/service.yml` bind-mounts `/opt/phanthy-motus/data`
at the same path on the host, so these files persist after container restart.
The `channel_reply_path` for a saved file starts with
`/work/resource/vision_capture` by default; it is intended for a channel that
mounts the same host data directory there. Set `PHANTHY_CHANNEL_OUTPUT_DIR` if
that channel uses a different mount, and verify the mount before sending a file.
The previous `camera_snapshot` card name is replaced by `vision_capture`; update
existing canvas calls. Previous JPEGs remain in
`/opt/phanthy-motus/data/camera_snapshot` and are not moved automatically.

**Deployment prerequisite:** run only one Go1 driver process/container per set
of five Nano cameras. The in-process position registry is not shared across
driver instances, and the Nano services do not provide a shared lease. Before
enabling the snapshot card or a second deployment, stop any other Go1 driver
instance targeting the same Nano IPs and ports. Duplicate deployments can
interrupt an active stream or make a capture fail.

Within one driver process, capture admission and stream startup share a lock:
an active RGB/depth/pointcloud receiver or another capture on the same position
returns `RESOURCE_BUSY` without an `action_id` or another TCP connection. A
stream cannot start or switch onto a position occupied by a capture or another
stream. Rejected hot switches preserve the old receiver. Stopping a stream
retains its occupancy until the receiving thread exits. Other positions remain
available.

Capture endpoints use `camera.py`'s five default RGB endpoints. The bundle
merges `positions` field by field, in increasing precedence:
`camera_pointcloud` → `camera_depth` → `camera_rgb` → `vision_capture`.
Configuration is inherited even if a stream card is disabled. Normally configure
`camera_rgb.positions` once; `vision_capture.positions` is only an explicit
override for deployments that need it. For example, overriding just
`camera_rgb.positions.left.board_ip` changes both RGB streaming and captures;
the existing `image_port` is retained. Keep depth/pointcloud mappings consistent
for the same physical position.

The Dockerfile installs `ffmpeg` for MP4 encoding, so the image will grow by
that package and its dependencies; no model/data artefacts or Python image
decoder are added. JPEG validation remains the frame-size limit and SOI/EOI
markers, not a full decode. `driver.yaml` is metadata only. Docker image size
has not been measured locally.

## RGB path

`rgb_stream.cc` reads calibration from the camera, applies CMei undistortion,
rotates the image upright, crops black borders, and sends
`[big-endian uint32 length][JPEG]` over TCP.

`nvjpeg_worker.cc` performs Jetson NVJPG encoding in a separate process.
This separation is required because loading GStreamer `nvjpegenc` alongside
the Unitree SDK/OpenCV can load incompatible libjpeg ABIs. Both sources are
compiled and deployed by `../nano_bootstrap.sh`; neither the obsolete
`camera_adapter` nor a manually installed adapter service is required.

## Depth and point cloud

`depth_stream.cc` and `pointcloud_stream.cc` use their respective TCP
protocols consumed by `camera.py`. Their services are installed only when
`DEPTH_ENABLE=1` or `PCL_ENABLE=1` is supplied to the container bootstrap.
