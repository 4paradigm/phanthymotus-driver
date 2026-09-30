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

`camera_snapshot` connects directly to the selected RGB port for one JPEG and
then disconnects. In the canvas, call `camera_snapshot` with
`{"action":"capture_photo","position":"front"}` (or chin/left/right/belly);
`camera_rgb` does not need to be started. The call immediately returns
`{ok: true, state: "capturing", action_id, position, file_path}`. `file_path` is
the planned destination and may not exist yet. The background worker
POSTs `completed` or `error` to `${AGENT_CORE_URL}/api/acp/complete`; only
`capture_photo` declares `x-completion` (45 seconds). Successful completion
contains `file_path` under `/opt/phanthy-motus/data/camera_snapshot`, published
only after the atomic write. `info.last_capture` also keeps the latest terminal
`{action_id, status, result}`. ACP uses the `camera` physical resource. The
callback is retried up to three times (3-second request timeout, 0.5/1-second
backoff); after repeated failures the result remains in `info.last_capture`,
the failure is logged, and Core's 45-second barrier timeout is the fallback.
Other Go1 actuators without `x-resource` may still wait on this pending action
under the platform's conservative fallback. `stop` reports lifecycle state
`idle` and a separate `capture_active` flag; an accepted capture still finishes
and sends its ACP callback.

**Deployment prerequisite:** run only one Go1 driver process/container per set
of five Nano cameras. The in-process position registry is not shared across
driver instances, and the Nano services do not provide a shared lease. Before
enabling the snapshot card or a second deployment, stop any other Go1 driver
instance targeting the same Nano IPs and ports. Duplicate deployments can
interrupt an active stream or make a capture fail.

Within one driver process, snapshot admission and stream startup share a lock:
an active RGB/depth/pointcloud receiver or another snapshot on the same position
returns `RESOURCE_BUSY` without an `action_id` or another TCP connection. A
stream cannot start or switch onto a position occupied by a snapshot or another
stream. Rejected hot switches preserve the old receiver. Stopping a stream
retains its occupancy until the receiving thread exits. Other positions remain
available.

Snapshot endpoints use `camera.py`'s five default RGB endpoints. The bundle
merges `positions` field by field, in increasing precedence:
`camera_pointcloud` → `camera_depth` → `camera_rgb` → `camera_snapshot`.
Configuration is inherited even if a stream card is disabled. Normally configure
`camera_rgb.positions` once; `camera_snapshot.positions` is only an explicit
override for deployments that need it. For example, overriding just
`camera_rgb.positions.left.board_ip` changes both RGB streaming and snapshots;
the existing `image_port` is retained. Keep depth/pointcloud mappings consistent
for the same physical position.

No new runtime packages, model downloads, or image decoders are installed.
JPEG validation remains the frame-size limit and SOI/EOI markers, not a full
decode. The necessary Dockerfile COPY adds only the small Python source file,
so image-size impact is negligible; `driver.yaml` only advertises the actuator
and does not add image contents. Docker image size has not been re-measured.

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
