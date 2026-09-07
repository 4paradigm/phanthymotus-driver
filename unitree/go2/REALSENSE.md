# External RealSense stereo sensors

The Go2 bundle exposes two independent, single-instance sensor cards for one
attached RealSense stereo camera. No `/dev/videoN` path is hard-coded.

| Card | ROS topic | Dashboard format | Content |
| --- | --- | --- | --- |
| `ext_depth` | `/{namespace}/ext_depth/image` | `image/depth-zlib` | 640x480 little-endian uint16 millimetres, zlib-compressed |
| `ext_infrared` | `/{namespace}/ext_infrared/image` | `image/jpeg` | 640x480 left infrared (stream index 1), grayscale JPEG |

Both use `sensor_msgs/msg/CompressedImage` with best-effort sensor QoS at
6 fps on USB 2 and 15 fps on USB 3 (unknown transport uses 6 fps). This leaves
bandwidth for the existing RGB stream on USB 2. `info` reports the negotiated
profile's `fps` and `usb_type`. Depth messages use `format="16UC1; compressedDepth zlib"`: payload is
plain zlib of the pixel buffer, without a ROS compressedDepth transport header,
as required by this repository's renderer. Infrared messages use `format="jpeg"`.
Depth values are converted using the device's queried `get_depth_scale()`;
zero marks invalid or unrepresentable values. Infrared intensity is not distance
or temperature. JPEG is intended for visual monitoring, not lossless IR storage.
Depth is in the depth camera view, not aligned to the RGB image.

The infrared card reads the SDK's native `infrared` stream, index 1, format
`Y8`; it does not convert RGB to grayscale. D435i is a near-infrared stereo
camera, not a thermal camera. Brightness represents reflected light intensity,
so a grayscale scene is the expected image, not a missing visualization effect.
Adding a false-color palette would not make it a temperature measurement.
`info.source_stream` and `info.stream_index` identify the native source.

## Lifecycle

The plugins are enabled in `config.yaml` and advertised in `driver.yaml`.
They do not open USB devices at bundle startup. Add either card to the canvas
and start it; no card configuration is required. MCP lifecycle calls use
`start`, `stop` and `info`, returning plain dictionaries.

`start` schedules capture and returns `starting`. Poll `info` for `running`
and `fresh: true`; these require actual published frames. Missing SDK/device,
unsupported profiles, USB errors, process exit, or stale data produce `error`.
Startup has a 10-second readiness deadline; established streams become stale
after 3 seconds. Explicit `start` retries after an error.

One subprocess owns the stereo sensor and opens depth Z16 plus left IR Y8
together. Each card controls only its own publication. Stopping one leaves the
other running; stopping the last card releases the sensor and process, with
bounded shutdown even if the SDK hangs. Multiple attached RealSense cameras
are rejected explicitly to avoid selecting a camera arbitrarily.

The RGB sensor is not opened, reset or reconfigured. `ext_camera` can own its
color interface concurrently. Its RealSense enumeration fix is tracked
separately in PR #248; these stereo cards do not depend on that fix.

## Build and verification

The Dockerfile installs the official `pyrealsense2==2.56.5.9235` wheel (available
for CPython 3.10 on Linux ARM64) and copies `realsense.py`. Keep the existing
`/dev` access and DDS environment/profile mounts from the Go2 deployment.

```bash
python3 -m unittest discover -s tests -p test_go2_realsense.py
```

Tests cover calibrated depth encoding, invalid data, startup/fault reporting,
independent card lifecycle and shared-device release. Hardware acceptance must
also check nonzero depth pixels, decoded infrared frames, independent stop and
restart, and delivery through the Agent Core monitoring WebSocket. Advertised
profiles or a running subprocess alone are insufficient.

The tested D435i uses a USB 2 connection. Stereo at 15 fps plus RGB 720p/15
interrupted the RGB stream; the USB 2 stereo profile therefore uses 6 fps.
With stereo at 6 fps, RGB 720p/15 was verified concurrently. RGB 1080p/8 still
timed out when stereo was enabled on this USB 2 connection: use RGB 720p/15
for three-stream operation, or stop both stereo cards when using RGB 1080p.
The USB 3 profile is selected from SDK capabilities but was not hardware-tested
on this USB 2 connection. Resolution is fixed to 640x480
because the existing depth renderer consumes a headerless buffer of that size.

On-device checks also covered independent stop/restart, capture-process exit
reporting and retry, and startup with no device mounted. Both new streams were
decoded through the production monitoring WebSocket, and their depth colormap
and grayscale IR image were visually checked in Chrome.

Protocol references: [repository driver guide](../../README_dev.md) and the
[official SDK depth units documentation](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0).
