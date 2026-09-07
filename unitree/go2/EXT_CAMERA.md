# External camera: RGB, depth and infrared

`ext_camera` is one multi-instance sensor. Its instance configuration contains
`channel: rgb | depth | infrared` (default `rgb`). There are no executable
camera-control actions. `config`, `start`, `stop` and `info` are internal
lifecycle operations, with plain-dict responses.

`camera_front` remains the Go2's built-in front video service. This card uses
external USB cameras. Normal USB webcams support `rgb`; depth and infrared
require a compatible RealSense with a resolvable physical USB path.

## Configure and switch

1. Add `ext_camera`, select the camera device and choose `channel`.
2. For RGB, choose an advertised resolution, pixel format and frame rate.
   RGB settings are preserved when switching to another channel; depth/IR
   use their fixed profiles and ignore these RGB-only fields.
3. Save. A running instance releases its old channel and starts the selected
   one. An idle instance stays idle until started. Invalid configuration is
   rejected before releasing a working channel.
4. **Refresh the page/reopen data details after changing channel.** The current
   Agent Core does not automatically refresh an existing card's cached output
   port and renderer after saving instance configuration. Use the topic and
   format returned by `info`; an already-open old-channel viewer is not changed
   by the driver. If the canvas still uses an old cached port, reopen the card's
   details to read its current output before opening the stream.

An instance publishes only its selected modality:

| channel | topic | format | payload |
| --- | --- | --- | --- |
| `rgb` | `/{namespace}/ext_camera/{instance_id}/rgb` | `image/jpeg` | Color JPEG |
| `depth` | `/{namespace}/ext_camera/{instance_id}/depth` | `image/depth-zlib` | zlib of 640x480 little-endian uint16 millimetres |
| `infrared` | `/{namespace}/ext_camera/{instance_id}/infrared` | `image/jpeg` | 640x480 left infrared Y8 encoded as grayscale JPEG |

Hyphens in instance IDs become underscores in ROS topics. Channel-specific
paths prevent a depth payload from being delivered to an old RGB subscription.
Existing downstream connections must be reviewed/reconnected for the newly
selected modality and format; they are not automatically rewired by the driver.

Multiple instances can select different channels of the same RealSense. The
stereo capture process is shared per physical camera and fans out to individual
instance topics. Stopping one instance leaves the others running; stopping the
last stereo instance releases the sensor. RGB uses its own V4L2 interface; two
RGB instances cannot simultaneously own the same video device.

Earlier experimental builds of this PR exposed `ext_depth`/`ext_infrared`.
Replace those with `ext_camera` instances configured with `depth`/`infrared`.
Those separate tool names and marketplace entries are no longer exported.

## What each modality is useful for

| channel | Useful question | Typical use |
| --- | --- | --- |
| RGB | What is visible, including color/text? | Remote inspection, input for OCR/object recognition |
| Depth | What is the depth at this image location? | Inspect nearby geometry, read pixel depth, inspect missing depth regions |
| Infrared | What intensity and texture does the stereo imager receive? | Diagnose depth quality; observe texture/target visibility under suitable illumination |

The most direct infrared use is alongside depth. When depth has holes or
unstable regions, inspect infrared for saturated highlights, poor texture,
occlusion or a poorly visible projector pattern. This helps investigate the
image input; the card does not implement an automatic diagnosis algorithm.

In weak visible light, available near-infrared illumination/projector light can
provide useful texture. The card does not add or control a light source. D400
stereo imagers also use visible light, so a bright room may resemble an ordinary
grayscale scene. The driver reads the native SDK `infrared` stream, index 1,
format `Y8`, rather than converting RGB into grayscale.

**D435i is not a thermal camera.** Brightness is reflected light intensity,
not temperature. Adding a palette would not turn it into a temperature
measurement. The left-only JPEG is for monitoring/algorithm input evaluation;
lossless capture, precision calibration or stereo matching requires additional
raw streams and calibration metadata. Recognition, navigation and complete
obstacle avoidance are downstream capabilities, not implemented by this sensor.

## Data and hardware constraints

- Depth uses the device's queried `get_depth_scale()` to convert to millimetres.
  Zero means invalid/unrepresentable. The payload is plain zlib of the pixel
  buffer, without a ROS compressedDepth transport header, matching the existing
  renderer. The view is not registered to RGB.
- Both modalities use `sensor_msgs/msg/CompressedImage` and best-effort QoS.
  Depth message format is `16UC1; compressedDepth zlib`; RGB/IR use `jpeg`.
- Stereo profiles are 640x480, 6fps on USB 2/unknown transport and 15fps on USB 3.
  `info` reports the actual selected profile, freshness and source stream/index.
  The fixed depth dimensions match the platform's headerless depth renderer.
- On the tested USB 2 D435i, RGB 720p/15 + depth/IR VGA/6 coexist. Stereo VGA/15
  or RGB 1080p/8 exceeded the tested concurrent bandwidth. USB 3.2 D435i switching
  and concurrent RGB 720p/15 + stereo VGA/15 have also been verified.
- Linux USB serial and RealSense SDK serial are not necessarily equal. Device
  selection binds SDK `physical_port` to the selected V4L2 node's USB ancestor;
  it does not choose an arbitrary first SDK camera or hard-code `/dev/video4`.
- Stereo start initially returns `starting`; only fresh published frames allow
  `running`. Missing devices/profiles, child-process exit and stale data report
  errors. Start retries after a fault. Shutdown is bounded if the SDK is stuck.

## Build and verification

The Dockerfile copies `realsense.py` and installs the official pinned
`pyrealsense2==2.56.5.9235` wheel in the existing dependency layer. Linux ARM64 /
CPython 3.10 availability and execution were verified. The SDK is needed for
calibrated depth scale and shared stereo acquisition. No Agent Core changes
are included in this driver PR.

```bash
python3 -m unittest discover -s tests
```

Tests cover real V4L2 capability formatting, unsupported formats, channel
configuration/topic changes, USB device binding, RGB compatibility, shared
instance lifecycle, stale/wrong-channel frames, depth units and overflow.
Hardware verification covers RGB→depth→infrared→RGB on the same instance,
concurrent instances and independent stop, plus actual decoded image payloads.

Sources: [driver contract](../../README_dev.md),
[SDK depth units](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0),
[SDK stream/format definitions](https://github.com/realsenseai/librealsense/blob/master/include/librealsense2/h/rs_sensor.h),
[D400/D430 FAQ](https://www.realsenseai.com/developers/faqs/), and
[optical filter discussion](https://dev.realsenseai.com/docs/optical-filters-for-intel-realsense-depth-cameras-d400/).
