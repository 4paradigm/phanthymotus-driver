# Helios RGB camera interface

## Verified inputs

Five upper-body colour channels were independently captured and then read concurrently on the robot using this capture module. Every channel produced multiple JPEGs, increasing Driver receive timestamps and a decoded three-channel colour image. Workers were stopped and all child processes reaped. This sensor-only check did not register a development Driver, modify the canvas, change the running containers, or constitute bot-image acceptance.

| Channel ID | Position | Kernel device identity | Native capture | JPEG output |
|---|---|---|---|---|
| `chest_rgb` | chest | `vi-output, G300 color 9-006f` | YUY2, 1280×800, 30 fps | 640×400, 15 fps |
| `head_rgb` | head | `vi-output, G300 color 9-0067` | YUY2, 1280×800, 30 fps | 640×400, 15 fps |
| `head_front_fisheye` | head, front, fisheye | `vi-output, shw3h 10-0020` | UYVY, 1920×1536, 60 fps | 640×512, 15 fps |
| `head_rear_left_fisheye` | head, rear left, fisheye | `vi-output, shw3h 10-0021` | UYVY, 1920×1536, 60 fps | 640×512, 15 fps |
| `head_rear_right_fisheye` | head, rear right, fisheye | `vi-output, shw3h 10-0022` | UYVY, 1920×1536, 60 fps | 640×512, 15 fps |

G300 MJPEG capture did not produce a frame in the probe; raw YUY2 capture did. Native input rates are negotiated camera formats, not claims about measured delivery rate. Positions were confirmed on site by the operator (2026-11-22); left/right are from the robot's own perspective. Intrinsics, distortion, FOV and extrinsics are unknown; `motus.camera/1` declares `source=unknown` with the fixed output size (640×400 for G300, 640×512 for SHW3H), reported even before a card starts streaming. Receive timestamps are not manufacturer hardware sample times or synchronised timestamps across cameras.

## Chassis cameras are not included

The chassis read-only `getSysConf` response identifies two installed S10 cameras at locations -1 (front) and -2 (rear), both with `read_rgb=0`, `read_depth=1`, `read_cloud=1`. Extra slots with no device address are not cameras. This configuration was rechecked during development. It does not establish whether the installed hardware supports RGB. Neither a generic `rgb_fps` parameter nor the chassis UI's generic image viewer proves that it does.

The two S10 cameras are therefore not offered in the selector, and no frame, depth colourisation or copied upper-body image is published for them. Adding them needs a verified manufacturer RGB interface and, if necessary, an explicitly coordinated acquisition-enable test. This change delivers the five upper-body RGB streams only.

## Lifecycle and resource limits

- Standard GStreamer V4L2 capture runs only while at least one card requests a channel. It uses no manufacturer SDK, camera-reset/trigger script, GPU or robot-motion API.
- One producer per source; separate publishers and stable topics per card. Reconfiguring a running card preserves its topic. A failed camera switch preserves the previous selection and producer.
- A bounded first-frame wait (2.5 s; measured first frame ~0.76 s for G300 and ~0.26 s for SHW3H on the robot) rejects unavailable/busy cameras, so `start` returns within the ~3 s MCP budget. Subsequent failure reconnects with a delay; `info` reports state, age and frame count. The standard camera renderer retains its last image on interruption; consult card status rather than treating a retained picture as a new frame.
- Output is colour JPEG, `sensor_msgs/CompressedImage`, BEST_EFFORT / KEEP_LAST(1), `image/jpeg`. The same Core renderer used by G1 displays it with an FPS counter.
- Measured on the robot (bot image `release.261125.145d4d8`, 2026-11-25): one card reaches 15.5–15.8 fps on the G300 cameras; the SHW3H fisheyes (1920×1536 at 60 fps in) reached only about 9 fps with single-threaded scaling, so `videoscale` and `videoconvert` use `n-threads=4`, which reaches 15.5 fps. With all five cameras open at once the Orin is CPU-bound (each stream also has a kernel `vi-output` capture thread near one core), so not every stream holds 15 fps.
- One-buffer leaky input queue; frame dropping precedes scaling/conversion; 2 MiB maximum encoded frame, bounded stderr and bounded process termination. No recordings, stream server or extra listening port is created.
- Only five video nodes plus the Tegra VI channel nodes (`/dev/capture-vi-channel0..71`, required by the kernel VI driver to start capture inside a container) are mapped in Compose. Runtime sysfs identity must match the expected colour channel; the code refuses an ambiguous/missing match. No depth/IR/metadata device, global `/dev`, GPU or new host privileges are requested.

## Acceptance still required

Local contract and lifecycle regressions, plus short on-robot sensor-only capture, do not replace bot-image testing. Build the exact Driver image through this change's PR, deploy using the bot instructions, verify camera selection/switching/multi-card stop on the original canvas, and repeat joint/battery checks. User screenshots and Master/teacher acceptance remain required before merge. Camera images of the work area are local verification evidence and are not committed to Git.
