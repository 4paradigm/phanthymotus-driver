# External USB camera

`camera_front` uses the Go2's built-in video service. `ext_camera` discovers
USB V4L2 cameras attached to the host and publishes JPEG images for the canvas.
The sensor exposes no executable action form. Device, resolution, frame rate
and pixel format are configured through the instance settings; `config`,
`start`, `stop` and `info` remain internal lifecycle operations. V4L2
`set_*`/`get_*` control actions are no longer exposed or dispatched.

Previously, discovery skipped every device whose V4L2 information contained
`RealSense`. This also removed the external camera's color interface. With a
RealSense as the only USB camera, `info` returned `available_devices: []` and
`start` without an explicit device path failed with
`No external camera device available`, even though Linux and the container
could access the camera.

Discovery now checks each node's `Device Caps` and supported pixel formats.
Supported RealSense color interfaces appear with a `(Color)` suffix. Depth,
infrared and metadata nodes are excluded; this card does not provide depth or
point-cloud streams. A RealSense node whose format probe fails is not assumed
to be a color camera. Ordinary USB webcams retain their existing discovery path.

## Configure the card

After installing the fixed driver image, refresh the canvas and select the
RealSense `(Color)` entry in `ext_camera`. Choose a resolution and frame rate
supported by the connected camera, save the configuration, and start the card.
Stop and start an already running instance to apply changed capture settings.

On the Go2 tested on 2026-09-07, the color interface was `/dev/video4` and
supported `YUYV`, `1280x720` at 15 fps. Its `1920x1080` mode supported only
8 fps. These are measurements of that camera and USB connection, not universal
RealSense limits; discovery does not hard-code the `/dev/videoN` number.

## Validation

Run the discovery regression tests from the repository root:

```bash
python3 -m unittest discover -s tests -p test_go2_ext_camera.py
```

The tests cover the six-node RealSense layout observed on the robot, metadata
nodes with failed format probes, unprobed RealSense nodes, and ordinary USB
webcams. Hardware validation additionally confirmed automatic device selection
and 30 JPEG frames at `1280x720`, approximately 15 fps, through the existing
card's Agent Core WebSocket endpoint.
