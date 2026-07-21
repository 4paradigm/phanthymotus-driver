# Go1 Nano camera streams

`camera.py` is the Pi-side three-in-one visual card. Set its `type` to
`rgb`, `depth`, or `pointcloud`; each instance selects one of the five
positions below.

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
