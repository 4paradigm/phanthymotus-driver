# Orbbec 2.5.4 independent IMU capture clock

`orbbec-2.5.4-imu-clock.patch` fixes `onNewIMUFrameCallback` in the
[official v2.5.4 source](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2.5.4).
The independent accel/gyro callback bypassed `time_domain` and used the device
clock. RGB-D used `getFrameTimestampUs`, which selects device/global/system
capture timestamps. The patch uses that same selector for IMU and its info
header. It does not change sample values, timestamps at receipt, rates, TF,
or the synchronized gyro+accel callback. Keep `time_domain=global` and the
existing `enable_sync_host_time=true` source gate for this Tianyi setup.

The camera service runs on the host, outside the Motus driver image. Updating
only the driver cannot apply this vendor fix. Build and deploy the camera
library separately; do not replace the robot driver to install it.

## Tested build on Tianyi (Jetson / Humble)

The installed package reports OrbbecCamera `a351a5b6`, ROS package 2.5.4,
SDK 2.5.5. Its `ob_camera_node.h` and SDK binary match the official v2.5.4
archive. The SDK SHA256 is
`7d74cde4843f79fc8bf98b714a795e76633a20c7987f72c0febb7fdfd2605cf6`.
Use the matching source/ABI; a different vendor version needs separate review.
No new dependency packages were needed on the tested host.

Run in a new scratch directory, with this patch available as an absolute path:

```bash
curl -fL https://codeload.github.com/orbbec/OrbbecSDK_ROS2/tar.gz/refs/tags/v2.5.4 -o source.tar.gz
# Tested archive SHA256: 1942559517b7b16e5ad0005b874e65c39a6cbe4cf7bb11a76e2f7092accb1785
mkdir src
tar -xzf source.tar.gz -C src
cd src/OrbbecSDK_ROS2-2.5.4
patch --dry-run --fuzz=0 -p1 < /absolute/path/orbbec-2.5.4-imu-clock.patch
patch --fuzz=0 -p1 < /absolute/path/orbbec-2.5.4-imu-clock.patch
cd ../..
source /opt/ros/humble/setup.bash
source /home/nvidia/orbbec_camera_ros2/install/setup.bash
cmake -S src/OrbbecSDK_ROS2-2.5.4/orbbec_camera -B build \
  -DBUILD_TESTING=OFF -DINSTALL_UDEV_RULES=OFF \
  -DUSE_NV_HW_DECODER=OFF -DUSE_RK_HW_DECODER=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build --target orbbec_camera -j2
```

This host's installed `orbbec_camera_msgs` export refers to the cross-build path
`/opt/sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so`. For a native build,
the real library is `/usr/lib/aarch64-linux-gnu/libpython3.10.so`. If that link
error occurs, insert the following after the dependency `endforeach()` in the
**scratch source** CMakeLists.txt and rerun configure/build. Leave installed
ROS/message exports unchanged.

```cmake
get_target_property(_native_imu_links orbbec_camera_msgs::orbbec_camera_msgs__rosidl_generator_py INTERFACE_LINK_LIBRARIES)
string(REPLACE "/opt/sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so" "/usr/lib/aarch64-linux-gnu/libpython3.10.so" _native_imu_links "${_native_imu_links}")
set_property(TARGET orbbec_camera_msgs::orbbec_camera_msgs__rosidl_generator_py PROPERTY INTERFACE_LINK_LIBRARIES "${_native_imu_links}")
```

## Deployment and rollback

Copy the original installed `orbbec_camera` package, dereferencing symlinks, to
an isolated overlay prefix. Replace only its `lib/liborbbec_camera.so` with the
built library; retain the installed SDK, resources and launch files. A wrapper
must source the original ROS/Orbbec environments, prepend the overlay package
to `AMENT_PREFIX_PATH` and its `lib` to `LD_LIBRARY_PATH`, and preserve the
original DDS profile. Check package resolution and shared
library dependencies before switching the camera service. This is a manual
host operation, not an automatic driver startup action.

On 2026-11-12 the test overlay was installed at
`/home/nvidia/orbbec-imu-fix-20261112/overlay/orbbec_camera` and selected using
`/etc/systemd/system/orbbec_head.service.d/90-motus-imu-clock.conf`.
The wrapper is `/home/nvidia/orbbec-imu-fix-20261112/run-camera.sh`.
The test wrapper selects the following native camera profile to reduce
RGB-D transport load (the computation card already consumes 640×480 images):

```text
gyro_rate:=100hz accel_rate:=100hz
color_width:=640 color_height:=480 depth_width:=640 depth_height:=480
```

Keep the launch's 30 Hz image rate. Intrinsics are read again from the vendor
CameraInfo after a source restart; do not reuse calibration from the previous
1280×720 profile. This profile also changes the native pointcloud resolution.
The older 1280×720 / 200 Hz profile had verified raw IMU clock timestamps after
the patch, but did not pass the Python pipeline's continuity checks under load.

Only `orbbec_head.service` was restarted; original installed files and active
Motus service containers were preserved. To roll back this test deployment,
remove **only** that drop-in, run `systemctl daemon-reload`, then restart
`orbbec_head.service`. Do not remove unrelated drop-ins.

Check independent gyro/accel freshness, monotonic capture stamps, RGB-D output,
and `camera_rgbd.info.imu_status` after switching. Then connect the IMU port
alongside the RGB-D pair and enable `rgbd_odometry.imu_assisted`. Camera IMU
extrinsics come from the camera's internal TF; absent body/head TF remains a
separate limitation and must not be filled with an identity transform.
