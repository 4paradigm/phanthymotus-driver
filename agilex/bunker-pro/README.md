# AgileX Bunker Pro v2

This driver uses the USB CAN adapter (`gs_usb`) on the Jetson host. The container
entrypoint first checks the attached adapter and the running host kernel. If the
module is absent, it builds it against the matching host headers, installs it
under `/lib/modules/<kernel>/extra`, runs `depmod`, and configures module loading
and a systemd unit for reboot persistence. Each container start reapplies the
500 kbit/s CAN link configuration, including after USB re-enumeration.

The service needs host networking, root for host setup, `SYS_MODULE`,
`NET_ADMIN`, `NET_RAW`, and the host module, headers, systemd, sysfs, and D-Bus
paths listed in `deploy/service.yml`. It does not use `privileged: true`. The
startup fails with a clear error if the adapter or host system D-Bus is
unavailable. Matching kernel headers are required only when `gs_usb` must be
built because the host does not already provide it. The container calls the
host systemd manager over the mounted system D-Bus socket with `busctl`; it does
not try to run `systemctl` against the container's non-systemd PID 1.

The host access is limited to the bootstrap lifecycle: `SYS_MODULE` loads the
matching `gs_usb` kernel module; `NET_ADMIN` configures the CAN link; `NET_RAW`
opens SocketCAN raw sockets. `SETUID` and `SETGID` let the entrypoint switch to
the unprivileged runtime account, and `SETPCAP` lets `setpriv` remove the
bootstrap capabilities. They are discarded before the driver starts.
`/lib/modules` is writable for the module and
`depmod` index, `/etc/modules-load.d` and `/etc/systemd/system` persist reboot
configuration, and `/usr/local/sbin` stores the helper called by the host unit.
`/usr/src` and `/sys` are read-only inputs (kernel headers and USB/interface
discovery); the system D-Bus mount is used to ask host systemd to enable and
restart that unit. Host networking provides the local Agent Core/DDS endpoints.
The container needs root to write the specific host bind mounts, but it does
not need all host capabilities or device access granted by privileged mode.
After setup, the entrypoint runs the ROS driver as UID 65534 with only ambient
`NET_RAW`, which SocketCAN needs to open raw CAN sockets. The running driver has
no `SYS_MODULE` or `NET_ADMIN` capability and cannot write the root-owned host
directories or access the root-owned system D-Bus socket. The container image
includes a compiler and kernel build helpers because a missing host `gs_usb`
module must be built against that host's matching headers; `python-can` is the
runtime SocketCAN dependency. These packages increase image size, but removing
the build tools would break the requested automatic bootstrap on hosts without
the module.

The CAN decoder follows the Bunker Pro 2.0 protocol tables. `motor_state`
combines chassis feedback (`0x211`), measured linear speed and steering angle
(`0x221`), and both motor high/low telemetry frames (`0x252`-`0x253`,
`0x262`-`0x263`) into flat fields. `battery` publishes BMS SOC, SOH, voltage,
current, temperature and alarm/warning flags (`0x361`-`0x362`), with chassis
voltage as a fallback. `remote_controller` exposes switch positions, stick
values and frame counter (`0x241`). No IMU frame appears in the Bunker Pro
interfaces you provided, so this driver does not advertise an `imu` card.

`odom` publishes the standard `motus.odom/1` format and separate scalar ports
for measured `vx`, steering angle and each wheel counter (`0x311`). The manual
defines wheel counters in millimeters; they are not integrated into pose. The
motion feedback does not contain yaw rate, so `wz` remains `null` rather than
being estimated from undocumented geometry.

`base_drive` exposes only a bounded `move` action and takes no `input_topic`.
`vx` is m/s and `vyaw` is degrees/s; the driver converts yaw rate to rad/s for
the CAN command. The move action requests CAN mode internally and verifies the
feedback before motion, so callers do not need a separate `enable_can` action.
The physical remote can still take priority; set its control switch as
described in the product manual. Motion requires fresh normal state, no critical
fault flags, and remains within configured speed limits. Software movement
permission defaults to enabled and can be disabled with
`BUNKER_MOTION_ENABLED=0`. A future servo card can separately consume a
navigation/control topic without changing `base_drive`.

Build from the repository driver build system so `common/` is included in the
Docker context. Run protocol tests with `pytest agilex/bunker-pro/tests` and the
service contract check with `python3 scripts/check_service_yml.py agilex/bunker-pro`.
