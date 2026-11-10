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

The CAN decoder follows AgileX `ugv_sdk` v2 layouts. It reads status (`0x211`),
measured velocity (`0x221`) and raw wheel counts (`0x311`). It does not infer
position from undocumented wheel-count scaling. Startup never changes chassis
control mode. Software movement permission defaults to enabled and can be
disabled with `BUNKER_MOTION_ENABLED=0`. Before movement, call the
`bunker_base` tool action `enable_can`; it sends the CAN control-mode request
(`0x421`) and waits for chassis feedback. This does not move a physical remote
switch. If the chassis does not accept CAN mode while the remote has control,
set the remote as described in the Bunker Pro manual, then retry. Movement still
requires fresh NORMAL/CAN feedback and stays within the configured low speed
limits.

The `motus.control/1` six-axis twist stream enters on the Agent Core domain and
is bridged to the private robot domain. State and `motus.odom/1` return through
the reverse bridge. Only measured forward and yaw velocity are declared;
unmeasured axes and pose stay `null`.

Build from the repository driver build system so `common/` is included in the
Docker context. Run protocol tests with `pytest agilex/bunker-pro/tests` and the
service contract check with `python3 scripts/check_service_yml.py agilex/bunker-pro`.
