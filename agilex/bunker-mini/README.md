# AgileX Bunker Mini v2

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

The bootstrap runs as host-root-equivalent trusted code. Before dropping
privileges, the image has `SYS_MODULE`, writable host module and persistence
directories, and unrestricted access to the host system D-Bus socket. A
compromised image or bootstrap command can load a malicious kernel module,
replace host startup files, or request arbitrary operations from host systemd.
This is a host compromise boundary, not a sandboxed or narrowly scoped setup
interface. Only deploy a reviewed, trusted image on a host whose operator
accepts that risk. `SETUID` and `SETGID` let the entrypoint switch to the
unprivileged runtime account, and `SETPCAP` lets `setpriv` remove the bootstrap
capabilities; they are discarded before the driver starts, but this does not
reduce the bootstrap-time trust requirement.
`/lib/modules` is writable for the module and
`depmod` index, `/etc/modules-load.d` and `/etc/systemd/system` persist reboot
configuration, and `/usr/local/sbin` stores the helper called by the host unit.
`/usr/src` and `/sys` are read-only inputs (kernel headers and USB/interface
discovery); the system D-Bus mount permits host systemd manager calls. Host
networking provides the local Agent Core/DDS endpoints. The container does not
use Docker `privileged` mode, but these mounts, capabilities, and D-Bus access
still allow host-level compromise during bootstrap.
After setup, the entrypoint runs the ROS driver as UID 65534 with only ambient
`NET_RAW`, which SocketCAN needs to open raw CAN sockets. The running driver has
no `SYS_MODULE` or `NET_ADMIN` capability and cannot write the root-owned host
directories or access the root-owned system D-Bus socket. The container image
includes a compiler and kernel build helpers because a missing host `gs_usb`
module must be built against that host's matching headers; `python-can` is the
runtime SocketCAN dependency. These packages increase image size, but removing
the build tools would break the requested automatic bootstrap on hosts without
the module.

The Dockerfile's apt additions each support this bootstrap/runtime:
`build-essential` compiles `gs_usb` only when the host lacks it; `kmod` provides
`modinfo`, `modprobe`, and `depmod`; `iproute2` provides `ip` for CAN link setup
and verification; `systemd` supplies `busctl` so the container can request the
host systemd unit over D-Bus (the container itself does not run systemd); and
`util-linux` supplies `setpriv` to drop to the unprivileged driver identity.
The build depends on the deployed host's matching kernel headers, so moving the
compiler into a separate Docker build stage would not build the host-compatible
module. `python3-pip` installs the pinned `python-can` and `PyYAML` dependencies.
The `ros-base` image contract does not guarantee these utilities, so the
component installs them explicitly. `--no-install-recommends` and apt-list
cleanup limit extra packages and metadata, but these tools do increase image
size.

The CAN decoder follows the Bunker Mini 2.0 protocol tables. `motor_state`
combines chassis feedback (`0x211`), measured linear speed and steering angle
(`0x221`), and both motor high/low telemetry frames (`0x252`-`0x253`,
`0x262`-`0x263`) into flat fields, with raw fault flags and lists of active faults instead of
individual false-valued flags. `battery` publishes BMS SOC, SOH, voltage,
current, temperature and alarm/warning flags (`0x361`-`0x362`), with chassis
voltage as a fallback. The `source` field distinguishes BMS data from chassis
voltage; unavailable battery measurements are omitted. `remote_controller` exposes switch positions, stick
values and frame counter (`0x241`). No IMU frame appears in the Bunker Mini
interfaces you provided, so this driver does not advertise an `imu` card.

`odom` publishes a single `state/odom` output in standard `motus.odom/1`
format, compatible with ActuCore navi. Steering angle and wheel counters
(`0x311`) are included in the same message under `vendor`. The manual
defines wheel counters in millimeters; they are not integrated into pose. The
motion feedback does not contain yaw rate, so `wz` remains `null` rather than
being estimated from undocumented geometry.

`base_drive` exposes only a bounded `move` action and takes no `input_topic`.
`vx` is m/s and `vyaw` is degrees/s; the limits default to 1.0 m/s and 60
degrees/s. If the frontend omits either velocity argument, it defaults to zero.
The driver converts yaw rate to rad/s for the CAN command. The move action
requests CAN mode internally and verifies the feedback before motion, so callers
do not need a separate `enable_can` action.
The physical remote can still take priority; set its control switch as
described in the product manual. Motion requires fresh normal state, no critical
fault flags, and remains within configured speed limits. Software movement
permission defaults to enabled and can be disabled with
`BUNKER_MOTION_ENABLED=0`.

`bunker_servo` consumes ActuCore navi's `control/velocity` output. Connect navi's
command output to this card and connect the single `odom` output to navi's
`state/odom` input. The servo's `info` and `start` responses advertise its
`motus.control/1` descriptor for Agent Core negotiation. Its six-axis body
twist is `[vx, vy, vz, wx, wy, wz]`, in m/s and **rad/s**; only `vx` and `wz`
are supported, with the other axes pinned to zero. This stream uses rad/s even
though the human-facing `base_drive.move.vyaw` uses degrees/s.

The subscription runs in the Agent Core DDS domain with BEST_EFFORT QoS and a
one-message queue. `ControlSink` checks the schema, timestamps/observation age,
sequence/source arbitration and limits, and caps each command step to 0.1 m/s
and 0.2 rad/s. Both the sink watchdog and the CAN target lease expire after
300 ms without accepted commands. Fresh chassis state, CAN mode and critical
fault checks still apply. `pause` stops the servo while keeping its subscription;
`resume` rechecks the chassis and creates a fresh sink. `stop` removes the
subscription. A direct `base_drive.move` pauses the servo and leaves it paused
after completion; resume explicitly to let navigation take control again.
Servo start/resume is refused while a direct move is active.

No footprint or minimum usable speed has been measured on this chassis, so
the descriptor does not invent these values. The odometry reports measured
forward velocity only; navigation has no measured yaw-rate feedback.

Build from the repository driver build system so `common/` is included in the
Docker context. Run protocol tests with `pytest agilex/bunker-mini/tests` and the
service contract check with `python3 scripts/check_service_yml.py agilex/bunker-mini`.
