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
built because the host does not already provide it.

The CAN decoder follows AgileX `ugv_sdk` v2 layouts. It reads status (`0x211`),
measured velocity (`0x221`) and raw wheel counts (`0x311`). It does not infer
position from undocumented wheel-count scaling. Startup never changes chassis
control mode. The `bunker_base` tool must explicitly request CAN mode with
`enable_can`; movement also requires fresh NORMAL/CAN feedback and is disabled
unless `BUNKER_MOTION_ENABLED=1` is deliberately set for commissioning.

The `motus.control/1` six-axis twist stream enters on the Agent Core domain and
is bridged to the private robot domain. State and `motus.odom/1` return through
the reverse bridge. Only measured forward and yaw velocity are declared;
unmeasured axes and pose stay `null`.

Build from the repository driver build system so `common/` is included in the
Docker context. Run protocol tests with `pytest agilex/bunker-pro/tests` and the
service contract check with `python3 scripts/check_service_yml.py agilex/bunker-pro`.
