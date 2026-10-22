# Deployment and real-canvas acceptance

As of 2026-10-22, a separate Agent Core and this driver are deployed as two
Docker containers on the NOS. The five-card layout is saved and repeated
MCP/API checks return fresh real data for all five cards. A 30-second Core
WebSocket check, three lifecycle rounds and new-reader interruption/recovery
checks have passed. The final uninterrupted 600.09-second Core DDS observation
passed: 599 state, 2946 joints, 2970 IMU, 2966 odometry, 576 cloud and 1194
lidar-info frames; every JSON frame was fresh and every cloud had 6000 points,
without parse errors. Browser control is disabled in Work
settings; visible point-cloud rendering, screenshots and refresh persistence
are not claimed as passed.

The persistent application directory is `/userdata/phanthy-motus-s10` and the
user-facing Core URL is `https://10.21.33.106:15678`. Credentials and access tokens
are kept outside this repository.

## Completed field checks

- All six Core WebSocket outputs passed a 30-second live-data check. State was
  approximately 1 Hz; joints, IMU and odometry were approximately 4.95 Hz;
  `lidar_info` was approximately 2 Hz. Lidar delivered 27 binary frames at
  approximately 0.90 Hz, 6000 points each, with a maximum inter-frame gap of
  3.01 seconds. JSON parse errors and unhealthy frames after warmup were zero.
  This verifies the Jazzy driver to Humble Core DDS-to-WebSocket path. The
  timestamp-free initial cloud cache was excluded. These are delivered output
  rates, not measurements of the vendor's original sampling rates.
- Three independent lifecycle rounds passed. Stopping each card stopped its
  DDS output while other running cards continued. After the final card stopped,
  the reader process count and all six reader subscriptions were zero. On each
  restart there was one reader process and one subscription per source.
- Suspending only the new reader child made all five cards stale. Lidar binary
  output stopped and stale metadata continued. Resuming the child recovered
  fresh data; terminating it triggered an automatic restart and fresh recovery.
  Vendor processes and robot networking were not interrupted.
- The 21-item HTTPS API audit passed, including 401 for unauthenticated access.
  Authentication is configured, automatic project startup is off, and the audit
  observed `project_running=false`. These checks do not imply any browser
  rendering or autonomous control was enabled.

Detailed working evidence is recorded outside the repository in the deployment
artifacts `core-ws-live-verification.json` and `lifecycle.json`. The final
600-second observation and real-browser checks remain separate acceptance items.

## Before applying anything

1. Confirm the robot Wi-Fi connection and NOS address before a subsequent
   deployment. Keep the existing Franka and PiPER deployments untouched.
2. Read current application services, free ports and writable persistent paths.
   Preserve the independent S10 application already deployed in
   `/userdata/phanthy-motus-s10`; do not repeat initial provisioning over its data.
3. Save an application-only inventory and any affected Compose/config/database
   backups privately. Do not put credentials or existing database copies in Git.
4. Run `preflight.py --public-contract` in the native NOS Jazzy environment.
   For a container deployment, compare its generated contract with the native
   contract before selecting that image. A package version comparison is not
   sufficient. The installed native message package is 1.0.23; comparison with
   the deployed container's public message definitions returned compatible.
   Repeat this check when replacing either message package or image.

## Supported application topologies

- **Container driver and Core on the same host:** the current field deployment.
  Core uses the existing Humble application image; the driver uses the Jazzy
  image `s10-pro-readonly:3a1cb138`, image ID
  `sha256:ffdaa0eb9187299028d63f2fde87de842cb2fe8e129e6c56bc10617de271dc28`.
  Runtime code remains at `3a1cb138`; the reader transport adjustment is an
  external configuration and read-only XML mount. Do not mount host Jazzy
  binaries into the Humble Core container.
- **Native NOS driver, separate Core container on the same host:** an optional
  development/diagnostic topology using the native Jazzy/drdds packages. Install
  only this repository's `common/` and `deep_robotics/s10_pro/` into a dedicated
  persistent application directory. The provided systemd unit is a template:
  review the user/path before installing it. It is not automatically installed
  or enabled by any project command.

The NOS currently has no Docker Compose plugin. The two running containers were
created and are managed with Docker CLI commands; the application's
`docker-compose.yml` persists the intended service configuration. Do not assume
that `docker compose` can be invoked on this host. `service.yml` remains the
normal platform installation template.

In both cases use the actual Core loopback DDS profile, domain 42, an independent
`s10_pro` namespace, and the free MCP port 15745. The child reader uses domain 0
and explicitly excludes the platform DDS profile. It never executes the vendor
`setup_ros2.sh`, changes XML, publishes robot commands or calls robot services.

For the container reader, install this application's `s10-reader-udp.xml` as a
real regular file, mount it read-only at
`/opt/phanthy-motus/s10-reader-udp.xml`, and set the external mounted config's
`reader.vendor_profile` to that container path. The field deployment maps
`/userdata/phanthy-motus-s10/s10-reader-udp.xml` to this path. The repository's
`service.yml` uses the normal `/opt/phanthy-motus` host installation prefix;
adjust only the host side of mounts for the independent field directory.

The child-only XML selects UDPv4 and disables builtin transports. The original
container configuration received drdds state but missed the local standard
IMU/odometry/cloud streams; adding host IPC alone did not restore them. A bounded
UDP-only container probe received all three, and this profile restored fresh
data through the actual driver. The final configuration uses private IPC; host
IPC is not required. Neither the vendor XML nor vendor services were changed.
The platform parent still uses the separate domain-42 loopback-only profile.

The separate S10 Core has its own persistent database and access configuration.
For any new installation, reuse the project's existing application image only
after checking architecture and standard message transport. Do not clone another
robot's live database, LLM secrets, canvas layout, switches or middleware state.
Keep smart control and automatic project startup disabled during read-only
acceptance. Preserve any existing application state if a platform is found.

## Remaining live and real-browser acceptance

Record the deployed source commit, config hash, image ID, Core ID/version and
user-openable URL. Then use the real browser to:

1. Inspect the saved five-card S10 Pro layout in the real browser. Confirm the
   standard existing card layout, settings, ports and controls. Browser access
   must be authorized before this step can be recorded as passed.
2. Start each card individually and the full project. Verify fresh real state,
   16 actual joint records, IMU and odometry values. Open the lidar point-cloud
   renderer and verify visible updating geometry as well as its metadata port.
3. Complete the separate uninterrupted ten-minute Core DDS observation; record
   delivered stream rates, maximum gaps and source ages. The card refresh rate
   is not the vendor sample frequency. Repeat lifecycle and interruption checks
   above if the image or runtime code changes.
4. Inspect point-cloud metadata alongside geometry. The renderer may retain its
   last frame; use metadata age and freshness to distinguish it from a live
   stream. Automated stale/recovery checks above do not inspect that display.
5. Refresh the browser and verify that layout/settings are preserved. Save
   screenshots and logs against the same deployment version. No motion test
   belongs to this five-card acceptance.

## Application-only rollback

Stop only the current `embodied-deep-robotics-s10-pro` driver container using
Docker CLI; do not invoke unavailable Compose tooling. If an
optional native deployment is used later, stop only its explicitly named driver
unit rather than a vendor service.
Verify the parent/reader processes have exited. Restore only the backed-up S10
application config/image and independent canvas state, checking for user edits
before restoring any database. A newly provisioned S10 Core can be stopped on
its own; preserve its persistent directory and previous image for recovery.
Do not restart vendor AOS/NOS services or the existing Franka/PiPER stacks.

No rollback command here disables robot motion or constitutes an emergency stop.

## Motion development boundary

The sixth actuator is implemented with a fail-closed production constructor.
Its real control backend is not bound and process authorization is false;
canvas configuration cannot change either. Tests use an explicitly fictional
backend and do not establish robot-side control authority or deadman behavior.
The S10 V1.0.2 codec and TLS transport use bounded I/O and strict validation.
The AOS endpoint was inspected without sending application frames. The reported
handheld version v2.5.5 still requires mapping to the robot system/control-server
version. Obtain the proper TLS trust/hostname material, authority/takeover and
loss-of-command stopping confirmation before binding production motion. No
stand, lie, velocity, mode switch, reset or emergency-stop command was sent.
