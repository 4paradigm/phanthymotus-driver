# Deployment and real-canvas acceptance

This checklist is pending while the user has intentionally disconnected the
robot. Offline tests and an HTTP registration are not real-canvas acceptance.

## Before applying anything

1. Reconnect the Mac to the robot Wi-Fi; confirm AOS/NOS still match the checked
   addresses. Keep the existing Franka and PiPER deployments untouched.
2. Read current application services, free ports and writable persistent paths.
   The previous NOS check did not find `/opt/phanthy-motus` or port 15678.
   Do not infer that the robot's complete container/application inventory is
   empty: the earlier sudo command required authentication and was not completed.
3. Save an application-only inventory and any affected Compose/config/database
   backups privately. Do not put credentials or existing database copies in Git.
4. Run `preflight.py --public-contract` in the native NOS Jazzy environment.
   For a container deployment, compare its generated contract with the native
   contract before selecting that image. A package version comparison is not
   sufficient. Native execution uses the already installed 1.0.23 package.

## Supported application topologies

- **Native NOS driver, separate Core container on the same host:** the preferred
  first validation path, using the existing native Jazzy/drdds packages. Install
  only this repository's `common/` and `deep_robotics/s10_pro/` into a dedicated
  persistent application directory. The provided systemd unit is a template:
  review the user/path before installing it. It is not automatically installed
  or enabled by any project command.
- **Container driver and Core on the same host:** build the Jazzy image, verify
  its interface contract against the native package, then use `service.yml`.
  Do not mount host Jazzy binaries into a Humble image.

In both cases use the actual Core loopback DDS profile, domain 42, an independent
`s10_pro` namespace, and the free MCP port 15745. The child reader uses domain 0
and explicitly excludes the platform DDS profile. It never executes the vendor
`setup_ros2.sh`, changes XML, publishes robot commands or calls robot services.

Provision a separate S10 Core with a clean persistent database and its own access
token/certificate. Reuse the project's existing application image only after
checking architecture and standard message transport. Do not clone another
robot's live database, LLM secrets, canvas layout, switches or middleware state.
Keep smart control and automatic project startup disabled during read-only
acceptance. Preserve any existing application state if a platform is found.

## Real browser acceptance (not yet performed)

Record the deployed source commit, config hash, image ID, Core ID/version and
user-openable URL. Then use the real browser to:

1. Find and drag the five cards from the S10 Pro driver into a dedicated canvas.
   Confirm the standard existing card layout, settings, ports and controls.
2. Start each card individually and the full project. Verify fresh real state,
   16 actual joint records, IMU and odometry values. Open the lidar point-cloud
   renderer and verify visible updating geometry as well as its metadata port.
3. Observe ten minutes of live data; record source receive rates, maximum gaps,
   ages and process CPU/memory. The card refresh rate is not the vendor sample
   frequency. Actual point limits and throughput must be measured.
4. Stop/restart three times. Check that a stopped card no longer publishes, other
   cards continue, and the reader exits when the final card stops. Track process
   and subscription counts, not only the button appearance.
5. Interrupt only the new reader process. Verify explicit stale/no-data status
   and recovery without stopping vendor processes or switching robot networking.
   The point renderer may retain its last frame: use the metadata freshness
   indicator to distinguish a retained frame from a live stream.
6. Refresh the browser and verify that layout/settings are preserved. Save
   screenshots and logs against the same deployment version. No motion test
   belongs to this five-card acceptance.

## Application-only rollback

Stop only `s10-pro-driver.service` or the `deep-robotics-s10-pro` Compose service.
Verify the parent/reader processes have exited. Restore only the backed-up S10
application config/image and independent canvas state, checking for user edits
before restoring any database. A newly provisioned S10 Core can be stopped on
its own; preserve its persistent directory and previous image for recovery.
Do not restart vendor AOS/NOS services or the existing Franka/PiPER stacks.

No rollback command here disables robot motion or constitutes an emergency stop.
