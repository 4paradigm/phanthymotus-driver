# Adam battery

The existing `battery` Sensor publishes JSON at 1 Hz on
`/{namespace}/state/battery`. `get` and `info` also return the same cached data
under `data`, keeping the existing `state` and `topic_out` envelope.
Neither query opens another connection or sends robot commands.

## Sources and compatibility

- DDS `rt/lowstate`: `timestamp_ms`, `voltage`, `current`, `power`,
  `wh_accumulated`, `status`, `source_topic` retain their previous meanings.
- PAC WebSocket `/robot_status/ws_battery`: `capacity` becomes `percentage`.
  The vendor battery page displays **capacity** as remaining charge; its raw
  `percentage` field is deliberately ignored. No voltage-based SOC estimate is used.
- PAC `mos_temp_dc`, `t1_temp_dc`, `t2_temp_dc` become
  `mos_temperature_c`, `t1_temperature_c`, `t2_temperature_c` in Celsius.
- PAC `cycle_count` is a nonnegative integer. `pstatus` is returned unchanged as
  `protection_status`. No temperature alarm thresholds or protection interpretations
  are invented. PAC electrical fields do not replace DDS electrical fields.

`percentage_available` indicates a valid, fresh PAC capacity. Missing/invalid optional
PAC fields become null. An invalid capacity rejects the message without refreshing
its receipt time. `dds_available` means a DDS sample has been received, not a
freshness guarantee; `dds_received_at_ms` preserves its receipt time. PAC can still
publish while DDS has no sample, in which case DDS electrical fields are null.

## Receiver and freshness

One background receiver is shared by battery topic publication and queries. It
starts/stops with the existing state plugin lifecycle, which the Driver starts and
closes. Queries only read the cache. The connection is receive-only, with bounded
connect/receive timeouts and automatic reconnect.

`pac_connected`, `pac_fresh`, `pac_age_ms`, `pac_last_received_at_ms`,
`pac_last_error` and `pac_source` expose receipt status. Age uses a monotonic clock;
receipt time uses the local wall clock. PAC does not provide a BMS sample timestamp:
these fields describe receipt of a valid PAC message, not BMS internal freshness.

On disconnect or expiry, **all PAC business fields become null**, including
`percentage`, and `percentage_available`/`pac_fresh` become false. Diagnostic timing
and the last error remain available. A reconnect needs a new valid sample before
reporting current readings. DDS fields remain available independently.

`plugins.state.battery_pac` configures `stale_after_sec` (default 10) and
`reconnect_sec` (default 2). Four live messages were observed about 1.001 seconds
apart, so the 10-second limit allows roughly ten missed updates. The PAC origin
is reused from the existing estop configuration, or the controller host;
`battery_pac.url` can explicitly override it without embedding an address in code.

## Verification

Run `python -m unittest discover -s pndbotics/adam -p 'test_battery*.py'` with Adam
requirements installed. Tests cover mapping, DDS compatibility, malformed messages,
missing fields, disconnect, expiry, reconnect and clean shutdown using mock sources.
Never stop PAC, disconnect cables or power down the robot to test expiry.

For live acceptance, compare `get` and the Canvas battery topic with the vendor
battery page: remaining charge, three temperatures, cycles and protection status.
Confirm `pac_fresh=true`, advancing receipt times and unchanged DDS field semantics.
This feature does not require a motion, control-domain switch or camera restart.

### Terminal acceptance (2026-09-29)

- 120 Adam tests and 4 repository Adam contract tests passed. Python syntax and
  config/marketplace/deployment YAML checks passed; the 16 battery tests also
  passed inside the ARM64 Docker build.
- The test image was layered on the currently deployed Adam image. Only the
  battery patch and its dependency were added, preserving unrelated deployment
  differences. The repository Dockerfile includes the new receiver for clean builds.
- Direct PAC probes returned capacity 97 while raw percentage remained 100,
  confirming the mapping matters. Later acceptance samples returned capacity 100,
  cycle count 13, approximately 53.4 / 51.3 / 51.6 Celsius and `Normal`.
  Values are observations, not constants.
- The operator's Canvas screenshot confirmed PAC fields reached the battery topic.
  Initial DDS fields were null, so the original and patched containers were tested
  under identical runtime settings before accepting the combined result.
- Both original and patched containers inherited DDS domain 0: each received zero
  lowstate messages and zero messages on the four related state topics in 12 seconds.
  A read-only probe on domain 1 received 4801 callbacks with valid BMS fields.
  The repository configuration already uses domain 1; only the deployed configuration
  needed correction. No robot control-mode change was made.
- After correcting the deployed domain to 1, the unchanged battery patch delivered
  both sources: approximately 45.90 V, 3.86 A, 177.17 W, 271.12 Wh accumulated,
  capacity 100, temperatures 53.0 / 50.5 / 51.0 Celsius, cycle count 13 and `Normal`.
  `dds_available` and `pac_fresh` were both true, with advancing receipt times.
- Final 12-second checks received 4802 direct DDS callbacks and 11 ROS battery
  messages; IMU, joints, motor and robot state topics also resumed continuous data.
  All 20 deployed Cards remained registered. Non-battery tool contracts were unchanged,
  restart count remained zero, arm command writes remained zero and hand control
  remained inactive. No movement was requested.
- The old container/image and a rollback script were retained on the development
  board. Final Canvas rechecking is performed by the operator; terminal verification
  of the combined DDS and PAC output is complete.

### Review follow-up: lifecycle and dependency contract (2026-10-08)

`websocket-client>=1.6,<2` remains the supported dependency range. The minimum
version **1.6.0** was tested with the battery suite, including real
`websocket.WebSocket` instances over local socket pairs. Its public `abort()`
interrupts a blocked receive; `close(timeout=...)` bounds the close handshake.
The battery tests also pass with **1.9.0**, including both shutdown APIs.
Both calls are retained. See the [1.6.0 implementation](https://github.com/websocket-client/websocket-client/blob/v1.6.0/websocket/_core.py#L430)
and the [review comment](https://github.com/4paradigm/phanthymotus-driver/pull/354#issuecomment-5889240785).
The fake socket uses explicit matching shutdown signatures instead of accepting
arbitrary keyword arguments. Local socket pairs exercise real framing and shutdown,
but do not test the HTTP upgrade handshake or the robot network.

Keep the range rather than introducing a one-off version pin. Record the resolved
version for each deployment and rerun the battery suite when upgrading it; test
1.6.0 as the compatibility floor. Before the final acceptance deployment on 2026-10-08, the running Adam image was
`release.260930.34902c1` and **did not have websocket-client installed**. Therefore
that previous deployment had no PAC client version to report and could not
validate this PR's combined DDS/PAC behavior. This observation does not replace the
2026-09-29 acceptance record above. The final acceptance deployment below supersedes that environment observation.

State shutdown attempts ROS publication deactivation, PAC shutdown and DDS polling
shutdown independently, logs each failure and reports an aggregate error. Close
also attempts ROS executor removal and node destruction even if stop fails. A PAC
abort failure does not skip joining its worker; a join timeout retains the live
thread reference. A socket close failure is logged and triggers immediate socket
shutdown before the reconnect loop continues. No threads are forcibly killed.

Run the commands below with the selected websocket-client version installed:

```sh
python -m unittest discover -s pndbotics/adam -p 'test_battery*.py'
python -m unittest discover -s pndbotics/adam -p 'test_*.py'
python -m unittest discover -s tests -p 'test_adam_driver.py'
```

### Two independent DDS domains

- **Robot SDK / CycloneDDS, domain 1:** receives raw robot data such as `rt/lowstate`.
  `main.py` passes `dds_domain_id` to `ChannelFactoryInitialize`, which creates an
  explicit SDK `DomainParticipant`.
- **ROS2 / PhanthyMotus, domain 42:** publishes the Driver's platform-facing ROS
  topics. ROS initializes separately and uses `ROS_DOMAIN_ID=42` from deployment.

The Driver transfers data between these independently initialized participants;
the different domain IDs are intentional. The 2026-09-29 correction to domain 1
concerned the robot SDK's `dds_domain_id`, not the global ROS domain. Neither domain
nor their initialization is changed by the lifecycle review fix.


### Final terminal acceptance (2026-10-08)

- Tested runtime revision: `7846959` (`fix(adam): harden battery PAC receiver
  lifecycle`). The [CI Build Result](https://github.com/4paradigm/phanthymotus-driver/pull/354#issuecomment-6051583292)
  associates this revision with image version `release.261008.0db3b03`.
  The image's `device.py` and `battery_status.py` hashes match the revision.
- Actual image dependency: **websocket-client 1.9.2**. Runtime introspection
  confirms `abort()` and `close(timeout=...)`. The closed-socket test checks safe
  termination rather than requiring a warning: 1.9.2 handles that socket error
  inside the client, while earlier tested versions may propagate it to our handler.
- Robot SDK DDS domain **1** and ROS domain **42** were verified independently.
  Over 12 seconds before restart, a separate read-only SDK subscriber received
  **4874 lowstate callbacks**; the battery topic received **12 messages**.
- Normal container stop took **0.883 seconds**, exited with code **0**, and produced
  no cleanup error or traceback in the stop interval. No forced thread termination
  or Docker kill was needed. A camera shutdown warning was also present.
- After normal container start, a second 12-second check received **4865 lowstate
  callbacks** and **12 battery messages**. IMU, joints and motor topics each
  received **413 messages**, and robot state received **414**. These topics do not
  all carry timestamps: changing payloads and robot ticks confirmed updates.
- DDS and PAC were simultaneously available after restart: approximately **44.43 V,
  3.95 A, 175.50 W, 110.95 Wh**, capacity **93**, temperatures **30.9 / 30.2 /
  30.8 Celsius**, cycle count **13**, protection **Normal**. `dds_available`,
  `pac_connected` and `pac_fresh` were true; `pac_age_ms` was **425** and
  `pac_last_error` was null. Direct PAC sampling returned capacity **93** versus
  raw percentage **100**, confirming the intended mapping. Values are samples,
  not constants. Container restart count remained **0**.
- **19 Cards** were registered, as expected by this revision. The previous local
  deployment exposed 20: its extra `arm_gesture` Card was already retired in the
  repository by commit `ed127ac`, independently of this battery PR. Unrelated
  non-battery query serialization tracebacks were observed before the stop test;
  these were not cleanup failures and are outside this acceptance's battery scope.
- Tests ran in a separate container from the **same deployed CI image**, with
  networking disabled and no robot device mounts. All **25 battery tests** and
  **4 Adam contract tests** passed. Full Adam discovery ran **129 tests with 3
  failures**, reproduced on a second run, all in existing `ArmWaveTests`:
  `test_a_new_wave_supersedes_the_running_one`,
  `test_handshake_reports_completion_after_elbow_oscillation`, and
  `test_stop_cancels_a_running_wave`. Running that group alone passed all **7
  tests**. This suggests test interaction; its cause is not established here.
  No arm implementation or tests were changed, and the full suite is not claimed
  to be green.
- The previous container and image were retained with a rollback script. No
  movement commands were issued. The operator confirmed that Canvas displays
  DDS/PAC values and all three availability/connection flags correctly after
  restart. The screenshot file has not yet been supplied for the evidence archive.
  Battery runtime acceptance passed; the full-suite failures remain a separate
  review caveat.
