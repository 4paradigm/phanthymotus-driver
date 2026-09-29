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
- MCP `get`/`info` and three consecutive ROS battery messages returned fresh PAC
  readings. All 20 deployed Cards remained registered; non-battery tool contracts
  and container deployment settings were unchanged. Restart count remained zero.
- DDS lowstate was unavailable **before** replacement and remained unavailable
  afterwards. Electrical fields consequently stayed null with `dds_available=false`.
  PAC functionality was verified independently; restoration of live DDS electrical
  feedback was not claimed or attempted by changing robot mode.
- The old container/image and a rollback script were retained on the development
  board. Browser/Canvas acceptance is performed by the operator.
