# PiPER Health card

`piper_health` is an independent **sensor**, titled **Health**, publishing a
Chinese key/value panel at **2 Hz** on `/<namespace>/piper/state/health`
(`data/json`; the exact namespace is returned by `info.topic_out`).

## First version

- Shared connection state and measured six-axis feedback freshness (0.5 s).
- Control mode, six individual motor-enable flags, controller/teaching state,
  raw fault code and per-motor fault flags.
- Deployment motion permission, driver task activity and Arm control lifecycle.
- A list of unmet or unknown prerequisites for **direct arm control**, including
  current joint limits. No TCP, obstacle, target, trajectory or workspace check
  is implied. Arm control retains its own automatic preparation and validation;
  an all-disabled arm can be preparable while this conservative diagnosis is
  blocked. Health never performs that preparation.

Missing, stale, future-dated or malformed feedback never becomes a ready result.
When feedback expires, motor enables and fault values are marked unavailable,
not replayed as current or replaced with zeros. Temperature/current/voltage and
firmware querying are outside this first version. Raw controller codes are kept;
the card does not guess an undocumented cause or clear a fault.

## Canvas and API

Add **Health** from the PiPER bundle and start it. Open its data stream or the
top Monitor to see the Chinese panel. The existing Joints card remains the
owner of the skeleton stream; there is still no `state/parameters` stream.

MCP `piper_health` accepts only `start`, `stop`, `info` (or an empty sensor query).
`info.health` returns `schema: piper.health/1`, timestamps, `status`,
`ready_for_direct_control`, `checks`, `blockers` and the Chinese `display` map.
Stable check IDs are `connection`, `feedback`, `faults`, `mode`, `teaching`,
`enabled`, `limits`, `permission`, `control_card`, `idle`. A check value of `null`
means unknown. The DDS panel is the flat `display` map.

Only the existing runtime's passive snapshot is used. The card never calls
`connected()` or `status()`, opens CAN/camera, changes permission or mode,
enables motors, clears faults, or moves/stops the arm. Starting it starts only
its publisher. Stopping it closes only its publisher. The existing shared
receiver must already be available for live feedback (normal driver startup
starts the existing Joints listener). No extra hardware dependency is added.

## Verification

Run from the repository root, using an environment with PyYAML:

```bash
PYTHONPATH=.:tests python3 -m unittest test_piper_health test_piper_unified test_piper_deployment test_piper_monitor
python3 scripts/check_service_yml.py agilex/piper
```

The health tests use synthetic snapshots and a recording SDK: ready, offline,
stale, malformed, fault, partial enable, busy, permission, limits, independent
start/stop, late callbacks and failures/recovery. The four modules above passed
114 tests. The service configuration check also passed.

Local integration used this checkout as a read-only source mount in the PiPER
container. The canvas discovered Health, displayed the expected disconnected
state, and received three DDS frames at an average 0.5-second interval. A
Health-to-decision_core data connection was saved, and the running decision
core produced a disconnected-device notice from that input. Only the PiPER
container was recreated; the other local service containers were preserved.

These checks establish software integration and offline behavior, not physical
robot acceptance. No actual CAN device was connected and no physical robot
motion was tested. Real connection, six-axis feedback, fault interpretation and
hardware disconnect/recovery still require supervised on-device verification.
A deployment must use this source revision/image before Health appears.
