# BrainCo Revo 2 driver

MCP driver for the BrainCo Revo 2 dexterous hand — a standalone 6-active-joint
(11-DOF) hand connected directly to the host over RS485 or CAN FD (no robot-side
DDS network of its own). Unlike the humanoid/quadruped drivers in this repo, this
driver only exposes what a bare hand actually offers — no locomotion, no arm, no
whole-body state.

## Cards

- **`hand`** (actuator) — direct finger control: `get_state`, `get_device_info`,
  `open`, `close`, `set_position` (single finger), `set_positions` (all 6),
  `run_gesture` (preset gestures: open/fist/pinch_two/pinch_three/pinch_side/point),
  `calibrate`, `reset_gesture`, `set_led`, `get_button_event`.

  **This is an actuator that drives real motors.** Per this repo's safety rule
  (see root `CLAUDE.md`), never call any `hand` action against real hardware
  without first describing the exact motion and getting explicit confirmation.

- **`hand_state`** (sensor) — background poll (`poll_interval_s`, default 0.15s)
  publishing per-finger position/motor-state/voltage as `data/json` to
  `/{namespace}/revo2/hand_state`.

- **`hand_touch`** (sensor) — only loaded when `config.yaml`'s `variant: touch`.
  Publishes tactile sensor data (`data/json`) to `/{namespace}/revo2/hand_touch`.
  The reader used (`array_pressure` / `force3d` / `pressure` / `capacitive`) is
  selected by `touch_vendor`, since the SDK exposes each touch sensor family as a
  genuinely different API, not one unified "touch" mode.

- **`model`** (resource) — static spec sheet (`resource/revo2_info.json`): DOF
  table, joint angle ranges, variant matrix, motor IDs/states, control modes. No
  URDF/skeleton rendering in this version — see "Not in this version" below.

## Position convention

The SDK exposes a single unified range regardless of protocol or variant:
**0 = fully open, 1000 = fully closed**, for every finger. `open`/`close` use the
`open_positions`/`close_positions` arrays in `config.yaml` (default `[0]*6` /
`[1000]*6`) rather than a hardcoded direction, in case a specific unit needs a
different convention.

## Variants

| Variant | Code | Interfaces | Touch |
|---|---|---|---|
| Basic | XRL/XRR | RS485, CAN FD | No |
| Pro | XEL/XER | RS485, CAN FD, EtherCAT | No |
| Touch | XTL/XTR | RS485, CAN FD, EtherCAT | Yes (pressure/array_pressure/force3d/capacitive, pick one via `touch_vendor`) |

Set `variant` in `config.yaml`; the `hand_touch` card only activates for `touch`.

## Connection

`protocol: modbus` (RS-485) or `protocol: canfd` (SocketCAN CAN FD) — both use
the same `bc-stark-sdk` PyPI wheel (`pip install bc-stark-sdk==2.0.2`, no git
clone / no vendoring needed, see `reference/get_sdk.md`). **EtherCAT is not
supported by this driver version** — it needs the IgH EtherCAT Master installed
on the host plus elevated capabilities, a much bigger dependency than RS485/CANFD;
add it as a follow-up once there's Pro/Touch hardware to validate against.

For RS-485, ensure the serial device is in the `dialout` group and has
`low_latency` enabled (`sudo setserial /dev/ttyUSB0 low_latency`). For CAN FD,
bring the interface up before starting the driver:

```bash
sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
```

One or two hands can share the same bus — list them under `hands:` in
`config.yaml` with their `side` and `slave_id` (factory default: left=126,
right=127). Every `hand`/`hand_state` action takes an optional `side` argument
to pick which configured hand it targets; it defaults to the first entry.

**A power-on calibration (`calibrate` action) is required once before the hand
will respond to position commands** — the docs warn all fingers open during
calibration, so don't calibrate while the hand is holding something.

## Not in this version

- EtherCAT protocol support (needs IgH EtherCAT Master on the host).
- Hand skeleton (`sensor/skeleton`) rendering — would need vendoring
  `brainco_hand_ros2`'s URDF/mesh assets and verifying joint sign/direction on
  real hardware first; `hand_state` uses `data/json` instead for now.
- Decoded button gestures (short-press=reset / long-press=factory-reset /
  double-press=packing-gesture) — the SDK only exposes raw press/release events
  (`get_button_event`); that debounce/gesture logic lives in firmware, not the
  API, so this driver doesn't invent one.
- Factory-only calibration/backlash/stall-current tuning (`factory_*` SDK
  methods) — out of scope for a general-purpose driver.

## Reference docs

`reference/` holds raw captures of the official docs
(`brainco-hz.com/docs/revolimb-hand/revo2/`) plus the full Python type stub
(`main_mod.pyi`) and C header (`stark-sdk.h`) that ground this driver's API
usage — useful when extending the driver (e.g. adding EtherCAT or skeleton
support later).

## Verification status

No physical Revo 2 unit was available while writing this driver. What's
verified: config/schema sanity, dispatch-logic unit tests against a fake SDK
(`tests/test_device.py`), and that the MCP HTTP server starts even without
`bc_stark_sdk` installed (connection retries in the background instead of
crashing). Open/close position direction, actual `slave_id`s, and the exact
`bc-stark-sdk` wheel compatibility on JetPack 5.11 (Orin 5, glibc 2.31 vs. the
wheel's `manylinux_2_34` floor) still need confirming on real hardware.
