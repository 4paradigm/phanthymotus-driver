# LinkerHand L20 Lite USB-RS485 driver

This is the active PhanthyMotus driver for the L20 Lite connected through a
USB-RS485 adapter. It uses Modbus RTU at 115200 baud, 8N1. The configured
right-hand slave address is `0x27`; the left-hand address is `0x28`. The operator
confirmed that the open/close targets below work on both hands.

The transport in `modbus_client.py` implements the frames emitted by the
official LinkerHand C++ SDK: function `0x04` reads ten 16-bit registers and
`0x10` writes ten registers. Position, torque and speed start at registers
`0x0000`, `0x000A` and `0x0014` respectively. Every value is `0..255` in the
low byte of a register. `main.py` uses `modbus_device.py` for the hand card.

Configure `serial_port` and `side` in `config.yaml`. The driver never moves the
hand on startup; only explicit MCP `hand` actions do. Every `open`, `close`
or `set_positions` request must include the boolean
`confirm_motion: true`; missing, false or string values are rejected before
opening the serial port. On the canvas, enter `true` in the confirmation field
after checking that the hand and its surroundings are clear. State reads,
torque configuration and torque release do not need motion confirmation.
The configured right-hand `open_pose` is
`[254,254,254,254,254,254,127,126,127,254]`; `fist_pose` is
`[93,255,0,0,0,0,128,128,128,255]`. The operator confirmed both poses on
their left and right hands, so this deployment explicitly sets
`left_open_pose` and `left_fist_pose` to the same respective values. The
driver selects poses after detecting the hand and rejects all motion,
including `set_positions`, if that side lacks explicit calibration. Detection
and state reads remain available. These calibrations must not be assumed safe
for other hardware. The official L20 Lite
example maps slots 7–10 to index sway,
middle sway, ring sway and thumb rotation; it has no separate pinky-sway slot.
These are 0–255 position commands, not degrees. Both
actions first read position, set speed/torque to the tested 60/100 values,
command a halfway pose, then command and poll the final pose. They leave the
configured torque enabled after success so the hand holds its position.
`stop` writes zero torque to all ten registers; shutdown does this if connected.
If a move times out or raises after a position command, the driver attempts to
release torque.

Motion actions use ACP: the card returns `state=running` and a unique
`action_id` immediately, then reports `completed`, `error` or `cancelled` to
Agent Core at `/api/acp/complete`. Do not repeat an action while it is pending.
The driver also rejects overlapping motions locally. `stop` interrupts a
pending motion and releases torque; if completion delivery fails, the driver
blocks further motion until it is inspected and restarted.

With `auto_detect_gestures: true` (the existing setting name), `info` performs
read-only position queries to addresses `0x27` and `0x28` on the configured
serial port and returns `detected_sides` and the selected `side`. `get_state`,
`open`, `close`, `set_positions`, and `release` each recheck the bus
and target the sole responding hand. Zero or two responding hands, a bad frame,
or a serial error prevents those actions from using an ambiguous target.
`set_torque` only changes later command settings and never writes
to the device immediately. The `side` setting remains a fallback when automatic
detection is disabled. The legacy `open_pose`/`fist_pose` keys calibrate the
right hand; `left_open_pose`/`left_fist_pose` calibrate the left hand. This
stage still scans only one configured serial port.
The canvas card name remains `hand`; its `info` result describes the detected
left/right side, but the card header does not live-rename on USB hot-plug.

For local tests without hardware, run this command from the repository root:

```bash
python3 -m pytest tests/test_linkerhand_l20_lite_modbus.py -q
```

With one powered hand connected to the configured serial port, run the tracked
hardware smoke script from this directory with the repository root on
`PYTHONPATH`. Stop the running driver container first so two processes do not
access the same RS-485 port. The script loads the calibrated `config.yaml`
(or `CONFIG_PATH`) and detects the responding hand. First read state without
motion:

```bash
PYTHONPATH=../.. python3 hardware_smoke.py --read-only
```

Only after checking that the hand and its surroundings are clear, optionally
move the index root by 20 position units and restore it:

```bash
PYTHONPATH=../.. python3 hardware_smoke.py --move-index-and-restore
```

This is a small one-joint smoke test, not full acceptance testing. It does not
test Docker, MCP HTTP or Agent Core registration.

The container requires access to the USB serial device. Its Compose service
already passes `/dev` into the privileged container. Do not run two hand
controllers against the same port simultaneously.

## Canvas card

`driver.yaml` declares one actuator card, `hand`. Once the driver is deployed
and registered, Agent Core discovers its `get_tool()` schema and the canvas
renders the card automatically; no custom frontend component is required.
The card title is **L20 Lite 灵巧手**. It offers read-only `info`, `get_state`,
`open`, `close`, `set_positions`, `set_torque` and `release`. `set_positions`
uses ten individual joint fields; fields left blank keep their current measured
values. The former CSV `positions`, `set_joint`, and `set_speed` controls are
not exposed or accepted. `start` and `stop` are canvas lifecycle actions, with
`stop` also releasing torque.
`set_torque` starts at the configured value of 100, affects later motions only,
and returns to the configured value when the driver restarts.

All three motion actions accept `speed_ratio` from `1.0x` to
`3.0x` and `hold_torque`. By default `hold_torque=true` preserves the existing
driver behavior; choose `false` to release torque after reaching the target.
Invalid options are rejected before serial access. Every motion still needs
`confirm_motion=true`. With automatic detection enabled, each motion action
checks for the sole responding hand on the configured serial port before
sending a motion command.
