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
hand on startup; only explicit MCP `hand` actions do. Every `open`, `close`,
`reset` or `set_positions` request must include the boolean
`confirm_motion: true`; missing, false or string values are rejected before
opening the serial port. On the canvas, enter `true` in the confirmation field
after checking that the hand and its surroundings are clear. State reads do
not need motion confirmation. The lifecycle `stop` action remains available
to interrupt movement and release torque without commanding a new pose.
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
ring sway, pinky sway and thumb rotation; it has no middle-sway slot.
These are 0–255 position commands, not degrees. `open`, `close` and `reset`
first read position, set speed/torque, command a halfway pose, then command
and poll the final pose. `open` and `close` accept a per-call `set_torque` of
1..255 (default 100); it does not change later actions. They retain that
torque setting after reaching the target. `reset` opens to the calibrated pose
and then sets all joint torques to 0, including when already open.
`stop` writes zero torque to all ten registers; shutdown does this if connected.
If a move times out or raises after a position command, the driver attempts to
release torque.

Motion actions use ACP: the card returns `state=running` and a unique
`action_id` immediately, then reports `completed`, `error` or `cancelled` to
Agent Core at `/api/acp/complete`. Do not repeat an action while it is pending.
The driver also rejects overlapping motions locally. `stop` interrupts a
pending motion and releases torque; if completion delivery fails, the driver
blocks further motion until it is inspected and restarted.

The tool is multi-instance: drag `hand` onto the canvas once per physical hand,
then open each card's instance configuration and select `bound_side: left` or
`right`. Do not bind two cards to the same side. The driver scans the configured
serial port plus connected `/dev/ttyUSB*` and `/dev/serial/by-path/*` devices,
deduplicates their paths, and probes `0x27` and `0x28` with read-only frames.
Each bound card follows its selected side even if USB port numbering changes;
it refuses movement when that side is missing or detection is ambiguous. Before
writing a motion command it probes the selected slave again. `info` reports the
bound side, detected sides, count, and current serial port; the canvas card
header itself does not live-rename.

An existing single card without instance configuration retains the previous
one-port behavior: with `auto_detect_gestures: true`, actions require exactly
one responding hand on `serial_port`. Once any card is explicitly bound, an
unbound card cannot move or read state until it is also bound. The legacy
`open_pose`/`fist_pose` keys calibrate the right hand;
`left_open_pose`/`left_fist_pose` calibrate the left hand.

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

`driver.yaml` declares one actuator tool, `hand`, which can have two canvas
instances. Once the driver is deployed and registered, Agent Core discovers its
`get_tool()` schema; no custom frontend component is required. Open each
card's instance configuration (gear icon) and select its hand before starting
the project. A saved binding is replayed by Agent Core on start and action
calls. Saving a binding does not move the hand.
The card title is **L20 Lite 灵巧手**. It offers read-only `info`, `get_state`,
`open`, `close`, `reset` and `set_positions`. `set_positions`
uses ten individual joint fields; each accepts an integer 0–255 protocol
position value (not degrees), and fields left blank keep their current
measured values. The range is a wire-level limit, not a guarantee that every
target is physically safe. Values outside 0–255 (or non-integers) are rejected
before any serial command is sent. Slots 8 and 9 are `ring_sway` and `pinky_sway` per
the official L20 Lite map; older canvas workflows using `middle_sway` or the
previous `ring_sway` slot must be reviewed before reuse. The former CSV
`positions` field and the `set_joint`, `set_speed`, standalone `set_torque`
and `release` actions are not exposed or accepted. `start` and
`stop` are canvas lifecycle actions, with
`stop` also releasing torque.
`open` and `close` accept a per-action `set_torque` integer in 1–255 (default
100), a raw torque limit rather than a lock or a value in N·m. They and
`set_positions` accept `speed_ratio` from `1.0x` to `3.0x`. All three retain
the commanded torque after reaching the target, including when already at
the target. `reset` uses the calibrated open pose and always releases torque
at the target. The former `hold_torque` parameter is no longer accepted;
update saved canvas workflows that pass it. `stop` releases torque immediately.
Invalid options are rejected before serial access. Every motion request must
carry the boolean `confirm_motion=true` (not the string `"true"`); otherwise no
motion is accepted. If the canvas preserves a previous true value, the operator
must still recheck that the hand and its surroundings are safe before running
another motion. With automatic detection enabled, each motion action
checks for the sole responding hand on the configured serial port before
sending a motion command.

## Separate `hand_gesture` card (local development)

`driver.yaml` also registers `hand_gesture` as an independent actuator card.
It adapts six motions from the 2026-10-19 colleague prototype without
changing the existing `hand` actions: `ok_gesture`, `thumb_up_gesture`
(the prototype named this `thumbs_up_gesture`), `v_sign_gesture`, `number`,
`grasp`, and `custom_grasp`. Both hands were physically checked by the
operator, and `config.yaml` keeps an explicit pose table for each side.
Do not use those tables on a different hand without recalibration.

Drag `hand_gesture` onto the canvas, choose `bound_side` in its instance
configuration, and use `info` to verify the detected hand and USB path.
With only one connected hand, an unbound card can auto-select it; with zero
or two hands it requires a binding. Each motion request requires the boolean
`confirm_motion: true` after checking that the hand and its surroundings are
clear. Missing confirmation or an uncalibrated side is rejected before a
position command. Motion returns `state=running` and an `action_id` and reports
completion through ACP. `stop` cancels a gesture and releases torque.

`number` requires selecting one integer from the 0–9 dropdown and moves to
that number's calibrated pose; it does not run a counting sequence.
`grasp` offers `soft`, `standard`, and `firm` with raw
torque limits 30, 85, and 160; `custom_grasp` accepts a raw 1–255 limit.
The optional `speed_ratio` is 1.0x–3.0x, capped at 255. The three simple
gestures and `number` release torque at completion unless `hold_torque=true`; grasp holds
unless `hold_torque=false`. Contact detection is inferred from position
stall only: there is no tactile sensor or verified force measurement, so
these options do **not** guarantee an object will not be squeezed or dropped.

The new gesture implementation lives in `gesture_device.py` and shares the
existing driver's USB discovery and serial bus lock. The hand card's
`modbus_device.py` implementation and its original configuration remain
unchanged apart from loading this additional plugin. Run offline checks with
`python3 -m pytest tests/test_linkerhand_l20_lite_gesture.py -q`; these checks
do not move hardware.
