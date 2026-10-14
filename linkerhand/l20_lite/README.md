# LinkerHand L20 Lite USB-RS485 driver

This is the active PhanthyMotus driver for the L20 Lite connected through a
USB-RS485 adapter. It uses Modbus RTU at 115200 baud, 8N1. The verified right
hand uses slave address `0x27` on `/dev/ttyUSB0`; left hand uses `0x28`, but
left-hand motion has not been hardware-tested here.

The transport in `modbus_client.py` implements the frames emitted by the
official LinkerHand C++ SDK: function `0x04` reads ten 16-bit registers and
`0x10` writes ten registers. Position, torque and speed start at registers
`0x0000`, `0x000A` and `0x0014` respectively. Every value is `0..255` in the
low byte of a register. The separate `can_client.py` and `device.py` files are
the previous CAN draft; `main.py` now imports `modbus_device.py` instead.

Configure `serial_port` and `side` in `config.yaml`. The driver never moves the
hand on startup; only explicit MCP `hand` actions do. Every `open`, `close`,
`set_joint` or `set_positions` request must include the boolean
`confirm_motion: true`; missing, false or string values are rejected before
opening the serial port. On the canvas, enter `true` in the confirmation field
after checking that the hand and its surroundings are clear. State reads,
speed/torque configuration and torque release do not need motion confirmation.
The right-hand default
`open` pose is the pose read immediately before a successful physical fist/open
test. The configured `close` pose is the user's requested right-hand target:
`[93,255,0,0,0,0,128,128,128,255]`. The operator confirmed successful
close/open motions on this specific right hand. The driver requires an explicit
`fist_pose` in the deployment config and refuses to start without one; this
calibration must not be assumed safe for another hand. The official L20 Lite
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

For local tests without hardware:

```bash
python3 -m pytest tests/test_linkerhand_l20_lite_modbus.py -q
```

With the powered right hand connected, run the tracked hardware smoke script
from this directory with the repository root on `PYTHONPATH`. First read state
without motion:

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
The card title is **L20 Lite 灵巧手**. It offers `get_state`, `open`, `close`,
`set_joint`, `set_positions`, `set_speed`, `set_torque` and `release`. The
`set_positions` canvas field accepts ten comma-separated numbers in joint
order; direct Python calls may still pass a ten-element list. `start` and
`stop` are canvas lifecycle actions, with `stop` also releasing torque.
