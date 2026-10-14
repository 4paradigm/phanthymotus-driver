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
hand on startup; only explicit MCP `hand` actions do. The right-hand default
`open` pose is the pose read immediately before a successful physical fist/open
test, and `close` is the official Modbus example's all-zero fist pose. Both
actions first read position, set speed/torque to the tested 60/100 values,
command a halfway pose, then command and poll the final pose. They leave the
configured torque enabled after success so the hand holds its position.
`stop` writes zero torque to all ten registers; shutdown does this if connected.
If a move times out or raises after a position command, the driver attempts to
release torque.

For local tests without hardware:

```bash
python3 -m pytest tests/test_linkerhand_l20_lite_modbus.py -q
```

The container requires access to the USB serial device. Its Compose service
already passes `/dev` into the privileged container. Do not run two hand
controllers against the same port simultaneously.
