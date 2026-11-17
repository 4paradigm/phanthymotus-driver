"""L20 Lite USB-RS485 transport, matching the official C++ SDK's Modbus frames.

The SDK uses function 0x04 for 10 16-bit registers and function 0x10 for
writes. Each joint value is the low byte of its register. This module keeps
the wire protocol local to the driver; it does not require a host-side SDK
installation inside the deployment container.
"""

from __future__ import annotations

import threading
import time
from copy import copy

JOINT_COUNT = 10
RIGHT_HAND_ID = 0x27
LEFT_HAND_ID = 0x28
POSITION_REGISTER = 0x0000
TORQUE_REGISTER = 0x000A
SPEED_REGISTER = 0x0014
# Fingertip pressure, as used by the vendor's RS485 host app (L10_hand_ui):
# write the finger number to holding register 0x0046, then read 98 input
# registers from 0x003C. Register 0 echoes the finger, register 1 is unused
# by the vendor app, and the remaining 96 are the raw tactile cells.
TACTILE_SELECT_REGISTER = 0x0046
TACTILE_BASE_REGISTER = 0x003C
TACTILE_REGISTER_COUNT = 98
TACTILE_CELL_COUNT = TACTILE_REGISTER_COUNT - 2
FINGER_IDS = {"thumb": 1, "index": 2, "middle": 3, "ring": 4, "pinky": 5}


def _bytes(values, *, expected: int = JOINT_COUNT) -> list[int]:
    if isinstance(values, (str, bytes)):
        raise ValueError("values must be an array of integers")
    try:
        result = list(values)
    except TypeError as exc:
        raise ValueError("values must be an array of integers") from exc
    if len(result) != expected or any(
        isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255
        for value in result
    ):
        raise ValueError(f"expected {expected} integer values in 0..255")
    return result


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC, low byte first."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc.to_bytes(2, "little")


def _frame(payload: bytes) -> bytes:
    return payload + crc16(payload)


class L20LiteModbusClient:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        side: str = "right",
        *,
        baudrate: int = 115200,
        timeout_s: float = 0.5,
        serial_factory=None,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if not 0.05 <= timeout_s <= 5:
            raise ValueError("timeout_s must be in 0.05..5 seconds")
        self.port = port
        self.side = side
        self.slave_id = RIGHT_HAND_ID if side == "right" else LEFT_HAND_ID
        self._lock = threading.RLock()
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self._serial = serial_factory(
            port=port, baudrate=baudrate, bytesize=8, parity="N",
            stopbits=1, timeout=timeout_s, write_timeout=timeout_s,
        )

    def for_side(self, side: str) -> L20LiteModbusClient:
        """Address another hand using the same serial device and transaction lock."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if side == self.side:
            return self
        client = copy(self)
        client.side = side
        client.slave_id = RIGHT_HAND_ID if side == "right" else LEFT_HAND_ID
        return client

    def close(self) -> None:
        self._serial.close()

    def _read_exact(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            part = self._serial.read(size - len(data))
            if not part:
                raise TimeoutError(f"Modbus timeout on {self.port}: received {len(data)}/{size} bytes")
            data.extend(part)
        return bytes(data)

    def _transact(self, payload: bytes, expected_function: int) -> bytes:
        with self._lock:
            # Discard a delayed reply from an earlier timed-out transaction.
            self._serial.reset_input_buffer()
            self._serial.write(_frame(payload))
            self._serial.flush()
            header = self._read_exact(3)
            if header[0] != self.slave_id:
                raise ValueError(f"unexpected Modbus slave ID 0x{header[0]:02x}")
            if header[1] == (expected_function | 0x80):
                error_frame = header + self._read_exact(2)
                self._validate_crc(error_frame)
                raise RuntimeError(f"Modbus exception code 0x{header[2]:02x}")
            if header[1] != expected_function:
                raise ValueError(f"unexpected Modbus function 0x{header[1]:02x}")
            remainder = header[2] + 2 if expected_function == 0x04 else 5
            response = header + self._read_exact(remainder)
            self._validate_crc(response)
            return response

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        if crc16(frame[:-2]) != frame[-2:]:
            raise ValueError("Modbus CRC mismatch")

    def _read_registers(self, address: int, count: int = JOINT_COUNT,
                        max_value: int = 255) -> list[int]:
        payload = bytes((self.slave_id, 0x04)) + address.to_bytes(2, "big") + count.to_bytes(2, "big")
        response = self._transact(payload, 0x04)
        if response[2] != count * 2:
            raise ValueError(f"expected {count * 2} register bytes, got {response[2]}")
        data = response[3:-2]
        registers = [int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2)]
        if any(value > max_value for value in registers):
            raise ValueError(f"register contains value outside 0..{max_value}")
        return registers

    def _write_registers(self, address: int, values, *, expected: int = JOINT_COUNT) -> list[int]:
        values = _bytes(values, expected=expected)
        data = b"".join(value.to_bytes(2, "big") for value in values)
        payload = (
            bytes((self.slave_id, 0x10)) + address.to_bytes(2, "big")
            + len(values).to_bytes(2, "big") + bytes((len(data),)) + data
        )
        response = self._transact(payload, 0x10)
        if response[2:6] != payload[2:6]:
            raise ValueError("Modbus write acknowledgement has wrong address or count")
        return values

    def get_positions(self) -> list[int]:
        return self._read_registers(POSITION_REGISTER)

    def probe_positions(self, side: str) -> list[int]:
        """Read one known hand address without changing the client's target."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        with self._lock:
            previous_id = self.slave_id
            self.slave_id = RIGHT_HAND_ID if side == "right" else LEFT_HAND_ID
            try:
                return self.get_positions()
            finally:
                self.slave_id = previous_id

    def read_fingertip(self, finger: str, settle_s: float = 0.03) -> list[int]:
        """Select one fingertip and return its 96 raw tactile cells."""
        finger_id = FINGER_IDS[finger]
        with self._lock:
            self._write_registers(TACTILE_SELECT_REGISTER, [finger_id], expected=1)
            if settle_s > 0:
                time.sleep(settle_s)
            registers = self._read_registers(TACTILE_BASE_REGISTER, TACTILE_REGISTER_COUNT, 0xFFFF)
        if registers[0] != finger_id:
            raise ValueError(f"tactile reply is for finger {registers[0]}, expected {finger_id}")
        return registers[2:]

    def get_speeds(self) -> list[int]:
        return self._read_registers(SPEED_REGISTER)

    def get_torques(self) -> list[int]:
        return self._read_registers(TORQUE_REGISTER)

    def set_positions(self, values) -> list[int]:
        return self._write_registers(POSITION_REGISTER, values)

    def set_speeds(self, values) -> list[int]:
        return self._write_registers(SPEED_REGISTER, values)

    def set_torques(self, values) -> list[int]:
        return self._write_registers(TORQUE_REGISTER, values)
