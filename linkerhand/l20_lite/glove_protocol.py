"""Hex 3.4.6 six-quaternion glove frames; no vendor SDK dependencies."""

import math

CHANNELS = {
    0x20: ("left", "root"),
    0x1A: ("left", "top"),
    0x10: ("right", "root"),
    0x2A: ("right", "top"),
}
# SDK's Decoding maps wire 0x1A to internal 0x2A and wire 0x2A to 0x1A.
# Existing driver IDs are SDK cache keys, not unmodified wire addresses.
WIRE_TO_CHANNEL = {0x20: 0x20, 0x10: 0x10, 0x1A: 0x2A, 0x2A: 0x1A}


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.accepted = 0
        self.rejected = 0

    def feed(self, data):
        self.buffer.extend(data)
        samples = []
        while self.buffer:
            if self.buffer[0] != 0xA0:
                del self.buffer[0]
                continue
            if len(self.buffer) < 5:
                break
            candidate = WIRE_TO_CHANNEL.get(self.buffer[1] & 0x3F)
            if (
                candidate in CHANNELS
                and self.buffer[3] & 0x3F == 1
                and self.buffer[4] != 24
            ):
                self.rejected += 1
                del self.buffer[0]
                continue
            size = 7 + self.buffer[4]
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            check = 0
            for byte in packet[:-2]:
                check ^= byte
            if check != packet[-2] or packet[-1] != 0x0A:
                self.rejected += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            wire_channel = packet[1] & 0x3F
            channel = WIRE_TO_CHANNEL.get(wire_channel)
            # Initial scope: six-quaternion frames on the four glove channels.
            # Ignore battery, status and other sensor records.
            if channel not in CHANNELS or packet[3] & 0x3F != 1:
                continue
            payload = packet[5:-2]
            if len(payload) != 24:
                self.rejected += 1
                continue
            quats = []
            for offset in range(0, 24, 4):
                w, x, y, z = [
                    (v if v < 128 else v - 256) / 100.0
                    for v in payload[offset : offset + 4]
                ]
                quat = [x, y, z, w]
                norm2 = sum(v * v for v in quat)
                quats.append(None if norm2 < 0.95 else quat)
            side, group = CHANNELS[channel]
            self.accepted += 1
            samples.append(
                {
                    "side": side,
                    "channel": group,
                    "channel_id": channel,
                    "wire_channel_id": wire_channel,
                    "sequence": packet[2],
                    "quaternions": quats,
                }
            )
        return samples


def normalized(quats):
    result = []
    for quat in quats:
        if quat is None:
            result.append(None)
            continue
        norm = math.sqrt(sum(v * v for v in quat))
        result.append([v / norm for v in quat] if 0.5 <= norm <= 1.5 else None)
    return result
