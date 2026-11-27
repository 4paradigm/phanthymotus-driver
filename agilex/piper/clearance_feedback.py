"""Receive-only SocketCAN feedback for a separately selected measurement arm."""
import socket
import struct
import threading
import time


class PassiveFeedback:
    IDS = tuple(range(0x2A2, 0x2A9))

    def __init__(self, interface, *, socket_factory=socket.socket, clock=time.time,
                 monotonic=time.monotonic):
        if interface not in {'can0', 'can1'}:
            raise ValueError('测距来源只允许 can0 或 can1')
        self.interface, self.factory = interface, socket_factory
        self.clock, self.monotonic = clock, monotonic
        self.lock = threading.Lock()
        self.parts, self.error = {}, None
        self.cancel = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is not None and self.thread.is_alive(): return
        self.parts.clear(); self.error = None; self.cancel.clear()
        self.thread = threading.Thread(target=self._run, name='clearance-'+self.interface, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            with self.factory(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                    b''.join(struct.pack('=II', ident, 0xC00007FF) for ident in self.IDS))
                bus.settimeout(.1)
                bus.bind((self.interface,))
                while not self.cancel.is_set():
                    try: self.accept_packet(bus.recv(16))
                    except socket.timeout: continue
        except Exception:
            with self.lock:
                self.error = 'can_receive_failed'; self.parts.clear()

    def accept_packet(self, packet):
        if len(packet) != 16: return
        ident, size, data = struct.unpack('=IB3x8s', packet)
        if ident not in self.IDS or size != 8: return
        with self.lock:
            self.parts[ident] = (data, self.clock(), self.monotonic())

    def snapshot(self):
        with self.lock: parts, error = dict(self.parts), self.error
        result = dict(connected=error is None, fresh=False, can_interface=self.interface)
        if error: return result
        required = [parts.get(i) for i in (0x2A5, 0x2A6, 0x2A7, 0x2A8)]
        if not all(required): return result
        ages = [self.monotonic()-p[2] for p in required]
        if min(ages) < 0 or max(ages) > .5 or max(ages)-min(ages) > .1: return result
        # Pinned SDK 0x2A8: bytes 0..3 stroke, 4..5 effort, 6 status,
        # byte 7 RESERVED. It is not a documented gripper mode field.
        q = [v*.001 for p in required[:3] for v in struct.unpack('>ii', p[0])]
        result.update(fresh=True, joints_deg=q, feedback_time=min(p[1] for p in required[:3]),
            gripper=dict(width_mm=struct.unpack('>i', required[-1][0][:4])[0]*.001,
                         effort_nm=struct.unpack('>h',required[-1][0][4:6])[0]*.001,
                         status_code=required[-1][0][6],
                         feedback_time=required[-1][1]))
        pose = [parts.get(i) for i in (0x2A2,0x2A3,0x2A4)]
        if all(pose) and all(0 <= self.monotonic()-p[2] <= .5 for p in pose):
            values = [v*.001 for p in pose for v in struct.unpack('>ii',p[0])]
            result['end_pose'] = dict(position_mm=values[:3],orientation_deg=values[3:],
                                     feedback_time=min(p[1] for p in pose))
        return result

    def close(self):
        self.cancel.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
            if self.thread.is_alive(): raise RuntimeError('CAN 只读接收线程尚未退出')
        self.thread = None
        with self.lock: self.parts.clear()
