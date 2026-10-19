"""Mac transport and SDK lifecycle regressions, without USB or cameras."""
from enum import IntEnum
from pathlib import Path
import runpy
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agilex.piper.arm import PiperDriver
from agilex.piper import mac_can


def test_mac_entrypoint_installs_logsafe_before_server_or_output():
    events = []

    class Plugin:
        def __init__(self, config):
            self.data_dir = Path(config['data_dir'])
            self.motion_enabled = False

        def start(self):
            events.append('plugin_start')

        def stop(self):
            events.append('plugin_stop')

    class Server:
        def __init__(self, address, handler):
            events.append('server_created')

        def serve_forever(self):
            events.append('server_serving')
            raise KeyboardInterrupt

        def server_close(self):
            events.append('server_closed')

    with patch('common.logsafe.install', side_effect=lambda: events.append('logsafe')), \
         patch('agilex.piper.device.PiperPlugin', Plugin), \
         patch('http.server.ThreadingHTTPServer', Server), \
         patch('sys.argv', ['mac_main', '--data-dir', '/tmp/piper-test']), \
         patch('builtins.print', side_effect=lambda *a, **k: events.append('output')):
        runpy.run_module('agilex.piper.mac_main', run_name='__main__')

    assert events[0] == 'logsafe'
    assert 'output' in events and 'server_serving' in events
    assert events[-2:] == ['server_closed', 'plugin_stop']


class Status(IntEnum):
    INIT_CAN_BUS_IS_EXIST = 1
    INIT_CAN_BUS_OPENED_SUCCESS = 2
    READ_CAN_MSG_TIMEOUT = 3
    READ_CAN_MSG_OK = 4
    SEND_MESSAGE_SUCCESS = 5
    CLOSE_CAN_BUS_CONNECT_SHUT_DOWN = 6


class PortBase:
    CAN_STATUS = Status

    def __init__(self, **kwargs):
        self.recv_bus = self.send_bus = None
        self.callback_function = kwargs["callback_function"]

    def SendCanMessage(self, *args):
        return Status.SEND_MESSAGE_SUCCESS


class MacTests(unittest.TestCase):
    def transport(self, callback):
        modules = {"can": SimpleNamespace(), "libusb_package": SimpleNamespace(),
                   "usb": SimpleNamespace(), "usb.backend": SimpleNamespace(),
                   "usb.backend.libusb1": SimpleNamespace(), "usb.core": SimpleNamespace(),
                   "piper_sdk.hardware_port": SimpleNamespace(C_STD_CAN=PortBase)}
        with patch.dict(sys.modules, modules), patch.object(mac_can, "_configured", True), \
                patch.object(mac_can.platform, "system", return_value="Darwin"):
            return mac_can.port(callback)

    def test_adapter_boot_timestamp_becomes_host_receive_timestamp(self):
        seen = []
        adapter = self.transport(seen.append)
        frame = SimpleNamespace(timestamp=1.25)
        adapter.recv_bus = SimpleNamespace(recv=lambda timeout: frame)
        with patch.object(mac_can.time, "time", return_value=1700000000.):
            self.assertEqual(adapter.ReadCanMessage(), Status.READ_CAN_MSG_OK)
        self.assertEqual(seen[0].timestamp, 1700000000.)

    def test_readonly_transport_rejects_control_but_allows_information_query(self):
        adapter = self.transport(lambda frame: None)
        with patch.dict(mac_can.os.environ, {"PIPER_MOTION_ENABLED": "0"}):
            with self.assertRaises(RuntimeError):
                adapter.SendCanMessage(0x151, bytes(8))
            self.assertEqual(adapter.tx_frames, [])
            adapter.SendCanMessage(0x4AF, bytes(8))
        self.assertEqual(adapter.tx_frames, [{"id": "0x4af", "data": "00" * 8}])

    def test_close_joins_sdk_reader_before_closing_its_usb_handle(self):
        driver = PiperDriver()
        stop = threading.Event()
        reader = threading.Thread(target=stop.wait)
        reader.start()
        closed = []
        def disconnect(**kwargs):
            self.assertFalse(reader.is_alive())
            closed.append(True)
        driver.arm = SimpleNamespace(_read_can_stop_event=stop, _can_deal_th=reader,
                                     DisconnectPort=disconnect)
        driver.close()
        self.assertEqual(closed, [True])
        self.assertIsNone(driver.arm)

    def test_individual_stale_motor_frame_is_not_hidden_by_aggregate_timestamp(self):
        now = time.time()
        driver = PiperDriver()
        flags = ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                 "collision_status", "driver_error_status", "stall_status")
        motors = SimpleNamespace(time_stamp=now, **{f"motor_{i}": SimpleNamespace(
            can_id=0x260+i, foc_status=SimpleNamespace(**dict.fromkeys(flags, False))) for i in range(1, 7)})
        arm = SimpleNamespace(frame_lock=threading.Lock(),
            frame_times=dict.fromkeys((0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267)), now),
            GetArmJointMsgs=lambda: SimpleNamespace(time_stamp=now,
                joint_state=SimpleNamespace(**{f"joint_{i}": 0 for i in range(1, 7)})),
            GetArmStatus=lambda: SimpleNamespace(time_stamp=now, arm_status=SimpleNamespace(
                _ctrl_mode="CAN_CTRL(0x1)", _arm_status=0, _teach_status="DISABLED(0x0)", _err_code=0)),
            GetArmLowSpdInfoMsgs=lambda: motors, GetArmEnableStatus=lambda: (True,) * 6)
        driver.arm = arm
        arm.frame_times[0x263] = now-1
        with self.assertRaisesRegex(RuntimeError, "某个关节或电机"):
            driver.get_state()
        arm.frame_times[0x263] = now
        motors.motor_3.foc_status.driver_error_status = True
        self.assertEqual(driver.get_state().motor_faults, ("J3:driver_error_status",))


if __name__ == "__main__":
    unittest.main()
