"""Mac candleLight transport for the same pinned SDK and card implementation."""
import os
import platform
import threading
import time


_configured = False


def port(callback, index=0):
    import can
    import libusb_package
    import usb.backend.libusb1
    import usb.core
    from piper_sdk.hardware_port import C_STD_CAN
    global _configured
    if platform.system() != "Darwin":
        raise RuntimeError("gs_usb Mac transport requires macOS")
    if not _configured:
        backend = usb.backend.libusb1.get_backend
        usb.backend.libusb1.get_backend = lambda *args, **kwargs: backend(find_library=libusb_package.find_library)
        # candleLight on this Mac needs no kernel detach; libusb cannot detach
        # the reported Darwin kernel driver. This is scoped to the Mac runtime.
        usb.core.Device.is_kernel_driver_active = lambda self, interface: False
        _configured = True

    class MacPort(C_STD_CAN):
        def __init__(self):
            super().__init__(channel_name=index,bustype="gs_usb",expected_bitrate=1_000_000,
                             judge_flag=False,auto_init=False,callback_function=callback)
            self.tx_frames = []
            self.tx_lock = threading.Lock()

        def Init(self):
            if self.recv_bus is not None:
                return self.CAN_STATUS.INIT_CAN_BUS_IS_EXIST
            self.recv_bus = self.send_bus = can.Bus(interface="gs_usb",channel=index,bitrate=1_000_000)
            return self.CAN_STATUS.INIT_CAN_BUS_OPENED_SUCCESS

        def ReadCanMessage(self):
            message = self.recv_bus.recv(timeout=.1)
            if message is None:
                return self.CAN_STATUS.READ_CAN_MSG_TIMEOUT
            # gs_usb timestamps count from adapter boot; the pinned SDK expects
            # Unix seconds. Use host receive time for its freshness checks.
            message.timestamp = time.time()
            self.callback_function(message)
            return self.CAN_STATUS.READ_CAN_MSG_OK

        def SendCanMessage(self, arbitration_id, data, dlc=8, is_extended_id=False):
            # A second, transport-level read-only gate for Mac acceptance.
            if arbitration_id != 0x4AF and os.environ.get("PIPER_MOTION_ENABLED","0") != "1":
                raise RuntimeError("Mac USB-CAN 只读门禁拒绝控制帧")
            with self.tx_lock:
                result = super().SendCanMessage(arbitration_id,data,dlc,is_extended_id)
                if result != self.CAN_STATUS.SEND_MESSAGE_SUCCESS:
                    raise RuntimeError("USB-CAN 发送未成功")
                self.tx_frames.append({"id":hex(arbitration_id),"data":bytes(data).hex()})
                return result

        def Close(self):
            if self.recv_bus is not None:
                self.recv_bus.shutdown()
                self.recv_bus = self.send_bus = None
            return self.CAN_STATUS.CLOSE_CAN_BUS_CONNECT_SHUT_DOWN

    return MacPort()
