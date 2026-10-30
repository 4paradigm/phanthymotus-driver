"""Opt-in process-wide SocketCAN compatibility for QEMU development hosts.

The native Linux/ARM64 path never imports or installs this module's hooks.
Callers must opt in explicitly; disabling an installed hook requires a new process.
"""

import threading
import time

# Shared by all hands and reconnect attempts within this process.
_SOCKETCAN_PATCH_LOCK = threading.Lock()


def patch_socketcan_for_qemu() -> None:
    """Compatibility patch for python-can when running under QEMU emulation on x86."""
    # Reconnect attempts must reuse a single wrapper. Re-wrapping on every
    # attempt eventually overflows all socket operations, including HTTP.
    with _SOCKETCAN_PATCH_LOCK:
        try:
            import socket
            from can.interfaces.socketcan import socketcan, constants

            orig_setsockopt = socket.socket.setsockopt
            if getattr(orig_setsockopt, "_linkerhand_l20_qemu_compat", False):
                return
            def safe_setsockopt(self, level, optname, value, *args):
                try:
                    return orig_setsockopt(self, level, optname, value, *args)
                except OSError as e:
                    if getattr(e, "errno", None) in (92, 42):  # ENOPROTOOPT
                        return
                    raise
            safe_setsockopt._linkerhand_l20_qemu_compat = True
            safe_setsockopt.__wrapped__ = orig_setsockopt
            socket.socket.setsockopt = safe_setsockopt

            def safe_capture_message(sock, get_channel, flags=0):
                try:
                    cf, ancillary_data, msg_flags, addr = sock.recvmsg(
                        constants.CANFD_MTU, socketcan.RECEIVED_ANCILLARY_BUFFER_SIZE, flags
                    )
                    if get_channel:
                        channel = addr[0] if isinstance(addr, tuple) else addr
                    else:
                        channel = None
                except OSError as error:
                    raise socketcan.can.CanOperationError(
                        f"Error receiving: {error.strerror}", error.errno
                    ) from error

                can_id, can_dlc, flags, data = socketcan.dissect_can_frame(cf)
                timestamp = time.time()
                if len(ancillary_data) == 1:
                    try:
                        cmsg_level, cmsg_type, cmsg_data = ancillary_data[0]
                        if cmsg_level == socket.SOL_SOCKET and cmsg_type == constants.SO_TIMESTAMPNS:
                            s, ns = socketcan.RECEIVED_TIMESTAMP_STRUCT.unpack_from(cmsg_data)
                            if ns < 1e9:
                                timestamp = s + ns * 1e-9
                    except Exception:
                        pass

                is_extended_frame_format = bool(can_id & constants.CAN_EFF_FLAG)
                is_remote_transmission_request = bool(can_id & constants.CAN_RTR_FLAG)
                is_error_frame = bool(can_id & constants.CAN_ERR_FLAG)
                is_fd = len(cf) == constants.CANFD_MTU
                bitrate_switch = bool(flags & constants.CANFD_BRS)
                error_state_indicator = bool(flags & constants.CANFD_ESI)
                is_rx = not bool(msg_flags & socket.MSG_DONTROUTE)

                arbitration_id = (can_id & 0x1FFFFFFF) if is_extended_frame_format else (can_id & 0x000007FF)

                return socketcan.can.Message(
                    timestamp=timestamp,
                    arbitration_id=arbitration_id,
                    is_extended_id=is_extended_frame_format,
                    is_remote_frame=is_remote_transmission_request,
                    is_error_frame=is_error_frame,
                    channel=channel,
                    dlc=can_dlc,
                    data=data[:can_dlc],
                    is_fd=is_fd,
                    is_rx=is_rx,
                    bitrate_switch=bitrate_switch,
                    error_state_indicator=error_state_indicator,
                )

            safe_capture_message._linkerhand_l20_nonblocking = True
            socketcan.capture_message = safe_capture_message
        except Exception:
            pass
