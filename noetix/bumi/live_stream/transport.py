"""RTMP publish transport; keep stream credentials out of process argv.

FFmpeg writes FLV to stdout. This adapter uses the system's librtmp1 ABI
in the *same process* to publish complete FLV tags, without shell commands
or an extra child process containing the RTMP URL.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import multiprocessing
import threading
import time


class RTMPTransportError(RuntimeError):
    """Stable public error; never includes URLs or sensitive library messages."""


class FLVFramer:
    """Yield complete FLV records for librtmp's RTMP_Write (not arbitrary reads)."""

    def __init__(self):
        self._buffer = bytearray()
        self._first = True

    def feed(self, chunk: bytes) -> list[bytes]:
        self._buffer.extend(chunk)
        records = []
        while True:
            if self._first:
                if len(self._buffer) < 13:
                    break
                if self._buffer[:3] != b"FLV" or self._buffer[3] != 1:
                    raise RTMPTransportError("INVALID_FLV_HEADER")
                head_size = int.from_bytes(self._buffer[5:9], "big")
                if not 9 <= head_size <= 4096:
                    raise RTMPTransportError("INVALID_FLV_HEADER")
                if len(self._buffer) < head_size + 4 + 11:
                    break
                offset = head_size + 4
            else:
                offset = 0
                if len(self._buffer) < 11:
                    break
            size = int.from_bytes(self._buffer[offset + 1:offset + 4], "big")
            if size > 8 * 1024 * 1024:
                raise RTMPTransportError("OVERSIZED_FLV_TAG")
            record_len = offset + 11 + size + 4
            if len(self._buffer) < record_len:
                break
            records.append(bytes(self._buffer[:record_len]))
            del self._buffer[:record_len]
            self._first = False
        if len(self._buffer) > 9 * 1024 * 1024:
            raise RTMPTransportError("FLV_BUFFER_LIMIT")
        return records


def _load_librtmp():
    libname = ctypes.util.find_library("rtmp")
    if not libname:
        raise RTMPTransportError("LIBRTMP_UNAVAILABLE")
    try:
        lib = ctypes.CDLL(libname)
        lib.RTMP_Alloc.argtypes = []
        lib.RTMP_Alloc.restype = ctypes.c_void_p
        lib.RTMP_Init.argtypes = [ctypes.c_void_p]
        lib.RTMP_SetupURL.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lib.RTMP_SetupURL.restype = ctypes.c_int
        lib.RTMP_EnableWrite.argtypes = [ctypes.c_void_p]
        lib.RTMP_Connect.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.RTMP_Connect.restype = ctypes.c_int
        lib.RTMP_ConnectStream.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.RTMP_ConnectStream.restype = ctypes.c_int
        lib.RTMP_Write.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        lib.RTMP_Write.restype = ctypes.c_int
        lib.RTMP_Close.argtypes = [ctypes.c_void_p]
        lib.RTMP_Free.argtypes = [ctypes.c_void_p]
        if hasattr(lib, "RTMP_LogSetLevel"):
            lib.RTMP_LogSetLevel.argtypes = [ctypes.c_int]
            lib.RTMP_LogSetLevel(0)  # Disable URL-bearing internal diagnostics.
    except (OSError, AttributeError) as exc:
        raise RTMPTransportError("LIBRTMP_UNAVAILABLE") from None
    return lib


class RTMPPublisher:
    """One RTMP connection; no credential in subprocess argv or exception strings."""

    def __init__(self, url: str, library=None):
        self._lib = library if library is not None else _load_librtmp()
        self._handle = None
        self._url_buffer = ctypes.create_string_buffer(url.encode("utf-8"))

    def connect(self) -> None:
        lib = self._lib
        handle = lib.RTMP_Alloc()
        if not handle:
            raise RTMPTransportError("RTMP_ALLOC_FAILED")
        self._handle = handle
        try:
            lib.RTMP_Init(handle)
            if not lib.RTMP_SetupURL(handle, self._url_buffer):
                raise RTMPTransportError("RTMP_SETUP_FAILED")
            lib.RTMP_EnableWrite(handle)
            if not lib.RTMP_Connect(handle, None):
                raise RTMPTransportError("RTMP_CONNECT_FAILED")
            if not lib.RTMP_ConnectStream(handle, 0):
                raise RTMPTransportError("RTMP_PUBLISH_REJECTED")
        except Exception:
            self.close()
            raise

    def send(self, record: bytes) -> None:
        if not self._handle:
            raise RTMPTransportError("RTMP_NOT_CONNECTED")
        # RTMP_Write expects complete FLV tags. Keep Python-owned memory alive.
        packet = ctypes.create_string_buffer(record)
        written = self._lib.RTMP_Write(self._handle, packet, len(record))
        if written != len(record):
            raise RTMPTransportError("RTMP_WRITE_FAILED")

    def close(self) -> None:
        handle, self._handle = self._handle, None
        if handle:
            try:
                self._lib.RTMP_Close(handle)
            finally:
                self._lib.RTMP_Free(handle)


def _isolated_publish_loop(conn) -> None:
    """Keep blocking native librtmp calls in a killable child process."""
    # Spawn starts a fresh interpreter, so protect stdout/stderr before any I/O.
    from common import logsafe
    logsafe.install(check_fd=False)
    publisher = None
    try:
        operation, value = conn.recv()
        if operation != "connect":
            return
        publisher = RTMPPublisher(value)
        publisher.connect()
        conn.send(("ready", None))
        while True:
            operation, value = conn.recv()
            if operation == "close":
                break
            if operation != "send":
                raise RTMPTransportError("RTMP_PROTOCOL_ERROR")
            publisher.send(value)
            conn.send(("sent", None))
    except EOFError:
        pass
    except RTMPTransportError as error:
        try:
            conn.send(("error", str(error)))
        except (OSError, EOFError, BrokenPipeError):
            pass
    except (OSError, RuntimeError, ValueError):
        try:
            conn.send(("error", "RTMP_WRITE_FAILED"))
        except (OSError, EOFError, BrokenPipeError):
            pass
    finally:
        if publisher is not None:
            try:
                publisher.close()
            except OSError:
                pass
        conn.close()


class IsolatedRTMPPublisher:
    """Bound native connect/write in a subprocess without putting URLs in argv."""

    CONNECT_TIMEOUT = 12.0
    SEND_TIMEOUT = 4.0

    def __init__(self, url: str):
        self._url = url
        self._ctx = multiprocessing.get_context("spawn")
        self._conn = None
        self._process = None
        self._aborted = threading.Event()

    def _exchange(self, operation: str, value: object, timeout: float) -> None:
        if self._conn is None or self._aborted.is_set():
            raise RTMPTransportError("RTMP_CANCELLED")
        try:
            self._conn.send((operation, value))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self._aborted.is_set():
                    raise RTMPTransportError("RTMP_CANCELLED")
                if self._conn.poll(0.1):
                    status, detail = self._conn.recv()
                    if status == "error":
                        raise RTMPTransportError(str(detail))
                    if status not in ("ready", "sent"):
                        raise RTMPTransportError("RTMP_PROTOCOL_ERROR")
                    return
                if self._process is None or not self._process.is_alive():
                    raise RTMPTransportError("RTMP_PROCESS_EXITED")
        except (OSError, EOFError, BrokenPipeError, ValueError):
            raise RTMPTransportError("RTMP_PROCESS_EXITED") from None
        self.abort()
        raise RTMPTransportError(
            "RTMP_CONNECT_TIMEOUT" if operation == "connect" else "RTMP_WRITE_TIMEOUT"
        )

    def connect(self) -> None:
        parent, child = self._ctx.Pipe(duplex=True)
        self._conn = parent
        self._process = self._ctx.Process(
            target=_isolated_publish_loop, args=(child,), daemon=True
        )
        try:
            self._process.start()
        finally:
            child.close()
        url, self._url = self._url, None
        self._exchange("connect", url, self.CONNECT_TIMEOUT)

    def send(self, record: bytes) -> None:
        self._exchange("send", record, self.SEND_TIMEOUT)

    def abort(self) -> None:
        """Interrupt an in-flight native call without waiting for its return."""
        self._aborted.set()
        proc = self._process
        if proc is not None:
            try:
                if proc.is_alive():
                    proc.terminate()
                    proc.join(timeout=0.8)
                    if proc.is_alive():
                        proc.kill()
                        proc.join(timeout=0.5)
            except (OSError, ValueError, AssertionError):
                pass

    def close(self) -> None:
        self.abort()
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass
        self._url = None
