"""RTMP publish transport; keep stream credentials out of process argv.

FFmpeg writes FLV to stdout. This adapter uses the system's librtmp1 ABI
in the *same process* to publish complete FLV tags, without shell commands
or an extra child process containing the RTMP URL.
"""
from __future__ import annotations

import ctypes
import ctypes.util


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
