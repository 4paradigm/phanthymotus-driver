"""Read-only PAC battery cache. No robot commands are sent on this connection."""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from urllib.parse import urlsplit, urlunsplit


SOURCE = "/robot_status/ws_battery"
FIELDS = {
    "capacity": "percentage",
    "mos_temp_dc": "mos_temperature_c",
    "t1_temp_dc": "t1_temperature_c",
    "t2_temp_dc": "t2_temperature_c",
    "cycle_count": "cycle_count",
}


def parse_sample(raw):
    """Only a numeric vendor capacity makes a sample valid; never use percentage."""
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("PAC battery message must be an object")
    result = {}
    for source, target in FIELDS.items():
        value = data.get(source)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            value = None
        elif not math.isfinite(value):
            value = None
        result[target] = value
    soc = result["percentage"]
    if soc is None or not 0 <= soc <= 100:
        raise ValueError("PAC capacity must be a number in [0, 100]")
    cycles = result["cycle_count"]
    result["cycle_count"] = (
        int(cycles) if cycles is not None and cycles >= 0 and cycles == int(cycles) else None)
    protection = data.get("pstatus")
    result["protection_status"] = protection if isinstance(protection, str) else None
    return result


class BatteryStatusReceiver:
    def __init__(self, pac_url, *, stale_after_sec=10.0, reconnect_sec=2.0,
                 connector=None, monotonic=time.monotonic, wall_time=time.time):
        url = urlsplit(pac_url)
        if url.scheme not in ("http", "https", "ws", "wss") or not url.netloc:
            raise ValueError("battery PAC URL must be an HTTP or WebSocket origin")
        self._url = urlunsplit(("wss" if url.scheme in ("https", "wss") else "ws",
                               url.netloc, SOURCE, "", ""))
        self._stale_after = float(stale_after_sec)
        self._reconnect = float(reconnect_sec)
        if not math.isfinite(self._stale_after) or self._stale_after <= 0:
            raise ValueError("battery stale timeout must be positive and finite")
        if not math.isfinite(self._reconnect) or self._reconnect <= 0:
            raise ValueError("battery reconnect interval must be positive and finite")
        self._connector = connector
        self._monotonic, self._wall_time = monotonic, wall_time
        self._lock = threading.Lock()
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._socket = None
        self._connected = False
        self._sample = None
        self._received_mono = None
        self._received_at_ms = None
        self._last_error = None

    def start(self):
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="adam_battery_pac", daemon=True)
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            with self._lock:
                socket = self._socket
                self._connected = False
            errors = []
            if socket is not None:
                # Interrupt recv without sending an application-level message.
                try:
                    socket.abort()
                except OSError:
                    # The peer or worker may already have closed the transport.
                    # Still join below; a live worker remains a cleanup failure.
                    logging.getLogger(__name__).warning(
                        "PAC battery transport already unavailable during abort", exc_info=True)
                except Exception as exc:
                    logging.getLogger(__name__).exception("PAC battery abort failed")
                    errors.append(f"abort: {exc}")
            if self._thread is not None:
                try:
                    self._thread.join(5.0)  # connect timeout is bounded to 3 seconds
                    if self._thread.is_alive():
                        raise RuntimeError("PAC battery receiver did not stop")
                    self._thread = None
                except Exception as exc:
                    logging.getLogger(__name__).exception("PAC battery thread cleanup failed")
                    errors.append(f"thread: {exc}")
            if errors:
                raise RuntimeError("PAC battery cleanup failed: " + "; ".join(errors))

    def _accept(self, raw):
        sample = parse_sample(raw)
        with self._lock:
            self._sample = sample
            self._received_mono = self._monotonic()
            self._received_at_ms = int(self._wall_time() * 1000)
            self._last_error = None

    def snapshot(self):
        with self._lock:
            age = (None if self._received_mono is None else
                   max(0, int((self._monotonic() - self._received_mono) * 1000)))
            fresh = bool(self._connected and self._sample is not None and age is not None
                         and age <= self._stale_after * 1000)
            result = {key: None for key in (*FIELDS.values(), "protection_status")}
            if fresh and self._sample is not None:
                result.update(self._sample)
            result.update({
                "percentage_available": fresh,
                "percentage_message": ("State of charge provided by Adam PAC capacity" if fresh else
                    "PAC battery disconnected" if not self._connected else
                    "PAC battery waiting for a valid sample" if age is None else "PAC battery sample is stale"),
                "pac_connected": self._connected,
                "pac_fresh": fresh,
                "pac_age_ms": age,
                "pac_last_received_at_ms": self._received_at_ms,
                "pac_last_error": self._last_error,
                "pac_source": SOURCE,
            })
            return result

    def _run(self):
        import websocket
        connect = self._connector or websocket.create_connection
        while not self._stop.is_set():
            socket = None
            try:
                socket = connect(self._url, timeout=3, enable_multithread=True)
                socket.settimeout(1.0)
                connected_at = self._monotonic()
                with self._lock:
                    self._socket = socket
                    self._connected = not self._stop.is_set()
                    # A reconnect needs a new valid sample, not an old cache.
                    self._sample = None
                while not self._stop.is_set():
                    try:
                        raw = socket.recv()
                    except websocket.WebSocketTimeoutException:
                        with self._lock:
                            silent = self._received_mono
                        if self._monotonic() - max(silent or connected_at, connected_at) > self._stale_after:
                            raise TimeoutError("PAC battery receive timeout")
                        continue
                    if not raw:
                        raise ConnectionError("PAC battery connection closed")
                    try:
                        self._accept(raw)
                    except (ValueError, TypeError) as exc:
                        with self._lock:
                            self._last_error = str(exc)
            except Exception as exc:
                with self._lock:
                    if not self._stop.is_set():
                        self._last_error = str(exc)
            finally:
                with self._lock:
                    self._connected = False
                    self._socket = None
                if socket is not None:
                    try:
                        socket.close(timeout=0.2)
                    except Exception as exc:
                        logging.getLogger(__name__).exception("PAC battery socket close failed")
                        with self._lock:
                            self._last_error = f"socket close: {exc}"
                        try:
                            socket.shutdown()
                        except Exception as shutdown_exc:
                            logging.getLogger(__name__).exception("PAC battery socket shutdown failed")
                            with self._lock:
                                self._last_error += f"; socket shutdown: {shutdown_exc}"
            self._stop.wait(self._reconnect)
