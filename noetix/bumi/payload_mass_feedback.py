"""Consume coherent native DDS status frames, never SDK getter snapshots.

The small native helper links the already shipped DDS runtime and status type.
It only discovers status publishers and creates a reader, never a controller.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import time

from payload_mass_sampling import MassError, finite


class DdsFeedback:
    def __init__(self, sampler, cfg, *, popen=subprocess.Popen):
        self._sampler = sampler
        self._domain = cfg.get("dds_domain", 0)
        self._topic = cfg.get("dds_status_topic", "")
        self._binary = Path(__file__).resolve().parent / "noetix_sdk_bumi/build/payload_mass_dds"
        self._popen = popen
        self._process = None
        self._thread = None
        self._cancel = threading.Event()

    def start(self):
        if self._thread is not None:
            return self._thread.is_alive()
        try:
            if type(self._domain) is not int or not 0 <= self._domain <= 200:
                raise MassError("BAD_PARAMETER", "dds_domain must be 0..200")
            if not isinstance(self._topic, str) or len(self._topic) > 200:
                raise MassError("BAD_PARAMETER", "dds_status_topic is invalid")
            self._process = self._popen([str(self._binary), str(self._domain), self._topic],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1)
            self._thread = threading.Thread(target=self._run, name="bumi_mass_feedback", daemon=True)
            self._thread.start()
            return True
        except Exception as exc:
            self._sampler.record_error(exc)
            return False

    def _accept(self, event):
        if event.get("event") == "discovery":
            return
        if event.get("event") == "error":
            raise MassError(event.get("error_code", "DDS_ERROR"), event.get("message", "DDS reader failed"))
        if event.get("event") != "frame" or event.get("adapter") != "native_dds_status":
            raise MassError("BAD_DATA", "Unexpected native status event")
        # Both timestamps are needed: source time proves robot updates; native
        # receipt time prevents old frames buffered in the pipe looking fresh.
        stamp = event.get("source_timestamp")
        if type(stamp) is not int or stamp < 0:
            raise MassError("FRESHNESS_UNVERIFIED", "Missing motor-array source timestamp")
        received = finite(event.get("host_received_monotonic_s"), "native receipt time", 0)
        age = time.monotonic() - received
        if not -.05 <= age <= .25:
            raise MassError("STALE_SOURCE", "Native feedback is delayed; refusing buffered state")
        if not self._sampler.push(event):
            # Repeated timestamps are not samples and do not refresh host age.
            # Malformed data is already recorded by SharedSampler.push().
            return

    def _run(self):
        try:
            while not self._cancel.is_set():
                line = self._process.stdout.readline(65537)
                if not line:
                    raise MassError("DDS_READER_EXITED", "Native status reader exited")
                if len(line) > 65536 or not line.endswith("\n"):
                    raise MassError("BAD_DATA", "Native status frame exceeds size limit")
                self._accept(json.loads(line))
        except Exception as exc:
            if not self._cancel.is_set():
                self._sampler.record_error(exc)
        finally:
            if self._process.poll() is None:
                self._process.terminate()

    def stop(self):
        self._cancel.set()
        process = self._process
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("DDS feedback reader did not stop")
        if process is not None and process.stdout is not None:
            process.stdout.close()
