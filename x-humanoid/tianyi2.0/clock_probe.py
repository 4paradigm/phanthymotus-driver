"""Isolated read-only HTTP clock sampling; IPC never supplies capture time."""
import http.client
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time


def measure(connection, prefix):
    before, mono = time.time_ns(), time.monotonic_ns()
    connection.request('GET', prefix + '/api/platform/v1/timestamp')
    requested = time.monotonic_ns()
    response = connection.getresponse()
    headers = time.monotonic_ns()
    payload = response.read(4096)
    after = time.time_ns()
    if response.status != 200:
        raise ValueError('chassis clock request failed')
    value = json.loads(payload)
    if isinstance(value, str) and value.isdigit():
        value = int(value)
    return dict(device_ms=value, before_ns=before, after_ns=after, monotonic_ns=mono,
                request_ms=(requested-mono)/1e6, headers_ms=(headers-requested)/1e6,
                body_ms=(time.monotonic_ns()-headers)/1e6, rtt_ms=(after-before)/1e6)


class ClockProbe:
    def __init__(self, host, port, prefix):
        self._process = subprocess.Popen(
            [sys.executable, '-u', str(Path(__file__).resolve()), host, str(port), prefix],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self._sequence = 0
        self._broken = False

    def read(self):
        if self._broken or self._process.poll() is not None:
            raise ValueError('clock probe unavailable')
        self._sequence += 1
        try:
            self._process.stdin.write(str(self._sequence) + '\n')
            self._process.stdin.flush()
            # One bounded request in flight; timeout retires this channel.
            deadline = time.monotonic() + .6
            data = bytearray()
            while b'\n' not in data:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([self._process.stdout], [], [], remaining)[0]:
                    raise ValueError('clock probe IPC timeout')
                part = os.read(self._process.stdout.fileno(), 4096)
                if not part:
                    raise ValueError('clock probe closed')
                data.extend(part)
                if len(data) > 4096:
                    raise ValueError('clock probe response oversized')
            result = json.loads(data)
            if not isinstance(result, dict) or result.get('sequence') != self._sequence:
                raise ValueError('clock probe response sequence mismatch')
            if 'error' in result:
                raise ValueError(result['error'])
            return result
        except (OSError, ValueError, TypeError):
            self._broken = True
            raise

    def close(self):
        self._broken = True
        if not self._process.stdin.closed:
            try:
                self._process.stdin.close()
            except OSError:
                pass  # A failed flush must not bypass worker reaping.
        try:
            self._process.wait(timeout=.7)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=.3)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        self._process.stdout.close()


def main():
    connection = http.client.HTTPConnection(sys.argv[1], int(sys.argv[2]), timeout=.5)
    try:
        for line in sys.stdin:
            sequence = int(line)
            try:
                result = dict(measure(connection, sys.argv[3]), sequence=sequence)
            except (OSError, ValueError, TypeError, http.client.HTTPException) as error:
                result = dict(sequence=sequence, error='clock probe: ' + str(error)[:200])
            print(json.dumps(result, allow_nan=False), flush=True)
    finally:
        connection.close()


if __name__ == '__main__':
    main()
