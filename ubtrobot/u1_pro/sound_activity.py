"""Receipt-clock activity gate for the verified U1 six-microphone PCM layout.

This suppresses background DOA results; it does not establish capture alignment.
"""
import math
from collections import deque

import numpy as np


class SoundActivityGate:
    def __init__(self, minimum_dbfs=-50.0, margin_db=10.0):
        if (isinstance(minimum_dbfs, bool) or isinstance(margin_db, bool)
                or not isinstance(minimum_dbfs, (int, float))
                or not isinstance(margin_db, (int, float))
                or not math.isfinite(minimum_dbfs) or not -90 <= minimum_dbfs <= -10
                or not math.isfinite(margin_db) or not 1 <= margin_db <= 30):
            raise ValueError('activity minimum_dbfs must be [-90,-10], margin_db [1,30]')
        self.minimum_dbfs, self.margin_db = minimum_dbfs, margin_db
        self.reset()

    def reset(self):
        self.level = self.background = None
        self.active = False
        self.reason = 'calibrating'
        self._calibration = deque(maxlen=100)
        self._elapsed = self._rising = self._quiet = 0.0
        self._active_since = None
        self._last_above = None

    def invalidate(self, reason):
        self.active = False
        self._rising = self._quiet = 0.0
        self._active_since = None
        self._last_above = None
        self.reason = reason

    def update(self, data, device, now):
        if (device['sample_format'] != 'S16LE' or device['sample_rate'] != 16000
                or device['channels'] != 8 or device['device_name'] not in
                ('Bothlent UAC Dongle (USB Audio)', 'Mic Arry 8 (USB Audio)')
                or not data or len(data) % 16):
            self.invalidate('unsupported_or_invalid_audio')
            return
        samples = np.frombuffer(bytes(data), dtype='<i2').reshape(-1, 8)[:, :6].astype(np.float64)
        # Last two raw channels are playback channels, not physical microphones.
        samples -= samples.mean(axis=0)
        rms = float(np.median(np.sqrt(np.mean(samples * samples, axis=0))))
        self.level = 20 * math.log10(max(rms / 32768, 1e-6))
        duration = min(len(samples) / 16000, .1)
        if self._elapsed < 1.0:
            self._calibration.append(self.level)
            self._elapsed += duration
            self.background = float(np.percentile(self._calibration, 20))
            self.reason = 'calibrating'
            return
        threshold = max(self.minimum_dbfs, self.background + self.margin_db)
        above = self.level >= threshold
        if above:
            self._last_above = now
            self._rising += duration
            self._quiet = 0
            if not self.active and self._rising >= .15:
                self.active = True
                self._active_since = now
        else:
            self._rising = 0
            self._quiet += duration
            if self._quiet >= .8:
                self.invalidate('below_threshold')
            # Freeze background during activity so sustained speech/music does
            # not raise its own threshold. Startup should be reasonably quiet.
            if not self.active:
                self.background += (1 - math.exp(-duration / 3)) * (self.level - self.background)
        self.reason = ('settling' if self.active and now - self._active_since < .5 else
                       'sound_detected' if self.allows(now) else 'below_threshold')

    def allows(self, now):
        return (self.active and self._active_since is not None and self._last_above is not None
                and now - self._active_since >= .5 and now - self._last_above <= .15)

    def info(self):
        return {'active': self.active, 'reason': self.reason,
                'level_dbfs': self.level, 'background_dbfs': self.background,
                'minimum_dbfs': self.minimum_dbfs, 'margin_db': self.margin_db,
                'threshold_dbfs': None if self.background is None else
                    max(self.minimum_dbfs, self.background + self.margin_db),
                'timing_basis': 'receipt_time_only', 'settle_seconds': .5, 'pause_hold_seconds': .8}
