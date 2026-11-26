"""Offline AS2-W BMS calculations. No ROS/DDS dependencies.

V1 assumptions pending hardware confirmation: bmsvoltage[0] is pack mV;
negative current is discharge. SOH is only a BMS report, never capacity.
Plausibility tolerances are configurable software heuristics, NOT safety limits.
"""
from collections import deque
from dataclasses import dataclass
import math
from numbers import Real


def number(value):
    return (float(value) if isinstance(value, Real)
            and not isinstance(value, bool)
            and math.isfinite(value) else None)


def field(msg, name):
    return msg.get(name) if isinstance(msg, dict) else getattr(msg, name, None)


@dataclass(frozen=True)
class PowerConfig:
    # Confirmed AS2-W nominal capacity; override for other batteries.
    # Explicit None disables runtime estimates. SOH is never applied.
    capacity_wh: float | None = 648.0
    window_s: float = 300.0
    min_observation_s: float = 180.0
    min_coverage: float = 0.9
    max_gap_s: float = 0.25
    stale_s: float = 1.0
    min_power_w: float = 1.0
    # Observed cell/pack difference <=93 mV.
    voltage_tolerance_mv: float = 500.0
    cell_min_mv: float = 1500.0
    cell_max_mv: float = 4500.0
    max_segments: int = 24000

    def __post_init__(self):
        for key in (
            'window_s', 'min_observation_s', 'max_gap_s', 'stale_s',
            'min_power_w', 'voltage_tolerance_mv', 'cell_min_mv',
                'cell_max_mv'):
            value = number(getattr(self, key))
            if value is None or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if (not 0 < self.min_coverage <= 1
                or self.min_observation_s > self.window_s):
            raise ValueError('invalid observation/coverage configuration')
        if (self.cell_max_mv <= self.cell_min_mv
                or self.max_gap_s > self.stale_s):
            raise ValueError('invalid cell bounds or gap/stale configuration')
        if (type(self.max_segments) is not int
                or not 1 <= self.max_segments <= 100000):
            raise ValueError('max_segments must be an integer in [1, 100000]')


def parse_bms(msg, config):
    errors = []
    voltage = None
    raw = field(msg, 'bmsvoltage')
    cells = field(msg, 'cell_vol')
    try:
        values = [number(v) for v in raw] if len(raw) == 3 else []
        cell_values = [number(v) for v in cells] if len(cells) == 40 else []
    except (TypeError, ValueError):
        values, cell_values = [], []
    layout_ok = (len(cell_values) == 40
                 and all(v is not None
                         and config.cell_min_mv <= v <= config.cell_max_mv
                         for v in cell_values[:12])
                 and all(v == 0 for v in cell_values[12:]))
    if not layout_ok:
        errors.append('cell_layout_or_voltage')
    if (len(values) != 3 or any(v is None for v in values)
            or values[0] <= 0 or values[1] <= 0 or values[2] != 0):
        errors.append('pack_voltage_missing_or_invalid')
    elif layout_ok:
        cell_sum = sum(cell_values[:12])
        if max(abs(values[0] - values[1]), abs(values[0] - cell_sum),
               abs(values[1] - cell_sum)) > config.voltage_tolerance_mv:
            errors.append('voltage_crosscheck')
        else:
            voltage = values[0] / 1000
    current = number(field(msg, 'current'))
    if current is None or not -(2**31) <= current < 2**31:
        current = None
        errors.append('current_invalid')
    else:
        current /= 1000
    soc, soh = number(field(msg, 'soc')), number(field(msg, 'soh'))
    soc = soc if soc is not None and 0 <= soc <= 100 else None
    soh = soh if soh is not None and 0 <= soh <= 100 else None
    valid = voltage is not None and current is not None
    status = ('invalid' if not valid else 'discharging' if current < 0
              else 'charging' if current > 0 else 'idle')
    return dict(voltage_v=voltage, current_a=current, soc=soc, soh_percent=soh,
                power_w=voltage * max(0, -current) if valid else None,
                status=status, errors=errors,
                voltage_raw_mv=values if values else None,
                cell_sum_mv=sum(cell_values[:12]) if layout_ok else None)


class BatteryPower:
    """Caller serializes access. Each received sequence is processed at most once."""

    def __init__(self, config=None):
        self.config = config or PowerConfig()
        self.energy_wh = 0.0
        self.skipped_intervals = 0
        self.reset()

    def reset(self):
        self.previous = None
        self.sequence = None
        self.received = None
        self.timestamp_ms = None
        self.sample = None
        self._clear_window()

    def _clear_window(self):
        self.segments = deque()
        self.area = self.covered = 0.0
        self.window_start = None
        self.previous = None

    def _trim(self, now):
        boundary = now - self.config.window_s
        while self.segments and self.segments[0][1] <= boundary:
            a, b, p, q = self.segments.popleft()
            self.area -= (p + q) * (b - a) / 2
            self.covered -= b - a
        if self.segments and self.segments[0][0] < boundary:
            a, b, p, q = self.segments.popleft()
            r = p + (q - p) * (boundary - a) / (b - a)
            self.area -= (p + r) * (boundary - a) / 2
            self.covered -= boundary - a
            self.segments.appendleft((boundary, b, r, q))

    def update(self, msg, sequence, received_mono, timestamp_ms):
        if self.sequence is not None and sequence <= self.sequence:
            return False
        sample = parse_bms(msg, self.config)
        previous_sequence = self.sequence
        previous_time = self.received
        self.sequence, self.received = sequence, received_mono
        self.timestamp_ms, self.sample = timestamp_ms, sample
        dt = (received_mono - previous_time
              if previous_time is not None else None)
        continuous = (dt is not None and 0 < dt <= self.config.max_gap_s
                      and sequence == previous_sequence + 1)
        if previous_time is not None and not continuous:
            self.skipped_intervals += 1
            self._clear_window()
        if sample['status'] != 'discharging':
            self._clear_window()
            return True
        power = sample['power_w']
        if continuous and self.previous is not None:
            a, p = self.previous
            area = (p + power) * (received_mono - a) / 2
            self.energy_wh += area / 3600
            self.segments.append((a, received_mono, p, power))
            self.area += area
            self.covered += received_mono - a
            while len(self.segments) > self.config.max_segments:
                x, y, v, w = self.segments.popleft()
                self.area -= (v + w) * (y - x) / 2
                self.covered -= y - x
        if self.window_start is None:
            self.window_start = received_mono
        self.previous = (received_mono, power)
        self._trim(received_mono)
        return True

    def snapshot(self, now):
        stale = self.received is None or not 0 <= now - \
            self.received <= self.config.stale_s
        if stale:
            self._clear_window()
        self._trim(now)
        s = self.sample or {}
        status = 'stale' if stale else s['status']
        fresh = status not in ('invalid', 'stale')
        average = self.area / self.covered if self.covered > 1e-9 else None
        span = min(self.config.window_s, now -
                   self.window_start) if self.window_start is not None else 0
        coverage = max(0, self.covered) / span if span > 0 else 0
        capacity = number(self.config.capacity_wh)
        runtime = None
        if not fresh:
            runtime_status = status
        elif status == 'charging':
            runtime_status = 'charging'
        elif capacity is None or capacity <= 0:
            runtime_status = 'capacity_unconfigured'
        elif s.get('soc') is None:
            runtime_status = 'invalid_soc'
        elif (self.covered + 1e-7 < self.config.min_observation_s
              or coverage + 1e-9 < self.config.min_coverage):
            runtime_status = 'warming_up'
        elif average is None or average < self.config.min_power_w:
            runtime_status = 'low_power'
        else:
            runtime_status = 'available'
            runtime = capacity * s['soc'] / 100 / average * 60
        return dict(voltage_v=s.get('voltage_v') if fresh else None,
                    soh_percent=s.get('soh_percent') if not stale else None,
                    power_w=s.get('power_w') if fresh else None,
                    average_power_w=average,
                    discharged_energy_wh=self.energy_wh,
                    remaining_runtime_minutes=runtime, fresh=fresh,
                    status=status, runtime_status=runtime_status,
                    timestamp_ms=self.timestamp_ms)

    def diagnostics(self, now):
        snapshot = self.snapshot(now)
        return dict(
            snapshot=snapshot, voltage_source='bmsvoltage[0]',
            assumptions_pending_confirmation=['voltage_source',
                                              'negative_current_is_discharge'],
            raw=self.sample, config=vars(self.config),
            valid_seconds=max(0, self.covered),
            segment_count=len(self.segments),
            skipped_intervals=self.skipped_intervals)
