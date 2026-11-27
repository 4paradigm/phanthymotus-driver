"""Pair passive PiPER joint reference and measured position samples."""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from itertools import islice
import math
import struct


REFERENCE_IDS = (0x155, 0x156, 0x157)
FEEDBACK_IDS = (0x2A5, 0x2A6, 0x2A7)
FRAME_IDS = REFERENCE_IDS + FEEDBACK_IDS


@dataclass(frozen=True)
class Sample:
    stamp: float
    angles: tuple[float, ...]


class JointFrames:
    """One ordered three-frame sample, without carrying parts across cycles."""

    def __init__(self, ids, max_span=0.015):
        self.ids = ids
        self.max_span = max_span
        self.start = self.last = 0.0
        self.angles = ()

    def accept(self, frame_id, data, stamp):
        if len(data) != 8:
            self.angles = ()
            return None
        degrees = tuple(value / 1000 for value in struct.unpack('>ii', data))
        if frame_id == self.ids[0]:
            self.start = self.last = stamp
            self.angles = degrees
            return None
        if (not self.angles or frame_id != self.ids[len(self.angles) // 2]
                or stamp < self.last or stamp - self.start > self.max_span):
            self.angles = ()
            return None
        self.last = stamp
        self.angles += degrees
        if len(self.angles) == 6:
            sample = Sample(stamp, self.angles)
            self.angles = ()
            return sample
        return None


def interpolate(samples, times, stamp, max_gap):
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        return samples[index].angles
    if index == 0 or index == len(times):
        return None
    before, after = samples[index - 1], samples[index]
    if after.stamp - before.stamp > max_gap:
        return None
    fraction = (stamp - before.stamp) / (after.stamp - before.stamp)
    return tuple(a + fraction * (b - a) for a, b in zip(before.angles, after.angles))


def estimate_lag(reference, feedback, max_gap):
    """Return a coarse delay only for a distinctive, well-matched motion."""
    if not reference or not feedback:
        return None
    ref_times = [sample.stamp for sample in reference]
    fb_times = [sample.stamp for sample in feedback]
    end = min(ref_times[-1], fb_times[-1])
    times = [end - index * 0.04 for index in range(20)]
    actual = [interpolate(feedback, fb_times, stamp, max_gap) for stamp in times]
    if any(angles is None for angles in actual):
        return None
    spans = [max(values) - min(values) for values in zip(*actual)]
    moving_axes = [index for index, span in enumerate(spans) if span >= 1.0]
    if not moving_axes:
        return None

    scores = []
    for lag_ms in range(0, 251, 10):
        expected = [interpolate(reference, ref_times, stamp - lag_ms / 1000, max_gap)
                    for stamp in times]
        if any(angles is None for angles in expected):
            return None
        mse = sum((a[axis] - b[axis]) ** 2 / spans[axis] ** 2
                  for a, b in zip(actual, expected) for axis in moving_axes)
        scores.append((math.sqrt(mse / (len(times) * len(moving_axes))), lag_ms))
    error, lag_ms = min(scores)
    separated = [score for score, delay in scores if abs(delay - lag_ms) >= 30]
    if error > 0.15 or min(separated) - error < 0.02:
        return None
    return lag_ms


class FollowPair:
    def __init__(self, tolerance_deg, persistence_s, stale_s, max_gap_s):
        self.tolerance_deg = tolerance_deg
        self.persistence_s = persistence_s
        self.stale_s = stale_s
        self.max_gap_s = max_gap_s
        self.reset()

    def reset(self):
        self.reference_frames = JointFrames(REFERENCE_IDS)
        self.feedback_frames = JointFrames(FEEDBACK_IDS)
        self.reference = deque(maxlen=1024)
        self.feedback = deque(maxlen=1024)
        self.last_comparison = None
        self.exceeded_since = [None] * 6

    def accept(self, frame_id, data, stamp):
        if frame_id in REFERENCE_IDS:
            decoder, history = self.reference_frames, self.reference
        else:
            decoder, history = self.feedback_frames, self.feedback
        sample = decoder.accept(frame_id, data, stamp)
        if sample is None:
            return
        if history and sample.stamp <= history[-1].stamp:
            return
        history.append(sample)
        self._compare()

    def _compare(self):
        if not self.reference or not self.feedback:
            return
        reference, feedback = self.reference, self.feedback
        stamp = min(reference[-1].stamp, feedback[-1].stamp)
        previous = self.last_comparison
        if previous and stamp <= previous[0]:
            return
        # Only the boundary samples near the newest common timestamp are needed.
        ref = list(islice(reversed(reference), 16))[::-1]
        fb = list(islice(reversed(feedback), 16))[::-1]
        expected = interpolate(ref, [sample.stamp for sample in ref], stamp, self.max_gap_s)
        actual = interpolate(fb, [sample.stamp for sample in fb], stamp, self.max_gap_s)
        if expected is None or actual is None:
            self.last_comparison = None
            self.exceeded_since = [None] * 6
            return
        if previous and stamp - previous[0] > self.max_gap_s:
            self.exceeded_since = [None] * 6
        errors = tuple(b - a for a, b in zip(expected, actual))
        for axis, error in enumerate(errors):
            if abs(error) <= self.tolerance_deg:
                self.exceeded_since[axis] = None
            elif self.exceeded_since[axis] is None:
                self.exceeded_since[axis] = stamp
        self.last_comparison = stamp, expected, actual, errors

    def snapshot(self, now, *, include_lag=False):
        def age(history):
            return max(0.0, now - history[-1].stamp) if history else None

        reference_age, feedback_age = age(self.reference), age(self.feedback)
        result = {
            'reference': 'external_can_joint_target',
            'reference_age_ms': round(reference_age * 1000, 1) if reference_age is not None else None,
            'feedback_age_ms': round(feedback_age * 1000, 1) if feedback_age is not None else None,
            'max_error_deg': None,
            'worst_joint': None,
            'estimated_lag_ms': None,
            'joints': [],
        }
        if feedback_age is None or feedback_age > self.stale_s:
            return dict(result, state='feedback_unavailable')
        if reference_age is None or reference_age > self.stale_s:
            return dict(result, state='reference_unavailable')
        if (abs(reference_age - feedback_age) > self.max_gap_s or self.last_comparison is None
                or now - self.last_comparison[0] > self.stale_s):
            return dict(result, state='unaligned')
        stamp, expected, actual, errors = self.last_comparison
        worst = max(range(6), key=lambda axis: abs(errors[axis]))
        joints = []
        for axis, (target, measured, error, since) in enumerate(zip(expected, actual, errors, self.exceeded_since), 1):
            duration = stamp - since if since is not None else 0.0
            joints.append(dict(joint=axis, target_deg=round(target, 3), measured_deg=round(measured, 3),
                               error_deg=round(error, 3), above_tolerance_ms=round(duration * 1000, 1),
                               sustained=since is not None and duration >= self.persistence_s))
        state = 'following'
        if any(joint['sustained'] for joint in joints):
            state = 'sustained_deviation'
        elif any(since is not None for since in self.exceeded_since):
            state = 'transient_deviation'
        lag = estimate_lag(list(self.reference), list(self.feedback), self.max_gap_s) if include_lag else None
        return dict(result, state=state, joints=joints, worst_joint=worst + 1,
                    max_error_deg=round(abs(errors[worst]), 3), estimated_lag_ms=lag)
