"""Conservative pair-of-legs candidate from the G1 downward depth camera.

This is a geometry heuristic, not person identification. It fails closed when
the scene contains no unique pair. The camera must see both lower legs.
"""
from array import array
import sys
import zlib


class DepthLegDetector:
    WIDTH = 640
    HEIGHT = 480
    STEP = 4

    def __init__(self):
        self._previous = None
        self._stable = 0

    def reset(self):
        self._previous = None
        self._stable = 0

    def detect(self, compressed):
        try:
            decoder = zlib.decompressobj()
            raw = decoder.decompress(bytes(compressed), self.WIDTH * self.HEIGHT * 2 + 1)
            if not decoder.eof or decoder.unconsumed_tail:
                raise ValueError("truncated_or_oversized_depth")
        except (ValueError, zlib.error):
            self.reset()
            return None, "invalid_depth"
        if len(raw) != self.WIDTH * self.HEIGHT * 2:
            self.reset()
            return None, "invalid_depth_size"
        values = array('H')
        values.frombytes(raw)
        if sys.byteorder != 'little':
            values.byteswap()
        candidate, reason = self._candidate(values)
        if candidate is None:
            self.reset()
            return None, reason
        if self._previous is not None and abs(candidate - self._previous) <= .12:
            self._stable += 1
        else:
            self._stable = 1
        self._previous = candidate
        if self._stable < 3:
            return None, "leg_candidate_unconfirmed"
        return candidate, "legs_tracked"

    def _candidate(self, values):
        width = self.WIDTH // self.STEP
        height = self.HEIGHT // self.STEP
        y0, y1 = int(height * .35), int(height * .92)
        mask = bytearray(width * height)
        depths = [0] * (width * height)
        for y in range(y0, y1):
            samples = []
            row = y * width
            source = y * self.STEP * self.WIDTH
            for x in range(width):
                d = values[source + x * self.STEP]
                if 350 <= d <= 4000:
                    samples.append(d)
                    depths[row + x] = d
            if len(samples) < width * .55:
                continue
            samples.sort()
            floor = samples[int(len(samples) * .8)]
            for x in range(int(width * .08), int(width * .92)):
                d = depths[row + x]
                if d and 180 <= floor - d <= 1800:
                    mask[row + x] = 1
        seen = bytearray(len(mask))
        components = []
        for y in range(y0, y1):
            for x in range(int(width * .08), int(width * .92)):
                start = y * width + x
                if not mask[start] or seen[start]:
                    continue
                stack = [start]
                seen[start] = 1
                xs, ys, ds = [], [], []
                while stack:
                    i = stack.pop()
                    cy, cx = divmod(i, width)
                    xs.append(cx); ys.append(cy); ds.append(depths[i])
                    for j in (i - 1, i + 1, i - width, i + width):
                        if (0 <= j < len(mask) and abs(j // width - cy) <= 1
                                and abs(j % width - cx) <= 1 and mask[j] and not seen[j]):
                            seen[j] = 1
                            stack.append(j)
                if len(xs) < 25:
                    continue
                left, right = min(xs), max(xs)
                top, bottom = min(ys), max(ys)
                w, h = right - left + 1, bottom - top + 1
                if 2 <= w <= 24 and 5 <= h <= 60 and len(xs) <= 850:
                    ds.sort()
                    components.append(((left + right) / 2, bottom, w, h, ds[len(ds) // 2]))
        pairs = []
        for i, a in enumerate(components):
            for b in components[i + 1:]:
                gap = abs(a[0] - b[0])
                center = (a[0] + b[0]) / 2
                if (8 <= gap <= 60 and .15 * width <= center <= .85 * width
                        and abs(a[1] - b[1]) <= 14
                        and abs(a[4] - b[4]) <= 350
                        and max(a[2], b[2]) <= 2.5 * min(a[2], b[2])
                        and max(a[3], b[3]) <= 2.5 * min(a[3], b[3])):
                    pairs.append(center)
        if len(pairs) != 1:
            return None, "no_unique_leg_pair" if not pairs else "ambiguous_leg_pairs"
        return 2 * pairs[0] / width - 1, "leg_pair_candidate"
