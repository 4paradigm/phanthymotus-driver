"""FR3 kinematics and sampled arm clearance in the captured SDK EE frame."""
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from .arm_collision import CAPSULES, EXCLUDED_PAIRS


_LINKS, _STARTS, _ENDS, _RADII = (np.asarray(values) for values in zip(*CAPSULES))
_PAIRS = np.asarray([(i, j) for i, j in itertools.combinations(range(len(CAPSULES)), 2)
                     if _LINKS[i] != _LINKS[j] and
                     tuple(sorted((_LINKS[i], _LINKS[j]))) not in EXCLUDED_PAIRS])
_FIRST, _SECOND = _PAIRS.T


def segment_distances(a, b, c, d):
    """Evaluate the interior closest points and four endpoint projections in batches."""
    u, v = b - a, d - c
    w = c - a
    axes = np.stack((u, -v), axis=-1)
    parameters = (np.linalg.pinv(axes, rcond=3 * np.finfo(float).eps) @ w[..., None])[..., 0]
    s, t = parameters.T
    interior = np.linalg.norm(s[:, None] * u - t[:, None] * v - w, axis=1)
    valid = np.all((parameters >= 0) & (parameters <= 1), axis=1)
    minimum = np.where(valid, interior, np.inf)
    for offset, direction in ((-w, v), (u - w, v), (w, u), (w + v, u)):
        fraction = np.clip(np.sum(offset * direction, axis=1) /
                           np.maximum(np.sum(direction ** 2, axis=1), 1e-15), 0., 1.)
        distance = np.linalg.norm(offset - fraction[:, None] * direction, axis=1)
        minimum = np.minimum(minimum, distance)
    return minimum


class Kinematics:
    def __init__(self, q, actual, *, urdf=None, limits=None):
        from .planning import rigid, rotation
        if limits is None:
            from hardware import JOINT_LOWER, JOINT_UPPER
            limits = JOINT_LOWER, JOINT_UPPER
        low, high = limits
        self.low, self.high = np.asarray(low)+.05, np.asarray(high)-.05
        root = ET.fromstring(urdf) if urdf is not None else ET.parse(
            Path(__file__).parents[1]/'resource/fr3.urdf').getroot()
        self.origins = []
        for joint in root.findall('joint'):
            o = joint.find('origin'); x, y, z = map(float, o.get('rpy').split())
            self.origins.append(rigid(rotation((0,0,1),z)@rotation((0,1,0),y)@rotation((1,0,0),x),
                                      list(map(float,o.get('xyz').split()))))
        self._last_q = self._last_chain = None
        self.offset = np.linalg.solve(self.frames(q)[-1], actual)

    def frames(self, q):
        if self._last_q is not None and np.array_equal(q, self._last_q):
            return self._last_chain
        chains, starts, ends = self.geometry(np.asarray(q)[None, :])
        self.cache_sample(q, chains[0], starts[0], ends[0])
        return self._last_chain

    def geometry(self, samples):
        chain = [np.broadcast_to(np.eye(4), (len(samples), 4, 4))]
        angles = np.column_stack((samples, np.zeros(len(samples))))
        for origin, angle in zip(self.origins, angles.T):
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.broadcast_to(np.eye(4), chain[-1].shape).copy()
            rotation[:, 0, 0] = rotation[:, 1, 1] = c
            rotation[:, 1, 0], rotation[:, 0, 1] = s, -s
            chain.append(chain[-1] @ origin @ rotation)
        chains = np.stack(chain, axis=1)
        transforms = chains[:, _LINKS]
        starts = (transforms[..., :3, :3] @ _STARTS[..., None])[..., 0] + transforms[..., :3, 3]
        ends = (transforms[..., :3, :3] @ _ENDS[..., None])[..., 0] + transforms[..., :3, 3]
        return chains, starts, ends

    def cache_sample(self, q, chain, starts, ends):
        self._last_q, self._last_chain = np.asarray(q).copy(), chain
        self._capsule_starts, self._capsule_ends = starts, ends

    def sample_clearances(self, starts, ends):
        endpoints = (values[:, pairs].reshape(-1, 3) for values, pairs in
                     ((starts, _FIRST), (ends, _FIRST), (starts, _SECOND), (ends, _SECOND)))
        distances = segment_distances(*endpoints).reshape(len(starts), len(_FIRST))
        return np.min(distances - _RADII[_FIRST] - _RADII[_SECOND], axis=1)

    def pose(self, q):
        return self.frames(q)[-1] @ self.offset

    def clearance(self, q):
        self.frames(q)
        a, b = self._capsule_starts, self._capsule_ends
        distances = segment_distances(a[_FIRST], b[_FIRST], a[_SECOND], b[_SECOND])
        return float(np.min(distances - _RADII[_FIRST] - _RADII[_SECOND]))

    def table_clearance(self, q, table, normal):
        self.frames(q)
        starts = (self._capsule_starts - table) @ normal
        ends = (self._capsule_ends - table) @ normal
        return float(np.min(np.minimum(starts, ends) - _RADII))
