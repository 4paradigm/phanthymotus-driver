"""ROS-free, planar leg/IMU/visual EKF for the Bumi sensor card.

Leg kinematics and an IMU yaw *difference* provide prediction. Timestamped,
quality-gated RTAB-Map poses correct it, including delayed measurements.
The fused frame is deliberately independent of RTAB-Map's restartable segments.
This is a 2-D (x, y, yaw) estimator, not a 6-DoF VIO or a slip-proof odometer.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    x, y, z, w = (float(v) for v in q)
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError('non-finite quaternion')
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 0.8 or norm > 1.2:
        raise ValueError('invalid quaternion norm')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


@dataclass
class Step:
    stamp_ns: int
    delta_body: tuple[float, float]
    delta_yaw: float
    good_leg: bool
    state: np.ndarray
    cov: np.ndarray


class PlanarFusion:
    """Fixed-lag EKF. All updates are expressed in one continuously held frame.

    Input timestamps are mapped ROS clock nanoseconds, not callback arrival order.
    Measurement update rewinds a saved state then replays subsequent leg steps.
    """

    def __init__(self, *, max_history_s=2.5, max_speed_mps=1.3):
        self.history_ns = int(max_history_s * 1e9)
        self.max_speed_mps = max_speed_mps
        self.steps: list[Step] = []
        self.last_imu_yaw = None
        self.last_visual = None    # (stamp_ns, x, y, yaw) in current VO segment
        self.visual_segment = None
        self.visual_transform = None   # (rotation offset, 2-D translation)
        self.visual_updates = 0
        self.last_visual_stamp = None
        self.last_rejection = None
        self.slip_residual_m = 0.0
        self.slip_suspected = False
        self.scales = {'forward': 1.7, 'backward': 1.85,
                       'left': 4.4, 'right': 5.5}
        self._last_visual_anchor = None
        self._count = 0

    def _scale(self, v):
        x, y = v
        return np.array([x*self.scales['forward' if x >= 0 else 'backward'],
                         y*self.scales['left' if y >= 0 else 'right']], dtype=float)

    def _propagate(self, state, cov, step):
        yaw = wrap(state[2] + step.delta_yaw)
        d = self._scale(step.delta_body) if step.good_leg else np.zeros(2)
        heading = wrap(state[2] + step.delta_yaw/2)
        world_d = rotation(heading) @ d
        next_state = state.copy()
        next_state[:2] += world_d
        next_state[2] = yaw
        F = np.eye(3)
        F[0, 2] = -world_d[1]
        F[1, 2] = world_d[0]
        distance = float(np.linalg.norm(d))
        # Leg kinematics are model-based and unverified; slip increases noise.
        sigma_xy = (0.008 + 0.30*distance) if step.good_leg else 0.065
        if self.slip_suspected:
            sigma_xy *= 3.0
        sigma_yaw = 0.007 + 0.05*abs(step.delta_yaw)
        Q = np.diag([sigma_xy**2, sigma_xy**2, sigma_yaw**2])
        next_cov = F @ cov @ F.T + Q
        return next_state, (next_cov + next_cov.T)/2

    def predict(self, stamp_ns, imu_yaw, delta_body=(0.0, 0.0), good_leg=False):
        """Use one coherent IMU/joint read. Return False for repeated/stale time."""
        stamp_ns = int(stamp_ns)
        imu_yaw = wrap(imu_yaw)
        delta_body = tuple(float(v) for v in delta_body)
        if not all(math.isfinite(v) for v in delta_body):
            raise ValueError('non-finite leg delta')
        if not self.steps:
            self.steps.append(Step(stamp_ns, (0.0, 0.0), 0., False,
                                   np.zeros(3), np.diag([.015**2, .015**2, .03**2])))
            self.last_imu_yaw = imu_yaw
            return True
        delta_ns = stamp_ns-self.steps[-1].stamp_ns
        if delta_ns <= 0:
            return False
        dyaw = wrap(imu_yaw-self.last_imu_yaw)
        self.last_imu_yaw = imu_yaw
        # After a long SDK scheduling gap, never integrate an unbounded step.
        good_leg = bool(good_leg and delta_ns <= 120_000_000)
        step = Step(stamp_ns, delta_body, dyaw, good_leg, None, None)
        step.state, step.cov = self._propagate(self.steps[-1].state, self.steps[-1].cov, step)
        self.steps.append(step)
        cutoff = stamp_ns-self.history_ns
        while len(self.steps)>2 and self.steps[1].stamp_ns < cutoff:
            self.steps.pop(0)
        return True

    def _at(self, stamp_ns):
        if not self.steps:
            return None
        # Last state *at or before* image capture, within 100 ms maximum.
        idx = next((i for i in range(len(self.steps)-1, -1, -1)
                    if self.steps[i].stamp_ns <= stamp_ns), None)
        if idx is None or stamp_ns-self.steps[idx].stamp_ns > 100_000_000:
            return None
        return idx

    def visual(self, *, stamp_ns, segment, position, quaternion, covariance, inliers):
        """Correct a measured VO pose. Returns accepted/anchored/rejected reason.

        Distinct RTAB-Map odometry segments are aligned to the *predicted* fused
        pose at capture time and are never treated as a global frame reset.
        """
        stamp_ns = int(stamp_ns)
        if not segment or inliers < 10:
            return 'invalid_visual'
        if self.last_visual_stamp is not None and stamp_ns <= self.last_visual_stamp:
            return 'replayed_visual'
        idx = self._at(stamp_ns)
        if idx is None:
            return 'visual_outside_history'
        pos = np.asarray(position[:2], dtype=float)
        yaw = yaw_from_quat(quaternion)
        if not np.all(np.isfinite(pos)) or len(covariance) != 36:
            return 'invalid_visual'
        R = np.diag([max(0.03**2, float(covariance[0])),
                     max(0.03**2, float(covariance[7])),
                     max(0.05**2, float(covariance[35]))])
        if not np.all(np.isfinite(R)) or max(R[0, 0], R[1, 1]) > 0.2**2:
            return 'uncertain_visual'
        if segment != self.visual_segment:
            predicted = self.steps[idx].state
            offset_yaw = wrap(predicted[2]-yaw)
            offset_xy = predicted[:2]-rotation(offset_yaw)@pos
            self.visual_transform = (offset_yaw, offset_xy)
            self.visual_segment = segment
            self.last_visual = (stamp_ns, pos, yaw)
            self._last_visual_anchor = None
            self.last_visual_stamp = stamp_ns
            self.visual_updates = 0
            self.slip_suspected = False
            return 'anchored'
        if self.last_visual is not None:
            dt = (stamp_ns-self.last_visual[0])/1e9
            visual_displacement = float(np.linalg.norm(pos-self.last_visual[1]))
            visual_turn = abs(wrap(yaw-self.last_visual[2]))
            if dt <= 0 or visual_displacement > self.max_speed_mps*dt+0.12 or visual_turn > 3.0*dt+0.30:
                self.last_rejection = 'implausible visual motion'
                return 'implausible_visual'
        offset_yaw, offset_xy = self.visual_transform
        measured = np.array([*(rotation(offset_yaw)@pos+offset_xy), wrap(yaw+offset_yaw)])
        node = self.steps[idx]
        innovation = measured-node.state
        innovation[2] = wrap(innovation[2])
        # A huge yaw disagreement signals bad timing, extrinsics or VO tracking.
        if abs(innovation[2]) > 0.45:
            self.last_rejection = 'visual yaw disagreement'
            return 'visual_yaw_outlier'
        S = node.cov + R
        try:
            maha = float(innovation @ np.linalg.solve(S, innovation))
        except np.linalg.LinAlgError:
            return 'singular_covariance'
        # A plausible, high-inlier visual pose can expose leg slip. In this
        # case trust vision but flag slip; large innovations without strong VO
        # support remain rejected rather than causing a pose jump.
        if maha > 25 and inliers < 30:
            self.last_rejection = 'visual innovation gate'
            return 'innovation_outlier'
        self.slip_residual_m = float(np.linalg.norm(innovation[:2]))
        self.slip_suspected = self.slip_residual_m > 0.07
        if maha > 25:
            node.cov[:2, :2] += np.eye(2)*0.15**2
            S = node.cov + R
        K = np.linalg.solve(S.T, node.cov.T).T
        correction = K @ innovation
        # Smooth large corrections, never silently teleport a moving robot.
        travel = float(np.linalg.norm(correction[:2]))
        if travel > 0.12:
            correction[:2] *= 0.12/travel
        node.state = node.state+correction
        node.state[2] = wrap(node.state[2])
        I = np.eye(3)
        node.cov = (I-K)@node.cov@(I-K).T + K@R@K.T  # Joseph form
        # Slowly learn terrain-dependent leg scale only on well-tracked,
        # near-axis motions. Never learn from a VO jump or a mixed-direction
        # turn: otherwise an incorrectly estimated lateral scale can explode.
        if inliers >= 30 and not self.slip_suspected:
            anchor = self._last_visual_anchor
            if anchor is not None and stamp_ns-anchor[0] >= 350_000_000:
                raw_motion = np.sum([s.delta_body for s in self.steps
                                     if anchor[0] < s.stamp_ns <= stamp_ns and s.good_leg],
                                    axis=0) if any(anchor[0] < s.stamp_ns <= stamp_ns and s.good_leg
                                                   for s in self.steps) else np.zeros(2)
                visual_motion = rotation(-node.state[2]) @ (
                    rotation(offset_yaw) @ (pos-anchor[1]))
                for axis in (0, 1):
                    other = 1-axis
                    if (abs(raw_motion[axis]) >= 0.06 and
                            abs(raw_motion[other]) <= abs(raw_motion[axis])*0.20 and
                            abs(visual_motion[other]) <= abs(visual_motion[axis])*0.30):
                        ratio = visual_motion[axis]/raw_motion[axis]
                        if 0.7 <= ratio <= 8.0:
                            key = ('forward' if raw_motion[axis] >= 0 else 'backward') if axis == 0 else (
                                'left' if raw_motion[axis] >= 0 else 'right')
                            self.scales[key] = 0.92*self.scales[key]+0.08*ratio
                self._last_visual_anchor = (stamp_ns, pos.copy())
            elif anchor is None:
                self._last_visual_anchor = (stamp_ns, pos.copy())
        for i in range(idx+1, len(self.steps)):
            step = self.steps[i]
            step.state, step.cov = self._propagate(self.steps[i-1].state,
                                                   self.steps[i-1].cov, step)
        self.last_visual_stamp = stamp_ns
        self.last_visual = (stamp_ns, pos, yaw)
        self.visual_updates += 1
        self.last_rejection = None
        return 'corrected'

    def result(self, now_ns):
        if not self.steps:
            return None
        last = self.steps[-1]
        age_s = (int(now_ns)-last.stamp_ns)/1e9
        if age_s < -.05 or age_s > .25:
            return None  # A stale SDK reading cannot look like fresh odometry.
        visual_age_s = ((int(now_ns)-self.last_visual_stamp)/1e9
                        if self.last_visual_stamp is not None else math.inf)
        visual_ok = self.visual_updates > 0 and visual_age_s <= 1.0
        return dict(position=last.state[:2].tolist(), yaw=float(last.state[2]),
                    covariance=last.cov.tolist(),
                    status='tracking' if visual_ok else 'degraded',
                    visual_age_s=visual_age_s if math.isfinite(visual_age_s) else None,
                    last_visual_segment=self.visual_segment,
                    slip_suspected=self.slip_suspected,
                    slip_residual_m=self.slip_residual_m,
                    leg_scale=dict(self.scales),
                    last_rejection=self.last_rejection,
                    good_leg=last.good_leg)
