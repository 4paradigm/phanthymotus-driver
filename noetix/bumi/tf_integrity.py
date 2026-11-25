"""Bounded diagnostic view of ROS TF declarations."""
import math
import re

FRAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_/]*")


def valid_frame(name):
    return bool(FRAME.fullmatch(name)) and "//" not in name and not name.endswith("/")


def angle(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.acos(min(1.0, dot))


class Integrity:
    def __init__(self, cfg):
        self.required = set(cfg.get("required_frames", ["base_link"]))
        self.chains = [tuple(chain) for chain in cfg.get("required_chains", [])]
        for parent, child in self.chains:
            self.required.update((parent, child))
        if not all(valid_frame(frame) for frame in self.required):
            raise ValueError("Invalid required frame")
        for key, default in (
            ("stale_s", 2), ("future_s", .25), ("skew_s", .5),
            ("translation_rate", 8), ("rotation_rate", 25),
            ("translation_floor", .02), ("rotation_floor", .1),
            ("publish_hz", 2),
        ):
            value = float(cfg.get(key, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
            setattr(self, key, value)
        self.visual = bool(cfg.get("visual_check", False))
        self.sim = bool(cfg.get("use_sim_time", False))
        self.edges = {}
        self.issues = {}
        self.motion = {}
        self.last_time = None
        self.clock_resets = 0
        self.capacity_drops = 0
        self.translation_jumps = self.rotation_jumps = 0

    def clock(self, now):
        if self.last_time is not None and now < self.last_time:
            self.edges = {key: sample for key, sample in self.edges.items() if key[2]}
            self.motion.clear()
            self.issues = {key: expiry for key, expiry in self.issues.items()
                           if expiry == math.inf}
            self.clock_resets += 1
        self.last_time = now

    def issue(self, kind, child, received):
        if len(self.issues) < 2048 or (kind, child) in self.issues:
            self.issues[kind, child] = received + self.stale_s

    def observe(self, parent, child, stamp, xyz, quat, static, source, now, received):
        self.clock(now)
        if not valid_frame(parent) or not valid_frame(child):
            self.issue("invalid_frame", child[:128], received)
            return
        if not all(math.isfinite(v) and abs(v) < 1e12 for v in (*xyz, *quat)):
            self.issue("invalid_transform", child, received)
            return
        norm = math.hypot(*quat)
        if abs(norm - 1) > .01:
            self.issue("invalid_quaternion", child, received)
            return
        quat = tuple(v / norm for v in quat)
        if not static and (stamp <= 0 or stamp > now + self.future_s * 1e9):
            self.issue("invalid_stamp" if stamp <= 0 else "future_stamp", child, received)
            return
        key = parent, child, static, source
        old = self.edges.get(key)
        if old is None and len(self.edges) >= 2048:
            self.capacity_drops += 1
            self.issue("capacity", child, received)
            return
        if old:
            previous_stamp, previous_xyz, previous_quat, _ = old
            if not static and stamp < previous_stamp:
                self.issue("stamp_regression", child, received)
                return
            distance = math.dist(xyz, previous_xyz)
            rotation = angle(quat, previous_quat)
            if static or stamp == previous_stamp:
                if distance > 1e-5 or rotation > 1e-4:
                    self.issue("contradictory_sample", child, received)
                    if static and ("contradictory_sample", child) in self.issues:
                        self.issues["contradictory_sample", child] = math.inf
                return
            dt = (stamp - previous_stamp) / 1e9
            if dt <= self.stale_s:
                linear, angular = distance / dt, rotation / dt
                tj = distance > self.translation_floor and linear > self.translation_rate
                rj = rotation > self.rotation_floor and angular > self.rotation_rate
                self.translation_jumps += tj
                self.rotation_jumps += rj
                normal = self.motion.get(key, {}).get("consecutive_normal", 0)
                self.motion[key] = dict(translation_m=distance, rotation_rad=rotation,
                                        translation_m_s=linear, rotation_rad_s=angular,
                                        consecutive_normal=0 if tj or rj else normal + 1)
                if tj or rj:
                    self.issue("motion_jump", child, received)
            else:
                self.motion.pop(key, None)
        self.edges[key] = stamp, xyz, quat, received
        return True

    def snapshot(self, now, received):
        self.clock(now)
        self.issues = {key: expiry for key, expiry in self.issues.items() if expiry >= received}
        frames, stale, conflicts, cycles = set(), set(), set(), set()
        parents, active = {}, {}
        for key, sample in self.edges.items():
            parent, child, static, _ = key
            stamp, _, _, arrival = sample
            frames.update((parent, child))
            parents.setdefault(child, set()).add(parent)
            if not static and max((now - stamp) / 1e9, 0 if self.sim else received - arrival) > self.stale_s:
                stale.add(child)
            else:
                active.setdefault(child, []).append((key, sample))
        stale.difference_update(active)
        for child, links in active.items():
            parents[child] = {key[0] for key, _ in links}
        conflicts.update(child for child, links in active.items()
                         if len({(key[0], key[2]) for key, _ in links}) > 1)
        for child, links in active.items():
            comparisons = {}
            for (parent, _, static, _), (stamp, xyz, quat, _) in links:
                previous = comparisons.setdefault((parent, static, 0 if static else stamp), (xyz, quat))
                if math.dist(xyz, previous[0]) > 1e-5 or angle(quat, previous[1]) > 1e-4:
                    conflicts.add(child)
        # Topological elimination on child-to-parent edges detects cyclic paths.
        incoming = dict.fromkeys(frames, 0)
        for links in parents.values():
            for parent in links:
                incoming[parent] += 1
        pending = [frame for frame in incoming if incoming[frame] == 0]
        while pending:
            for parent in parents.get(pending.pop(), ()):
                incoming[parent] -= 1
                if incoming[parent] == 0:
                    pending.append(parent)
        remaining = {frame for frame, count in incoming.items() if count}
        for start in remaining:
            pending = list(parents.get(start, set()) & remaining)
            visited = set()
            while pending:
                frame = pending.pop()
                if frame == start:
                    cycles.add(start)
                    break
                if frame not in visited:
                    visited.add(frame)
                    pending.extend(parents.get(frame, set()) & remaining)
        missing = self.required - frames
        disconnected, skewed = [], []
        for parent, child in self.chains:
            pending = [child]
            visited = set()
            while pending:
                frame = pending.pop()
                if frame == parent:
                    break
                if frame not in visited:
                    visited.add(frame)
                    pending.extend(parents.get(frame, ()))
            else:
                disconnected.append([parent, child])
                continue
            # Compare timestamps only on an unambiguous, active chain.
            frame, stamps = child, []
            while frame != parent:
                links = active.get(frame, [])
                if len(links) != 1 or frame in cycles:
                    break
                key, sample = links[0]
                frame = key[0]
                if not key[2]:
                    stamps.append(sample[0])
            else:
                if stamps and max(stamps) - min(stamps) > self.skew_s * 1e9:
                    skewed.append([parent, child])
        issues = {}
        for kind, child in self.issues:
            issues.setdefault(kind, []).append(child)
        conflicts.update(issues.get("contradictory_sample", ()))
        reasons = sorted(issues)
        if conflicts or cycles:
            state = "inconsistent"
            reasons.append("conflicting_or_cyclic_topology")
        elif missing or disconnected or skewed or issues:
            state = "degraded"
            reasons.extend(label for present, label in (
                (missing, "missing_frames"), (disconnected, "disconnected_frames"),
                (skewed, "chain_timestamp_skew")) if present)
        elif stale:
            state = "stale"
            reasons.append("dynamic_tf_stale")
        elif not frames or self.visual:
            state = "insufficient_evidence"
            reasons.append("visual_evidence_unavailable" if self.visual else "no_tf")
        else:
            state = "healthy"
        return dict(state=state, timestamp={"sec": now // 10**9, "nanosec": now % 10**9},
                    checked_frames=sorted(self.required | frames), missing_frames=sorted(missing),
                    disconnected_frames=disconnected, stale_frames=sorted(stale),
                    conflicting_frames=sorted(conflicts), cyclic_frames=sorted(cycles),
                    skewed_chains=skewed, translation_jump_count=self.translation_jumps,
                    rotation_jump_count=self.rotation_jumps, visual_residual=None,
                    visual_state="insufficient_evidence" if self.visual else "disabled",
                    reason=reasons, issues=issues, clock_resets=self.clock_resets,
                    capacity_drops=self.capacity_drops,
                    motion=[dict(parent=k[0], child=k[1], source=k[3], **v)
                            for k, v in self.motion.items()])
