"""Hold the vendor impedance target until the card explicitly enables follow."""
import json
import os
from pathlib import Path
import time


def wait_for_follow(env, agent):
    import numpy as np

    gate = Path(os.environ["FR3_FOLLOW_GATE"])
    gate.unlink(missing_ok=True)
    observation = env.get_obs()
    target = np.asarray(agent.act(observation), dtype=float).copy()
    if target.shape != (8,) or not np.isfinite(target).all():
        raise RuntimeError("Invalid master alignment target")
    started = previous_time = time.monotonic()
    previous = np.asarray(observation["joint_positions"][:7], dtype=float)
    stable = 0
    announced = False
    last_report = 0.
    while True:
        joints = np.asarray(observation["joint_positions"], dtype=float)
        if joints.shape != (8,) or not np.isfinite(joints).all():
            raise RuntimeError("Invalid follower alignment feedback")
        now = time.monotonic()
        error = float(np.max(np.abs(target[:7] - joints[:7])))
        speed = float(np.max(np.abs(joints[:7] - previous))) / max(now - previous_time, .001)
        if not announced:
            stable = stable + 1 if error <= .05 and speed <= .02 else 0
            if stable >= 10:
                announced = True
            elif now - started > 20:
                raise RuntimeError("Vendor startup alignment did not reach measured tolerance")
        elif error > .08:
            raise RuntimeError("Follower moved away from the held alignment target")
        current_master = np.asarray(agent.act(observation), dtype=float)
        if current_master.shape != (8,) or not np.isfinite(current_master).all():
            raise RuntimeError("Invalid master feedback while waiting for follow")
        master_delta = float(np.max(np.abs(current_master[:7] - joints[:7])))
        if now - last_report >= .25:
            stage = "vendor_aligned" if announced else "vendor_aligning"
            details = dict(stage=stage, target_rad=target[:7].tolist(),
                           actual_rad=joints[:7].tolist(), max_joint_error_rad=error,
                           master_delta_rad=master_delta, max_velocity_rad_s=speed)
            print("PNP_PHASE=" + json.dumps(details, allow_nan=False), flush=True)
            last_report = now
        if gate.exists():
            gate.unlink()
            if announced and master_delta <= .05:
                return
        command = joints.copy()
        delta = target[:7] - joints[:7]
        scale = min(1., .05 / max(float(np.max(np.abs(delta))), .000001))
        command[:7] += delta * scale
        command[7] = target[7]
        previous, previous_time = joints[:7].copy(), now
        observation = env.step(command)
