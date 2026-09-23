"""Materialize only the live posture of a packaged, fixed-geometry profile."""
import json
import math
from pathlib import Path
import tempfile


def session_profile(path, feedback, now_ns):
    """Return (path, owned temporary directory), leaving custom profiles intact.

    The caller owns the directory until the numerical worker is closed, so a
    lazy worker restart can reopen the exact same session calibration.
    """
    source = Path(path)
    profile = json.loads(source.read_text())
    if profile.get('runtime_baseline') != 'fresh_low_state':
        return str(source), None
    stamp = feedback.get('arm_ns')
    if type(stamp) is not int or not 0 <= now_ns-stamp <= 100_000_000:
        raise ValueError('arm_feedback_stale')
    names = profile['locked_joint_names']
    measured = feedback.get('locked_joints')
    if not isinstance(measured, dict) or set(measured) != set(names):
        raise ValueError('body_feedback_missing')
    if any(type(measured[n]) not in (int, float) or not math.isfinite(measured[n]) for n in names):
        raise ValueError('body_feedback_invalid')
    profile['locked_joints'] = {n: measured[n] for n in names}
    model = Path(profile['urdf_path'])
    profile['urdf_path'] = str(model if model.is_absolute() else (source.parent/model).resolve())
    profile['baseline_sample_ns'] = stamp
    directory = tempfile.TemporaryDirectory(prefix='g1-session-profile-')
    try:
        target = Path(directory.name)/'profile.json'
        target.write_text(json.dumps(profile, sort_keys=True, separators=(',', ':'))+'\n')
        return str(target), directory
    except BaseException:
        directory.cleanup()
        raise
