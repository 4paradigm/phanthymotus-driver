"""Prepare a right-side runtime copy with consistent gripper units."""
from pathlib import Path
import sys


def prepare(source):
    replacements = {
        '    abs_deltas = np.abs(start_pos - joints)':
            '    # Startup guard compares only the seven arm joint angles.\n'
            '    abs_deltas = np.abs(start_pos[:7] - joints[:7])',
        '            start_pos[id_mask],\n            joints[id_mask],':
            '            start_pos[:7][id_mask],\n            joints[:7][id_mask],',
        '        current_joints = obs["joint_positions"]\n        delta = command_joints - current_joints':
            '        current_joints = np.array(obs["joint_positions"], copy=True)\n'
            '        current_joints[7] = np.clip(1.0 - current_joints[7], 0.0, 1.0)\n'
            '        delta = command_joints - current_joints',
        '    if (action - joints > 0.5).any():':
            '    residual = np.abs(action[:7] - joints[:7])\n'
            '    if (residual > 0.5).any():',
        '        joint_index = np.where(action - joints > 0.8)':
            '        joint_index = np.flatnonzero(residual > 0.5)',
    }
    for old, new in replacements.items():
        if source.count(old) != 1:
            raise ValueError(f"Vendor run_env source has changed at {old!r}")
        source = source.replace(old, new)
    compile(source, "right_run_env.py", "exec")
    return source


if __name__ == "__main__":
    source, destination = map(Path, sys.argv[1:])
    destination.write_text(prepare(source.read_text()))
