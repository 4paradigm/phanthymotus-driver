"""Locate the manufacturer's installed runtime without a login shell."""
import os
from pathlib import Path


def select_existing(candidates, valid, label):
    matches = []
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if valid(candidate) and candidate not in matches:
            matches.append(candidate)
    if len(matches) != 1:
        raise RuntimeError(f"Expected one installed {label}, found {len(matches)}: {matches}")
    return matches[0]


def discover_paths(home=None):
    home = Path.home() if home is None else Path(home)
    vendors = [home / "frankateleop", Path("/opt/frankateleop")]
    vendors.extend(path.parents[2] for path in home.glob("*/teleop/experiments/run_env.py"))
    def valid_vendor(path):
        return ((path / "teleop/experiments/run_env.py").is_file()
                and (path / "right_franka/1_launch_robot.sh").is_file()
                and (path / "polymetis/polymetis/build/franka_panda_client").is_file())

    bases = [home / name for name in ("miniconda3", "anaconda3", "miniforge3", "mambaforge")]
    bases.extend((Path("/opt/conda"), Path("/opt/miniconda3")))
    environments = home / ".conda/environments.txt"
    if environments.is_file():
        for line in environments.read_text().splitlines():
            path = Path(line.strip())
            if path.name == "polymetis":
                bases.append(path.parent.parent)
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix and Path(prefix).name == "polymetis":
        bases.append(Path(prefix).parent.parent)
    def valid_conda(path):
        return os.access(path / "envs/polymetis/bin/python", os.X_OK) and (path / "bin/conda").is_file()

    return (select_existing(vendors, valid_vendor, "Franka vendor tree"),
            select_existing(bases, valid_conda, "Polymetis Conda base"))
