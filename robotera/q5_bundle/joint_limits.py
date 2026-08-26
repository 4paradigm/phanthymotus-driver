"""Joint position limits shared by Q5 position-control cards.

The bundled URDF is the source of truth for the ordinary body and neck joint
limits.  Parsing it at startup keeps the card schema and runtime validation in
sync without duplicating a second set of safety numbers in YAML.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


URDF_PATH = Path(__file__).parent / "resource" / "q5_model.urdf"


def load_joint_limits(path: Path = URDF_PATH) -> dict[str, tuple[float, float]]:
    """Return finite lower/upper limits for joints that declare both values."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return {}
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        try:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
        except (KeyError, TypeError, ValueError):
            continue
        if lower <= upper:
            limits[name] = (lower, upper)
    return limits


JOINT_LIMITS = load_joint_limits()


def limits_for(joint_names: tuple[str, ...] | list[str]) -> dict[str, dict[str, float]]:
    """Format limits for card metadata without exposing tuple internals."""
    return {
        name: {"min_rad": JOINT_LIMITS[name][0], "max_rad": JOINT_LIMITS[name][1]}
        for name in joint_names if name in JOINT_LIMITS
    }
