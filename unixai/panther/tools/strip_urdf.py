"""Reduce the vendor's /robot_description to the kinematic chain the dashboard needs.

    python strip_urdf.py CAPTURE OUT

CAPTURE is the output of ``ros2 topic echo /robot_description --once --field data``
(any text encoding Python can sniff: PowerShell's ``>`` writes UTF-16 with a BOM, and
``ros2 topic echo`` appends a ``---`` separator after the document).

The skeleton renderer uses only link names and joint origins, axes, limits and
parent/child links. The vendor file also carries 104 ``file:///opt/ros/...`` mesh
references that do not exist inside the driver image, inertial blocks, four
``<gazebo>`` blocks and five ``<ros2_control>`` blocks. The last two contain nested
``<joint name=...>`` elements that a tag-based parser would mistake for real joints, so
they are removed along with the geometry. Joint values are copied verbatim.
"""
from __future__ import annotations

import hashlib
import sys
import xml.etree.ElementTree as ET

DROP_FROM_LINK = ("visual", "collision", "inertial")
DROP_FROM_ROBOT = ("gazebo", "ros2_control", "transmission")


def decode(blob: bytes) -> str:
    if blob.startswith((b"\xff\xfe", b"\xfe\xff")):
        text = blob.decode("utf-16")
    else:
        text = blob.decode("utf-8-sig")
    end = text.rfind("</robot>")
    if end < 0:
        raise ValueError("no closing </robot> in capture")
    return text[: end + len("</robot>")]


def strip(text: str) -> ET.Element:
    root = ET.fromstring(text.encode("utf-8"))
    for tag in DROP_FROM_ROBOT:
        for element in root.findall(tag):
            root.remove(element)
    for link in root.findall("link"):
        for tag in DROP_FROM_LINK:
            for element in link.findall(tag):
                link.remove(element)
    return root


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    blob = open(argv[0], "rb").read()
    root = strip(decode(blob))
    ET.indent(root)
    body = ET.tostring(root, encoding="unicode")
    joints = root.findall("joint")
    header = (
        '<?xml version="1.0"?>\n'
        "<!-- Kinematic chain of the UniX AI Panther (robot name mh3), reduced from the\n"
        "     robot's /robot_description by tools/strip_urdf.py. Geometry, inertia, gazebo\n"
        "     and ros2_control blocks removed; link/joint values are the vendor's. See\n"
        f"     resource/SOURCE.md. Source capture sha256 {hashlib.sha256(blob).hexdigest()}. -->\n"
    )
    with open(argv[1], "w", encoding="utf-8", newline="\n") as out:
        out.write(header + body + "\n")
    print(f"{len(root.findall('link'))} links, {len(joints)} joints -> {argv[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
