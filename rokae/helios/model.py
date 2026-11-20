"""Read the user's installed iNex model at runtime; ship no manufacturer model."""

import json
import xml.etree.ElementTree as ET
from urllib.parse import urljoin, urlsplit, urlunsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .feedback import JOINT_NAMES, validate_url


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _origin(url):
    parts = urlsplit(url)
    return parts.scheme, parts.hostname, parts.port or (443 if parts.scheme == "https" else 80)


def _floats(text, default="0 0 0"):
    values = [float(v) for v in (text or default).split()]
    if len(values) != 3:
        raise ValueError("URDF vector must have three values")
    return values


def _fmt(values):
    return " ".join(f"{v:.10g}" for v in values)


def recenter_head(urdf):
    """Move head_link_2's frame origin onto the head centre line, display-only.

    The installed model puts head_link_2's origin 33 mm to one side, so lines
    drawn from it to the symmetrically mounted head cameras look lopsided.
    head_joint_2 rotates about its own y axis with no rotation offset, so
    sliding the frame along y leaves the rotation axis, every child position
    and the kinematics unchanged; children are shifted by the same amount.
    Any model that does not match these conditions is returned untouched.
    """
    root = ET.fromstring(urdf)
    joint = next((j for j in root.findall("joint") if j.get("name") == "head_joint_2"), None)
    if joint is None or joint.get("type") != "revolute":
        return urdf
    origin, axis, child = joint.find("origin"), joint.find("axis"), joint.find("child")
    if origin is None or axis is None or child is None:
        return urdf
    xyz, rpy = _floats(origin.get("xyz")), _floats(origin.get("rpy"))
    ax = _floats(axis.get("xyz"))
    norm = sum(a * a for a in ax) ** 0.5
    if any(abs(a) > 1e-9 for a in rpy) or norm == 0 or abs(abs(ax[1]) / norm - 1) > 1e-9 \
            or abs(xyz[1]) < 1e-6:
        return urdf
    offset = xyz[1]
    origin.set("xyz", _fmt([xyz[0], 0.0, xyz[2]]))
    link_name = child.get("link")

    def shift(element):
        o = element.find("origin")
        if o is None:
            o = ET.SubElement(element, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        v = _floats(o.get("xyz"))
        o.set("xyz", _fmt([v[0], v[1] + offset, v[2]]))

    for j in root.findall("joint"):
        parent = j.find("parent")
        if parent is not None and parent.get("link") == link_name:
            shift(j)
    for link in root.findall("link"):
        if link.get("name") == link_name:
            for tag in ("inertial", "visual", "collision"):
                for element in link.findall(tag):
                    shift(element)
    return ET.tostring(root, encoding="unicode")


def fetch_model(state_url, timeout=3):
    """Bounded, same-origin reads of model metadata and XML, without control calls."""
    parts = urlsplit(validate_url(state_url))
    base = urlunsplit(("https" if parts.scheme == "wss" else "http", parts.netloc, "/", "", ""))
    opener = build_opener(ProxyHandler({}), _NoRedirect())

    def read(url, limit):
        request = Request(url, headers={"Accept": "application/json, application/xml, text/xml"})
        with opener.open(request, timeout=timeout) as response:
            raw = response.read(limit + 1)
        if len(raw) > limit:
            raise ValueError("iNex model response exceeds size limit")
        return raw

    metadata = json.loads(read(urljoin(base, "api/robot_3d_model"), 65536))
    reference = metadata.get("urdf_url") if isinstance(metadata, dict) else None
    if not isinstance(reference, str) or not reference:
        raise ValueError("iNex model metadata has no urdf_url")
    url = urljoin(base, reference)
    target = urlsplit(url)
    if (target.username or target.password or target.fragment or _origin(url) != _origin(base)):
        raise ValueError("iNex urdf_url must stay on the state endpoint's HTTP origin")
    raw = read(url, 1024 * 1024)
    if b"<!DOCTYPE" in raw.upper() or b"<!ENTITY" in raw.upper():
        raise ValueError("model XML declarations are not supported")
    root = ET.fromstring(raw)
    movable = [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]
    if root.tag != "robot" or len(movable) != len(JOINT_NAMES) or set(movable) != set(JOINT_NAMES):
        raise ValueError("installed model movable joints do not match iNex state mapping")
    return recenter_head(raw.decode("utf-8"))
