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
    return raw.decode("utf-8")
