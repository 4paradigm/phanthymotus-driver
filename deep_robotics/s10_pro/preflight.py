#!/usr/bin/env python3
"""Inspect installed S10 ROS message definitions without starting ROS or DDS.

Generate a public structural contract in the container and compare it on the
robot before enabling the reader. Package release numbers are informational;
ordered fields, primitive widths, array bounds and nested message types decide
whether these six subscriptions have the same installed message definitions.
No vendor shell setup, controller, participant, service, or subprocess is run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import sys
from collections.abc import Mapping
from pathlib import Path

try:
    from .reader import load_config, source_specs
except ImportError:  # Standalone invocation in the driver container.
    from reader import load_config, source_specs


CONTRACT_SCHEMA = "s10_pro.ros_contract/1"
REPORT_SCHEMA = "s10_pro.preflight/1"
MAX_TYPES = 512
MAX_FIELDS = 512
MAX_REFERENCE_BYTES = 2 * 1024 * 1024
_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_PRIMITIVES = {
    "bool": "bool", "boolean": "bool",
    "byte": "octet", "octet": "octet", "char": "char", "wchar": "wchar",
    "float": "float32", "float32": "float32",
    "double": "float64", "float64": "float64", "long double": "long double",
    "int8": "int8", "uint8": "uint8",
    "int16": "int16", "uint16": "uint16", "short": "int16", "unsigned short": "uint16",
    "int32": "int32", "uint32": "uint32", "long": "int32", "unsigned long": "uint32",
    "int64": "int64", "uint64": "uint64", "long long": "int64", "unsigned long long": "uint64",
}


def _message_name(name: str, owner_package: str | None = None) -> str:
    if not isinstance(name, str):
        raise ValueError("unsupported ROS message type name")
    parts = name.replace("::", "/").split("/")
    if len(parts) == 1 and owner_package:
        parts = [owner_package, "msg", parts[0]]
    elif len(parts) == 2:
        parts = [parts[0], "msg", parts[1]]
    if len(parts) != 3 or parts[1] != "msg" or not all(_IDENTIFIER.fullmatch(p) for p in parts):
        raise ValueError("unsupported ROS message type name")
    return "/".join(parts)


def _size(value: str) -> int:
    if not re.fullmatch(r"[0-9]{1,10}", value):
        raise ValueError("invalid ROS container bound")
    result = int(value)
    if not 1 <= result <= 2**32 - 1:
        raise ValueError("invalid ROS container bound")
    return result


def _template_arguments(text: str) -> list[str]:
    depth = 0
    start = 0
    parts = []
    for index, char in enumerate(text):
        if char == "<" and not text[index:index + 2] == "<=":
            depth += 1
        elif char == ">":
            depth -= 1
            if depth < 0:
                raise ValueError("unbalanced ROS field type")
        elif char == "," and depth == 0:
            parts.append(text[start:index].strip())
            start = index + 1
    if depth:
        raise ValueError("unbalanced ROS field type")
    parts.append(text[start:].strip())
    if any(not part for part in parts):
        raise ValueError("empty ROS type argument")
    return parts


def parse_field_type(type_text: str, owner_package: str | None = None, *, _depth=0) -> dict:
    """Normalize generated rosidl field strings, including nested containers."""
    if not isinstance(type_text, str) or len(type_text) > 4096 or _depth > 32:
        raise ValueError("invalid or overly nested ROS field type")
    text = re.sub(r"\s+", " ", type_text.strip())
    primitive = _PRIMITIVES.get(text)
    if primitive is not None:
        return {"kind": "primitive", "name": primitive}
    string = re.fullmatch(r"(w?string)(?:\s*<=\s*([0-9]+)|\s*<\s*([0-9]+)\s*>)?", text)
    if string:
        bound = string.group(2) or string.group(3)
        return {"kind": "string", "wide": string.group(1) == "wstring",
                "max_size": _size(bound) if bound is not None else None}
    postfix = re.fullmatch(r"(.+)\[\s*(<=\s*)?([0-9]+)\s*\]", text)
    if postfix:
        value = parse_field_type(postfix.group(1), owner_package, _depth=_depth + 1)
        bound = _size(postfix.group(3))
        if postfix.group(2):
            return {"kind": "sequence", "max_size": bound, "value": value}
        return {"kind": "array", "size": bound, "value": value}
    if text.endswith("[]"):
        return {"kind": "sequence", "max_size": None,
                "value": parse_field_type(text[:-2], owner_package, _depth=_depth + 1)}
    template = re.fullmatch(r"(sequence|bounded_sequence|array)\s*<(.*)>", text)
    if template:
        kind, inner = template.groups()
        arguments = _template_arguments(inner)
        if len(arguments) not in (1, 2) or kind != "sequence" and len(arguments) != 2:
            raise ValueError("invalid ROS container arguments")
        value = parse_field_type(arguments[0], owner_package, _depth=_depth + 1)
        bound = _size(arguments[1]) if len(arguments) == 2 else None
        if kind == "array":
            return {"kind": "array", "size": bound, "value": value}
        return {"kind": "sequence", "max_size": bound, "value": value}
    return {"kind": "message", "type": _message_name(text, owner_package)}


def _nested_messages(descriptor):
    if descriptor["kind"] == "message":
        yield descriptor["type"]
    elif descriptor["kind"] in ("array", "sequence"):
        yield from _nested_messages(descriptor["value"])


def collect_contract(specs: Mapping, get_message) -> dict:
    """Load class metadata recursively, without constructing a message instance."""
    types = {}
    sources = {}

    def visit(type_name):
        if type_name in types:
            return
        if len(types) >= MAX_TYPES:
            raise ValueError("too many recursive message definitions")
        message_class = get_message(type_name)
        fields = message_class.get_fields_and_field_types()
        if not isinstance(fields, Mapping) or len(fields) > MAX_FIELDS:
            raise ValueError("invalid generated message field mapping")
        signature = []
        # Add before traversing dependencies to terminate a recursive type graph.
        types[type_name] = {"fields": signature}
        package = type_name.split("/", 1)[0]
        for name, field_type in fields.items():
            if not isinstance(name, str) or not _IDENTIFIER.fullmatch(name):
                raise ValueError("invalid generated message field name")
            descriptor = parse_field_type(field_type, package)
            signature.append({"name": name, "type": descriptor})
            for nested_type in _nested_messages(descriptor):
                visit(nested_type)

    for key in sorted(specs):
        item = specs[key]
        type_name = _message_name(item["type"])
        sources[key] = {"topic": item["topic"], "type": type_name}
        visit(type_name)
    contract = {"schema": CONTRACT_SCHEMA, "sources": sources,
                "types": {name: types[name] for name in sorted(types)}}
    structural = _contract_structure(contract)
    encoded = json.dumps(structural, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    contract["fingerprint_sha256"] = hashlib.sha256(encoded.encode("ascii")).hexdigest()
    return contract


def _clean_descriptor(value, depth=0):
    """Validate reference data and strip release annotations before comparison."""
    if not isinstance(value, dict) or depth > 32:
        raise ValueError("invalid contract field descriptor")
    kind = value.get("kind")
    if kind == "primitive" and value.get("name") in set(_PRIMITIVES.values()):
        return {"kind": kind, "name": value["name"]}
    if kind == "message":
        return {"kind": kind, "type": _message_name(value["type"])}
    if kind == "string":
        if not isinstance(value.get("wide"), bool):
            raise ValueError("invalid contract string width")
        bound = value.get("max_size")
        if bound is not None:
            if isinstance(bound, bool) or not isinstance(bound, int):
                raise ValueError("invalid contract string bound")
            bound = _size(str(bound))
        return {"kind": kind, "wide": value["wide"], "max_size": bound}
    if kind in ("sequence", "array"):
        bound_name = "size" if kind == "array" else "max_size"
        bound = value.get(bound_name)
        if bound is not None:
            if isinstance(bound, bool) or not isinstance(bound, int):
                raise ValueError("invalid contract container bound")
            bound = _size(str(bound))
        elif kind == "array":
            raise ValueError("array contract must have a fixed length")
        return {"kind": kind, bound_name: bound,
                "value": _clean_descriptor(value["value"], depth + 1)}
    raise ValueError("unsupported contract descriptor")


def _contract_structure(document):
    if not isinstance(document, dict):
        raise ValueError("contract must be an object")
    if "contract" in document:
        document = document["contract"]
    if not isinstance(document, dict) or document.get("schema") != CONTRACT_SCHEMA:
        raise ValueError("unsupported contract schema")
    raw_sources, raw_types = document.get("sources"), document.get("types")
    if not isinstance(raw_sources, dict) or not isinstance(raw_types, dict) or len(raw_types) > MAX_TYPES:
        raise ValueError("invalid contract source/type mapping")
    sources = {}
    for key, item in raw_sources.items():
        if not isinstance(key, str) or not isinstance(item, dict):
            raise ValueError("invalid contract source")
        # Topic remapping is configuration, independent of a DDS type layout.
        sources[key] = {"type": _message_name(item["type"])}
    types = {}
    for name, item in raw_types.items():
        type_name = _message_name(name)
        if type_name in types or not isinstance(item, dict):
            raise ValueError("duplicate or invalid contract message type")
        fields = item.get("fields")
        if not isinstance(fields, list) or len(fields) > MAX_FIELDS:
            raise ValueError("invalid contract field list")
        clean = []
        names = set()
        for field in fields:
            if not isinstance(field, dict) or not isinstance(field.get("name"), str):
                raise ValueError("invalid contract field")
            name = field["name"]
            if name in names or not _IDENTIFIER.fullmatch(name):
                raise ValueError("duplicate or invalid contract field name")
            names.add(name)
            clean.append({"name": name, "type": _clean_descriptor(field["type"])})
        types[type_name] = {"fields": clean}
    # A partial reference could hide a mismatch in a dependency. Require every
    # source and every nested message named by a field to have its definition.
    required = {source["type"] for source in sources.values()}
    for definition in types.values():
        for field in definition["fields"]:
            required.update(_nested_messages(field["type"]))
    if required - types.keys():
        raise ValueError("contract omits a referenced message definition")
    return {"sources": sources, "types": types}


def compare_contracts(actual, expected) -> dict:
    """Compare complete ordered structures, ignoring version labels and hashes."""
    mismatches = []
    try:
        actual = _contract_structure(actual)
        expected = _contract_structure(expected)
    except (ValueError, TypeError, KeyError, AttributeError):
        return {"compatible": False, "mismatches": [{"path": "$", "reason": "invalid_or_incomplete_contract"}]}

    def diff(left, right, path):
        if len(mismatches) >= 100:
            return
        if type(left) is not type(right):
            mismatches.append({"path": path, "actual": left, "expected": right})
        elif isinstance(left, dict):
            for key in sorted(left.keys() | right.keys()):
                if key not in left or key not in right:
                    mismatches.append({"path": f"{path}.{key}", "actual": left.get(key), "expected": right.get(key)})
                else:
                    diff(left[key], right[key], f"{path}.{key}")
                if len(mismatches) >= 100:
                    return
        elif isinstance(left, list):
            if len(left) != len(right):
                mismatches.append({"path": f"{path}.length", "actual": len(left), "expected": len(right)})
            for index, (lvalue, rvalue) in enumerate(zip(left, right)):
                diff(lvalue, rvalue, f"{path}[{index}]")
        elif left != right:
            mismatches.append({"path": path, "actual": left, "expected": right})

    diff(actual, expected, "$")
    return {"compatible": not mismatches, "mismatches": mismatches}


def _version_metadata(path: Path) -> dict:
    """Expose a plain version token, never arbitrary vendor file contents."""
    if not path.exists():
        return {"status": "missing"}
    if not path.is_file():
        return {"status": "not_regular_file"}
    try:
        if path.stat().st_size > 4096:
            return {"status": "unrecognized"}
        text = path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return {"status": "unreadable"}
    if re.fullmatch(r"[vV]?[0-9]+\.[0-9]+(?:\.[0-9]+)?(?:[-+][A-Za-z0-9._-]+)?", text):
        return {"status": "read", "version": text}
    return {"status": "unrecognized"}


def host_metadata() -> dict:
    """Allowlisted, non-secret local metadata; no environment dump or commands."""
    def identifier_env(name):
        value = os.environ.get(name)
        return value if value and re.fullmatch(r"[A-Za-z0-9_]{1,80}", value) else None

    return {
        "python_version": platform.python_version(),
        "architecture": platform.machine(),
        "system": platform.system(),
        "ros_distro": identifier_env("ROS_DISTRO"),
        "rmw_implementation": identifier_env("RMW_IMPLEMENTATION"),
        "tools": {name: shutil.which(name) is not None for name in ("python3", "ros2", "colcon", "docker")},
        "robot_version": _version_metadata(Path("/opt/robot/version")),
    }


def _message_getter():
    # rosidl introspection imports class metadata, without importing rclpy or
    # creating a Context. get_message resolves generated message classes only.
    from rosidl_runtime_py.utilities import get_message

    return get_message


def _load_reference(path: Path):
    if path.stat().st_size > MAX_REFERENCE_BYTES:
        raise ValueError("reference contract exceeds size limit")
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--public-contract", action="store_true", help="omit local host metadata")
    parser.add_argument("--compare-contract", type=Path, help="compare with a generated public contract")
    args = parser.parse_args(argv)
    try:
        specs = source_specs(load_config(args.config))
        contract = collect_contract(specs, _message_getter())
        report = dict(contract) if args.public_contract else {
            "schema": REPORT_SCHEMA, "contract": contract, "host": host_metadata(),
        }
        exit_code = 0
        if args.compare_contract is not None:
            comparison = compare_contracts(contract, _load_reference(args.compare_contract))
            report["comparison"] = comparison
            if not comparison["compatible"]:
                exit_code = 2
        print(json.dumps(report, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":")))
        return exit_code
    except Exception as exc:
        # A bad setup can include private paths or credentials in an exception.
        # Only its class is emitted; no full environment or file body is copied.
        print(json.dumps({"schema": REPORT_SCHEMA, "status": "error", "error_type": type(exc).__name__}, sort_keys=True))
        print(f"[s10-preflight] failed ({type(exc).__name__})", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
