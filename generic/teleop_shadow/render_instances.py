"""Render validated teleop-shadow instances into deterministic Docker Compose."""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import ssl
import stat
import sys
import tempfile
from urllib.parse import urlsplit
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import yaml
from yaml.constructor import ConstructorError
from yaml.nodes import MappingNode

SCHEMA_VERSION = 4
MIN_MCP_PORT = 15700
MAX_MCP_PORT = 15799
MIN_CAPTURE_PORT = 1024
MAX_CAPTURE_PORT = 65535
MAX_INPUT_BYTES = 256 * 1024
MAX_INSTANCES = 64
MAX_CORE_CA_BYTES = 4 * 1024 * 1024
CORE_CA_TARGET = "/etc/motus-core-ca/core-ca.pem"
CORE_CA_SOURCE_ROOT = pathlib.Path("/opt/phanthy-motus")
REGISTRATION_COORDINATION_TARGET = "/var/lib/motus-registration-coordination"
REGISTRATION_COORDINATION_FILE = (
    f"{REGISTRATION_COORDINATION_TARGET}/registration.json"
)
AGENT_CORE_URL = "https://localhost:15678"
_ROOT_FIELDS = frozenset(
    {
        "schema_version",
        "core_ca_file",
        "registration_coordination_dir",
        "instances",
    }
)
_INSTANCE_FIELDS = frozenset(
    {
        "service",
        "container",
        "driver_id",
        "driver_name",
        "robot_id",
        "mcp_port",
        "capture_port",
        "capture_wss_url",
        "capture_tls_dir",
        "capture_state_dir",
    }
)
_COMPOSE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,62}$")
_INSTANCE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_IMAGE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._/@:+-]{0,254}$")
_SAFE_ABSOLUTE_PATH_RE = re.compile(r"^/[A-Za-z0-9._/-]+$")
_PRIVATE_KEY_PEM_RE = re.compile(br"-----BEGIN [A-Z0-9 ]*PRIVATE KEY-----")
_PEM_BLOCK_RE = re.compile(
    br"-----BEGIN ([A-Z0-9][A-Z0-9 ]*)-----\r?\n.*?-----END \1-----",
    re.DOTALL,
)
_PUBLIC_CERTIFICATE_LABELS = frozenset({b"CERTIFICATE", b"TRUSTED CERTIFICATE"})


class RenderError(ValueError):
    """The instances document cannot safely produce a Compose deployment."""


class _StrictSafeLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate mapping keys."""

    def construct_mapping(self, node: MappingNode, deep: bool = False) -> dict:
        if not isinstance(node, MappingNode):
            raise ConstructorError(
                None,
                None,
                f"expected a mapping node, got {node.id}",
                node.start_mark,
            )
        self.flatten_mapping(node)
        mapping: dict[Any, Any] = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                duplicate = key in mapping
            except TypeError as exc:
                raise ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    "found an unhashable key",
                    key_node.start_mark,
                ) from exc
            if duplicate:
                raise ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"found duplicate key {key!r}",
                    key_node.start_mark,
                )
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


@dataclass(frozen=True, slots=True)
class InstanceSpec:
    service: str
    container: str
    driver_id: str
    driver_name: str
    robot_id: str
    mcp_port: int
    capture_port: int
    capture_wss_url: str
    capture_tls_dir: str
    capture_state_dir: str


@dataclass(frozen=True, slots=True)
class DeploymentSpec:
    core_ca_file: str
    registration_coordination_dir: str | None
    instances: tuple[InstanceSpec, ...]


def _mapping(value: object, location: str) -> dict:
    if type(value) is not dict:
        raise RenderError(f"{location} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise RenderError(f"{location} field names must be strings")
    return value


def _reject_unknown_and_missing(
    value: dict,
    allowed: frozenset[str],
    location: str,
) -> None:
    unknown = sorted(set(value) - allowed)
    missing = sorted(allowed - set(value))
    if unknown:
        raise RenderError(f"{location} has unknown fields: {', '.join(unknown)}")
    if missing:
        raise RenderError(f"{location} is missing fields: {', '.join(missing)}")


def _compose_name(value: object, field: str) -> str:
    if not isinstance(value, str) or not _COMPOSE_NAME_RE.fullmatch(value):
        raise RenderError(
            f"{field} must match {_COMPOSE_NAME_RE.pattern!r} and be at most 63 characters"
        )
    return value


def _instance_id(value: object, field: str, max_length: int) -> str:
    if (
        not isinstance(value, str)
        or len(value) > max_length
        or not _INSTANCE_ID_RE.fullmatch(value)
    ):
        raise RenderError(
            f"{field} must be a stable 1-{max_length} character instance identifier"
        )
    return value


def _driver_name(value: object, field: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 200
        or value != value.strip()
        or not value.isprintable()
        or "$" in value
    ):
        raise RenderError(
            f"{field} must be a trimmed printable string of 1-200 characters without '$'"
        )
    return value


def _mcp_port(value: object, field: str) -> int:
    if type(value) is not int or not MIN_MCP_PORT <= value <= MAX_MCP_PORT:
        raise RenderError(
            f"{field} must be an integer in the Driver range {MIN_MCP_PORT}-{MAX_MCP_PORT}"
        )
    return value


def _capture_port(value: object, field: str) -> int:
    if type(value) is not int or not MIN_CAPTURE_PORT <= value <= MAX_CAPTURE_PORT:
        raise RenderError(
            f"{field} must be an integer in {MIN_CAPTURE_PORT}-{MAX_CAPTURE_PORT}"
        )
    return value


def _managed_directory(value: object, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) > 1024
        or not _SAFE_ABSOLUTE_PATH_RE.fullmatch(value)
    ):
        raise RenderError(f"{field} must be a safe absolute POSIX directory path")
    path = pathlib.PurePosixPath(value)
    root = pathlib.PurePosixPath(CORE_CA_SOURCE_ROOT.as_posix())
    if root not in path.parents or ".." in path.parts or str(path) != value:
        raise RenderError(
            f"{field} must be normalized below {CORE_CA_SOURCE_ROOT.as_posix()}"
        )
    return _validate_managed_directory_on_host(pathlib.Path(value), field)


def _registration_coordination_directory(value: object) -> str | None:
    if value is None:
        return None
    return _managed_directory(value, "registration_coordination_dir")


def _capture_wss_url(value: object, field: str, capture_port: int) -> str:
    if not isinstance(value, str) or len(value) > 2048:
        raise RenderError(f"{field} must be a bounded wss URL")
    parsed = urlsplit(value)
    try:
        url_port = parsed.port
    except ValueError as exc:
        raise RenderError(f"{field} contains an invalid port") from exc
    if (
        parsed.scheme != "wss"
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path != "/ws/teleop-capture"
        or url_port != capture_port
    ):
        raise RenderError(
            f"{field} must be wss://<host>:{capture_port}/ws/teleop-capture"
        )
    return value


def _core_ca_file(value: object) -> str:
    if (
        not isinstance(value, str)
        or len(value) > 1024
        or not _SAFE_ABSOLUTE_PATH_RE.fullmatch(value)
    ):
        raise RenderError(
            "core_ca_file must be a safe absolute POSIX file path without interpolation"
        )
    path = pathlib.PurePosixPath(value)
    if path == pathlib.PurePosixPath("/") or ".." in path.parts or str(path) != value:
        raise RenderError("core_ca_file must be normalized and must not contain '..'")
    lexical_root = pathlib.PurePosixPath(CORE_CA_SOURCE_ROOT.as_posix())
    if lexical_root not in path.parents or path.suffix.lower() not in {".pem", ".crt"}:
        raise RenderError(
            "core_ca_file must be a .pem or .crt file below /opt/phanthy-motus"
        )
    return _validate_core_ca_file_on_host(pathlib.Path(value))


def _assert_no_symlink_components(path: pathlib.Path, field: str) -> None:
    """Reject a symlink at the leaf or in any existing absolute ancestor."""

    if not path.is_absolute():
        raise RenderError(f"{field} must be an absolute host path")
    current = pathlib.Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        try:
            metadata = current.lstat()
        except OSError as exc:
            raise RenderError(
                f"{field} must be an existing host path with no symlink components"
            ) from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise RenderError(
                f"{field} must not use a symlink or symlink ancestor: {current}"
            )


def _canonical_host_path(
    path: pathlib.Path,
    field: str,
    *,
    require_directory: bool,
) -> pathlib.Path:
    """Return an existing canonical path below the canonical deployment root."""

    root = CORE_CA_SOURCE_ROOT
    _assert_no_symlink_components(root, "host deployment root")
    _assert_no_symlink_components(path, field)
    try:
        resolved_root = root.resolve(strict=True)
        resolved = path.resolve(strict=True)
        resolved.relative_to(resolved_root)
        metadata = path.stat()
    except (OSError, RuntimeError, ValueError) as exc:
        raise RenderError(
            f"{field} must resolve to an existing path inside "
            f"{CORE_CA_SOURCE_ROOT.as_posix()}"
        ) from exc
    if resolved_root != root or resolved != path:
        raise RenderError(
            f"{field} and the host deployment root must use canonical real paths"
        )
    expected_type = stat.S_ISDIR if require_directory else stat.S_ISREG
    if not expected_type(metadata.st_mode):
        kind = "directory" if require_directory else "regular file"
        raise RenderError(f"{field} must resolve to a {kind}")
    return resolved


def _validate_managed_directory_on_host(path: pathlib.Path, field: str) -> str:
    """Validate and canonicalize one host bind-source directory."""

    return str(
        _canonical_host_path(path, field, require_directory=True)
    )


def _validate_core_ca_file_on_host(path: pathlib.Path) -> str:
    """Return a canonical, public X.509-only CA path inside the deployment root."""

    resolved = _canonical_host_path(
        path,
        "core_ca_file",
        require_directory=False,
    )
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    flags |= getattr(os, "O_NONBLOCK", 0)
    descriptor = -1
    try:
        descriptor = os.open(resolved, flags)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise RenderError("core_ca_file must resolve to a regular file")
        if not 1 <= metadata.st_size <= MAX_CORE_CA_BYTES:
            raise RenderError(
                f"core_ca_file must contain 1-{MAX_CORE_CA_BYTES} bytes"
            )
        with os.fdopen(descriptor, "rb") as handle:
            descriptor = -1
            encoded = handle.read(MAX_CORE_CA_BYTES + 1)
    except OSError as exc:
        raise RenderError("core_ca_file must be a readable regular file") from exc
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    if not 1 <= len(encoded) <= MAX_CORE_CA_BYTES:
        raise RenderError(f"core_ca_file must contain 1-{MAX_CORE_CA_BYTES} bytes")
    if _PRIVATE_KEY_PEM_RE.search(encoded):
        raise RenderError("core_ca_file must never contain a private-key PEM block")
    position = 0
    block_count = 0
    for match in _PEM_BLOCK_RE.finditer(encoded):
        if encoded[position:match.start()].strip():
            raise RenderError("core_ca_file must contain only public X.509 PEM blocks")
        if match.group(1) not in _PUBLIC_CERTIFICATE_LABELS:
            raise RenderError("core_ca_file must contain only public X.509 PEM blocks")
        block_count += 1
        position = match.end()
    if block_count == 0 or encoded[position:].strip():
        raise RenderError("core_ca_file must contain only public X.509 PEM blocks")
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.load_verify_locations(cadata=encoded.decode("ascii"))
    except (OSError, UnicodeError, ssl.SSLError) as exc:
        raise RenderError(
            "core_ca_file must contain a parseable X.509 certificate or CA bundle"
        ) from exc
    return str(resolved)


def _parse_instance(raw: object, index: int) -> InstanceSpec:
    location = f"instances[{index}]"
    value = _mapping(raw, location)
    _reject_unknown_and_missing(value, _INSTANCE_FIELDS, location)
    capture_port = _capture_port(value["capture_port"], f"{location}.capture_port")
    return InstanceSpec(
        service=_compose_name(value["service"], f"{location}.service"),
        container=_compose_name(value["container"], f"{location}.container"),
        driver_id=_instance_id(
            value["driver_id"], f"{location}.driver_id", max_length=64
        ),
        driver_name=_driver_name(value["driver_name"], f"{location}.driver_name"),
        robot_id=_instance_id(
            value["robot_id"], f"{location}.robot_id", max_length=64
        ),
        mcp_port=_mcp_port(value["mcp_port"], f"{location}.mcp_port"),
        capture_port=capture_port,
        capture_wss_url=_capture_wss_url(
            value["capture_wss_url"],
            f"{location}.capture_wss_url",
            capture_port,
        ),
        capture_tls_dir=_managed_directory(
            value["capture_tls_dir"], f"{location}.capture_tls_dir"
        ),
        capture_state_dir=_managed_directory(
            value["capture_state_dir"], f"{location}.capture_state_dir"
        ),
    )


def _require_unique(instances: Sequence[InstanceSpec], field: str) -> None:
    seen: dict[object, int] = {}
    for index, instance in enumerate(instances):
        value = getattr(instance, field)
        if value in seen:
            raise RenderError(
                f"instances[{index}].{field} duplicates instances[{seen[value]}].{field}"
            )
        seen[value] = index


def _paths_overlap(left: str, right: str) -> bool:
    """Return whether two normalized POSIX paths share an ownership tree."""

    left_path = pathlib.PurePosixPath(left)
    right_path = pathlib.PurePosixPath(right)
    return (
        left_path == right_path
        or left_path in right_path.parents
        or right_path in left_path.parents
    )


def _path_and_parents(path: pathlib.Path) -> tuple[pathlib.Path, ...]:
    return (path, *path.parents)


def _host_paths_overlap(left: str, right: str) -> bool:
    """Compare canonical ownership trees, including same-file host aliases."""

    if _paths_overlap(left, right):
        return True
    left_path = pathlib.Path(left)
    right_path = pathlib.Path(right)
    try:
        return any(
            left_path.samefile(candidate)
            for candidate in _path_and_parents(right_path)
        ) or any(
            right_path.samefile(candidate)
            for candidate in _path_and_parents(left_path)
        )
    except OSError as exc:
        raise RenderError(
            "canonical host deployment paths could not be compared safely"
        ) from exc


def _require_isolated_storage(
    instances: Sequence[InstanceSpec],
    core_ca_file: str,
    registration_coordination_dir: str | None,
) -> None:
    managed = [
        (index, field, getattr(instance, field))
        for index, instance in enumerate(instances)
        for field in ("capture_tls_dir", "capture_state_dir")
    ]
    for position, (left_index, left_field, left_path) in enumerate(managed):
        for right_index, right_field, right_path in managed[position + 1:]:
            if _host_paths_overlap(left_path, right_path):
                raise RenderError(
                    f"instances[{left_index}].{left_field} overlaps "
                    f"instances[{right_index}].{right_field}"
                )
        if _host_paths_overlap(left_path, core_ca_file):
            raise RenderError(
                f"instances[{left_index}].{left_field} overlaps core_ca_file"
            )
        if (
            registration_coordination_dir is not None
            and _host_paths_overlap(left_path, registration_coordination_dir)
        ):
            raise RenderError(
                f"instances[{left_index}].{left_field} overlaps "
                "registration_coordination_dir"
            )
    if (
        registration_coordination_dir is not None
        and _host_paths_overlap(registration_coordination_dir, core_ca_file)
    ):
        raise RenderError(
            "registration_coordination_dir overlaps core_ca_file"
        )


def parse_instances_document(document: object) -> DeploymentSpec:
    """Validate a loaded instances document and return its typed form."""

    root = _mapping(document, "document")
    _reject_unknown_and_missing(root, _ROOT_FIELDS, "document")
    if type(root["schema_version"]) is not int or root["schema_version"] != SCHEMA_VERSION:
        raise RenderError(f"schema_version must be exactly {SCHEMA_VERSION}")
    raw_instances = root["instances"]
    if type(raw_instances) is not list:
        raise RenderError("instances must be a list")
    if not 1 <= len(raw_instances) <= MAX_INSTANCES:
        raise RenderError(f"instances must contain 1-{MAX_INSTANCES} entries")

    instances = tuple(
        _parse_instance(raw_instance, index)
        for index, raw_instance in enumerate(raw_instances)
    )
    for field in (
        "service", "container", "driver_id", "robot_id", "mcp_port",
        "capture_port", "capture_wss_url", "capture_tls_dir", "capture_state_dir",
    ):
        _require_unique(instances, field)
    all_ports = [
        port
        for instance in instances
        for port in (instance.mcp_port, instance.capture_port)
    ]
    if len(all_ports) != len(set(all_ports)):
        raise RenderError("all MCP and Capture listener ports must be globally unique")
    core_ca_file = _core_ca_file(root["core_ca_file"])
    registration_coordination_dir = _registration_coordination_directory(
        root["registration_coordination_dir"]
    )
    if len(instances) > 1 and registration_coordination_dir is None:
        raise RenderError(
            "registration_coordination_dir is required for multi-instance deployment"
        )
    _require_isolated_storage(
        instances,
        core_ca_file,
        registration_coordination_dir,
    )
    return DeploymentSpec(
        core_ca_file=core_ca_file,
        registration_coordination_dir=registration_coordination_dir,
        instances=instances,
    )


def load_instances(path: str | pathlib.Path) -> DeploymentSpec:
    """Load a bounded UTF-8 YAML file with duplicate-key rejection."""

    input_path = pathlib.Path(path)
    if not input_path.is_file():
        raise RenderError(f"instances input is not a regular file: {input_path}")
    try:
        with input_path.open("rb") as handle:
            encoded = handle.read(MAX_INPUT_BYTES + 1)
        if len(encoded) > MAX_INPUT_BYTES:
            raise RenderError(f"instances input exceeds {MAX_INPUT_BYTES} bytes")
        text = encoded.decode("utf-8")
        document = yaml.load(text, Loader=_StrictSafeLoader)
    except UnicodeError as exc:
        raise RenderError("instances input must be valid UTF-8") from exc
    except RecursionError as exc:
        raise RenderError("instances input YAML nesting is too deep") from exc
    except yaml.YAMLError as exc:
        raise RenderError(f"instances input is invalid YAML: {exc}") from exc
    return parse_instances_document(document)


def validate_image(image: object) -> str:
    """Accept a conservative Compose-safe image reference."""

    if not isinstance(image, str) or not _IMAGE_RE.fullmatch(image) or ".." in image:
        raise RenderError(
            "image must be a 1-255 character container reference without whitespace or interpolation"
        )
    return image


def _service(
    instance: InstanceSpec,
    deployment: DeploymentSpec,
    image: str,
) -> dict:
    port = instance.mcp_port
    volumes = [
        {
            "type": "bind",
            "source": deployment.core_ca_file,
            "target": CORE_CA_TARGET,
            "read_only": True,
            "bind": {"create_host_path": False},
        },
        {
            "type": "bind",
            "source": instance.capture_tls_dir,
            "target": "/etc/motus-capture-tls",
            "read_only": True,
            "bind": {"create_host_path": False},
        },
        {
            "type": "bind",
            "source": instance.capture_state_dir,
            "target": "/var/lib/motus-teleop-shadow",
            "read_only": False,
            "bind": {"create_host_path": False},
        },
    ]
    environment = [
        "MOTUS_BIND_HOST=127.0.0.1",
        f"MOTUS_MCP_PORT={port}",
        f"MOTUS_MCP_URL=http://localhost:{port}/mcp",
        f"MOTUS_DRIVER_ID={instance.driver_id}",
        f"MOTUS_DRIVER_NAME={instance.driver_name}",
        f"MOTUS_ROBOT_ID={instance.robot_id}",
        "MOTUS_CAPTURE_BIND_HOST=0.0.0.0",
        f"MOTUS_CAPTURE_PORT={instance.capture_port}",
        f"MOTUS_CAPTURE_WSS_URL={instance.capture_wss_url}",
        "MOTUS_CAPTURE_TLS_CERT_FILE=/etc/motus-capture-tls/cert.pem",
        "MOTUS_CAPTURE_TLS_KEY_FILE=/etc/motus-capture-tls/key.pem",
        "MOTUS_CAPTURE_STATE_FILE=/var/lib/motus-teleop-shadow/capture.json",
        f"AGENT_CORE_URL={AGENT_CORE_URL}",
        "MOTUS_AGENT_CORE_VERIFY_TLS=1",
        f"MOTUS_AGENT_CORE_CA_FILE={CORE_CA_TARGET}",
        "PYTHONUNBUFFERED=1",
    ]
    if deployment.registration_coordination_dir is not None:
        volumes.append(
            {
                "type": "bind",
                "source": deployment.registration_coordination_dir,
                "target": REGISTRATION_COORDINATION_TARGET,
                "read_only": False,
                "bind": {"create_host_path": False},
            }
        )
        environment.insert(
            -1,
            f"MOTUS_REGISTRATION_COORDINATION_FILE={REGISTRATION_COORDINATION_FILE}",
        )
    return {
        "image": image,
        "container_name": instance.container,
        "network_mode": "host",
        "volumes": volumes,
        "environment": environment,
        "logging": {
            "driver": "local",
            "options": {"max-size": "10m", "max-file": "3"},
        },
        "restart": "unless-stopped",
    }


def render_compose(deployment: DeploymentSpec, image: str) -> str:
    """Render canonical Compose YAML without consulting process secrets."""

    safe_image = validate_image(image)
    ordered_instances = sorted(deployment.instances, key=lambda item: item.service)
    services = {
        instance.service: _service(instance, deployment, safe_image)
        for instance in ordered_instances
    }
    rendered = yaml.safe_dump(
        {"services": services},
        allow_unicode=True,
        default_flow_style=False,
        sort_keys=False,
        width=1000,
    )
    return "# Generated by render_instances.py; contains no secret values.\n" + rendered


def atomic_write(path: str | pathlib.Path, content: str) -> None:
    """Atomically replace a regular output file in an existing directory."""

    output_path = pathlib.Path(path)
    parent = output_path.parent
    if not parent.is_dir():
        raise RenderError(f"output parent is not a directory: {parent}")
    if output_path.is_symlink() or (output_path.exists() and not output_path.is_file()):
        raise RenderError(f"output must be a regular file path: {output_path}")

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.",
        suffix=".tmp",
        dir=parent,
        text=True,
    )
    temporary_path = pathlib.Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            os.fchmod(handle.fileno(), 0o644)
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, output_path)
        directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_flags |= getattr(os, "O_CLOEXEC", 0)
        directory_fd = os.open(parent, directory_flags)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary_path.exists() or temporary_path.is_symlink():
            temporary_path.unlink()


def require_distinct_input_output(
    input_path: str | pathlib.Path,
    output_path: str | pathlib.Path,
) -> None:
    """Refuse to replace the source YAML through a direct path or hard link."""

    source = pathlib.Path(input_path)
    destination = pathlib.Path(output_path)
    try:
        source_resolved = source.resolve(strict=True)
        destination_resolved = destination.resolve(strict=False)
        aliases = source_resolved == destination_resolved
        if destination.exists() and source.samefile(destination):
            aliases = True
    except OSError as exc:
        raise RenderError("instances input and output paths could not be compared safely") from exc
    if aliases:
        raise RenderError("output must not replace or alias the instances input file")


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render strict teleop-shadow instances into Docker Compose."
    )
    parser.add_argument("--instances", required=True, help="strict instances YAML input")
    parser.add_argument("--image", required=True, help="one image reference for all instances")
    parser.add_argument(
        "--output",
        default="teleop-shadow.compose.yml",
        help="atomic output path (default: teleop-shadow.compose.yml)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--stdout", action="store_true", help="render to stdout; write no file")
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and render to stdout; write no file",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _argument_parser()
    arguments = parser.parse_args(argv)
    try:
        deployment = load_instances(arguments.instances)
        rendered = render_compose(deployment, arguments.image)
        if arguments.stdout or arguments.dry_run:
            sys.stdout.write(rendered)
        else:
            require_distinct_input_output(arguments.instances, arguments.output)
            atomic_write(arguments.output, rendered)
            print(
                f"rendered {len(deployment.instances)} teleop-shadow instances to "
                f"{arguments.output}",
                file=sys.stderr,
            )
    except (OSError, RenderError) as exc:
        print(f"render error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
