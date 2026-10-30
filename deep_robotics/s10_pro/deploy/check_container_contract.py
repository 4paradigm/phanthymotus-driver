#!/usr/bin/env python3
"""Validate a local Compose candidate without connecting to Docker or a robot."""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

CORE_SERVICE = "agent-core"
CORE_NAME = "phanthy-motus-agent-core-1"
DRIVER_NAME = "embodied-deep-robotics-s10-pro"
DRIVER_SERVICE = "deep-robotics-s10-pro"


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys rather than silently dropping a service."""


def unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load(path):
    with Path(path).open() as stream:
        document = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(document, dict):
        raise ValueError("Expected a YAML mapping")
    return document


def validate_fragment(fragment):
    if set(fragment) != {DRIVER_SERVICE}:
        raise ValueError("The install fragment must contain exactly one S10 driver service")
    driver = fragment[DRIVER_SERVICE]
    if not isinstance(driver, dict) or driver.get("container_name") != DRIVER_NAME:
        raise ValueError("The S10 driver container name must remain unchanged")
    if driver.get("image") != "__IMAGE__":
        raise ValueError("The install fragment requires the platform __IMAGE__ placeholder")
    if driver.get("network_mode") != "host":
        raise ValueError("Platform DDS requires host networking")


def validate_compose(candidate, baseline):
    """Require one approved standard Core and exactly one driver.

    The baseline is the approved standard Core configuration, not necessarily
    the old customized S10 deployment. This checks fully resolved Compose documents. It does not merge overlays,
    substitute environment variables, remove services or rewrite Core YAML.
    """
    for document in (candidate, baseline):
        if not isinstance(document.get("services"), dict):
            raise ValueError("Expected a full Compose document with services")
        if any(key in document for key in ("include", "extends")):
            raise ValueError("Resolve Compose includes before validation")
        for service in document["services"].values():
            if not isinstance(service, dict) or any(key in service for key in ("extends", "profiles")):
                raise ValueError("Resolve service extensions and profiles before validation")
    services = candidate["services"]
    original = baseline["services"]
    cores = [key for key, service in original.items() if service.get("container_name") == CORE_NAME]
    if cores != [CORE_SERVICE]:
        raise ValueError("Baseline must identify exactly one standard agent-core service")
    core_key = cores[0]
    if core_key == DRIVER_SERVICE or set(services) != {core_key, DRIVER_SERVICE}:
        raise ValueError("Candidate must contain exactly the existing Core and one S10 driver")
    if services[core_key] != original[core_key]:
        raise ValueError("Core service changed, including its name, image or configuration")
    # Shared resources can change Core behavior even with an identical service.
    if {k: v for k, v in candidate.items() if k != "services"} != {
            k: v for k, v in baseline.items() if k != "services"}:
        raise ValueError("Compose project settings and shared resources must remain unchanged")
    driver = services[DRIVER_SERVICE]
    if driver.get("container_name") != DRIVER_NAME or driver.get("network_mode") != "host":
        raise ValueError("S10 driver requires its existing name and host network")
    if not driver.get("image") or "__IMAGE__" in driver["image"] or "${" in driver["image"]:
        raise ValueError("Resolve the driver image before validating a Compose candidate")
    if "build" in driver:
        raise ValueError("Use a previously built local driver image")
    for service in services.values():
        if service.get("scale", 1) != 1 or service.get("deploy", {}).get("replicas", 1) != 1:
            raise ValueError("Each service must have exactly one replica")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args(argv)
    if bool(args.candidate) != bool(args.baseline):
        parser.error("--candidate and --baseline must be provided together")
    validate_fragment(load(Path(__file__).with_name("service.yml")))
    if args.candidate:
        validate_compose(load(args.candidate), load(args.baseline))
        print("PASS: exactly one approved standard Core and one S10 driver in local Compose")
    else:
        print("PASS: single-driver install fragment; full Compose and running containers not checked")


if __name__ == "__main__":
    main()
