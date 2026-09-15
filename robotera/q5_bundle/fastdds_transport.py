"""Shared Fast DDS profile selection for Q5's Domain-42 bridge processes."""

from __future__ import annotations

import os
from pathlib import Path


DEFAULT_FASTDDS_PROFILE = Path(__file__).with_name("resource") / "fastdds_udp_only.xml"
_PROFILE_ENV_VARS = (
    # Fleet service.yml files select the shared host profile with the legacy
    # spelling, so it must win over any canonical value inherited by the image.
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTDDS_DEFAULT_PROFILES_FILE",
)


def _apply_profile(profile: Path) -> str:
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    selected = str(profile)
    for name in _PROFILE_ENV_VARS:
        os.environ[name] = selected
    print(f"[q5-dds] using Fast DDS profile: {selected}", flush=True)
    return selected


def configure_bundled_fastdds_transport() -> str:
    """Force the loopback-only UDP profile used by the media bridge."""
    if not DEFAULT_FASTDDS_PROFILE.is_file():
        raise RuntimeError(f"Fast DDS profile unavailable: {DEFAULT_FASTDDS_PROFILE}")
    return _apply_profile(DEFAULT_FASTDDS_PROFILE)


def configure_fastdds_transport() -> str:
    """Select an existing deployment profile, falling back to the bundled copy."""
    configured_paths = [os.environ.get(name) for name in _PROFILE_ENV_VARS]
    profile = next((Path(path) for path in configured_paths if path and Path(path).is_file()), None)
    if profile is None:
        missing = [path for path in configured_paths if path]
        if missing:
            print(
                f"[q5-dds] configured Fast DDS profile missing: {missing}; "
                f"falling back to {DEFAULT_FASTDDS_PROFILE}",
                flush=True,
            )
        if not DEFAULT_FASTDDS_PROFILE.is_file():
            raise RuntimeError(
                f"Fast DDS profile unavailable: configured={missing}, "
                f"fallback={DEFAULT_FASTDDS_PROFILE}"
            )
        profile = DEFAULT_FASTDDS_PROFILE

    return _apply_profile(profile)
