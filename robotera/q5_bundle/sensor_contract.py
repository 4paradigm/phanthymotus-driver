"""Shared MCP declarations for Q5 read-only sensor cards."""

from __future__ import annotations


def topic_out(topic: str, data_format: str) -> list[dict[str, str]]:
    """Describe the Agent Core topic independently of the Q5 DDS publisher.

    In the single-container deployment the Domain 42 bridge is the publisher
    of this topic.  The Domain 211 card publisher is an optional local output
    and must not cause the MCP catalog to hide a readable sensor when it is
    unavailable.
    """
    return [{"topic": topic, "format": data_format}]
