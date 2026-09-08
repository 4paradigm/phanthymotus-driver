#!/usr/bin/env python3
"""
Transparent Bridge Integration - Monkey patch for device.py plugins.

This module provides a transparent way to route domain 42 publishers through
the socket bridge without modifying plugin code.

Usage in main.py (before creating plugins):
    import bridge_integration
    bridge_integration.enable(ros2.ctx_core)

How it works:
    1. Intercepts Node.create_publisher calls
    2. For domain 42 topics, returns BridgedPublisher instead
    3. BridgedPublisher sends to socket_bridge.py via Unix socket
    4. socket_bridge.py publishes to real domain 42 with dds-local.xml
"""

import os
from typing import Type, Optional
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile
from bridged_publisher import create_bridged_publisher, should_use_bridge


# Store original create_publisher
_original_create_publisher = None
_bridge_enabled = False
_ctx_core = None  # Store domain 42 context for comparison


def _patched_create_publisher(self, msg_type: Type, topic: str, qos: QoSProfile, *args, **kwargs):
    """Patched create_publisher that routes to bridge when appropriate."""

    # Check if topic should be bridged AND this node is on domain 42 context
    if _bridge_enabled and should_use_bridge(topic) and _ctx_core is not None:
        # Check if this node's context is the domain 42 context
        if self.context is _ctx_core:
            # Use bridged publisher for domain 42
            return create_bridged_publisher(self, msg_type, topic, qos)

    # Fall back to original create_publisher
    return _original_create_publisher(self, msg_type, topic, qos, *args, **kwargs)


def enable(ctx_core=None):
    """Enable bridge integration (monkey patch Node.create_publisher).
    
    Args:
        ctx_core: The domain 42 ROS2 context to bridge. If None, bridges all contexts.
    """
    global _original_create_publisher, _bridge_enabled, _ctx_core

    if _bridge_enabled:
        return

    # Save the domain 42 context
    _ctx_core = ctx_core

    # Save original method
    _original_create_publisher = RclpyNode.create_publisher

    # Replace with patched version
    RclpyNode.create_publisher = _patched_create_publisher

    _bridge_enabled = True
    print("[bridge-integration] enabled transparent bridge routing", flush=True)


def disable():
    """Disable bridge integration (restore original create_publisher)."""
    global _original_create_publisher, _bridge_enabled, _ctx_core

    if not _bridge_enabled:
        return

    # Restore original method
    if _original_create_publisher:
        RclpyNode.create_publisher = _original_create_publisher

    _bridge_enabled = False
    _ctx_core = None
    print("[bridge-integration] disabled bridge routing", flush=True)
