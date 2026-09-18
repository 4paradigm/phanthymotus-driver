#!/usr/bin/env python3
"""
Transparent Bridge Integration - Monkey patch for device.py plugins.

This module provides a transparent way to route domain 42 publishers through
the socket bridge without modifying plugin code.

Usage in main.py (before creating plugins):
    import bridge_integration
    bridge_integration.enable(ros2.ctx_core)

How it works, outbound:
    1. Intercepts Node.create_publisher calls
    2. For domain 42 topics, returns BridgedPublisher instead
    3. BridgedPublisher sends to socket_bridge.py via Unix socket
    4. socket_bridge.py publishes to real domain 42 with dds-local.xml

And inbound, which is the same path in reverse:
    1. Intercepts Node.create_subscription calls
    2. For the domain 42 context, returns BridgedSubscription instead
    3. socket_bridge.py subscribes on real domain 42 and writes each message
       back down the same Unix socket
    4. BridgedSubscription deserializes and invokes the caller's callback

Inbound existed nowhere until a card needed to *receive* from agent-core rather
than report to it. `servo` was the first — it takes a command stream — and it
started cleanly, reported running, and never heard anything, because its
subscription went onto the main process's domain-42 participant, which runs
under the vendor DDS profile and is invisible to agent-core. Every other card
publishes outward only, which is why nothing had exercised this before.
"""

import os
from typing import Type, Optional
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile
from bridged_publisher import (create_bridged_publisher,
                               create_bridged_subscription,
                               should_bridge_subscription, should_use_bridge)


# Store originals
_original_create_publisher = None
_original_create_subscription = None
_original_destroy_node = None
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


def _patched_create_subscription(self, msg_type: Type, topic: str, callback,
                                 qos: QoSProfile, *args, **kwargs):
    """Patched create_subscription that routes domain-42 readers to the bridge.

    Unconditional for the domain-42 context, unlike the publisher side — see
    `should_bridge_subscription`. A subscription created directly on that
    context cannot receive anything, so there is no "direct" fallback to prefer.
    """
    if _bridge_enabled and _ctx_core is not None and self.context is _ctx_core:
        if should_bridge_subscription(topic):
            return create_bridged_subscription(self, msg_type, topic, callback, qos)

    return _original_create_subscription(self, msg_type, topic, callback, qos,
                                         *args, **kwargs)


def _patched_destroy_node(self):
    """Close any bridged subscriptions before the node goes.

    rclpy does not know these objects exist, so without this a card's stop
    leaves the reader thread and its socket alive, still delivering into a
    callback whose plugin believes it was torn down. The canvas does
    start/stop/start within seconds, so this is the normal path, not the edge.
    """
    for subscription in self.__dict__.pop("_bridged_subscriptions", []):
        try:
            subscription.destroy()
        except Exception as e:      # noqa: BLE001 — teardown is best effort
            print(f"[bridge-integration] subscription cleanup failed: {e}",
                  flush=True)
    return _original_destroy_node(self)


def enable(ctx_core=None):
    """Enable bridge integration (monkey patch Node publisher/subscription).

    Args:
        ctx_core: The domain 42 ROS2 context to bridge. If None, bridges all contexts.
    """
    global _original_create_publisher, _original_create_subscription
    global _original_destroy_node, _bridge_enabled, _ctx_core

    if _bridge_enabled:
        return

    # Save the domain 42 context
    _ctx_core = ctx_core

    # Save original methods
    _original_create_publisher = RclpyNode.create_publisher
    _original_create_subscription = RclpyNode.create_subscription
    _original_destroy_node = RclpyNode.destroy_node

    # Replace with patched versions
    RclpyNode.create_publisher = _patched_create_publisher
    RclpyNode.create_subscription = _patched_create_subscription
    RclpyNode.destroy_node = _patched_destroy_node

    _bridge_enabled = True
    print("[bridge-integration] enabled transparent bridge routing (both directions)",
          flush=True)


def disable():
    """Disable bridge integration (restore the original methods)."""
    global _original_create_publisher, _original_create_subscription
    global _original_destroy_node, _bridge_enabled, _ctx_core

    if not _bridge_enabled:
        return

    # Restore original methods
    if _original_create_publisher:
        RclpyNode.create_publisher = _original_create_publisher
    if _original_create_subscription:
        RclpyNode.create_subscription = _original_create_subscription
    if _original_destroy_node:
        RclpyNode.destroy_node = _original_destroy_node

    _bridge_enabled = False
    _ctx_core = None
    print("[bridge-integration] disabled bridge routing", flush=True)
