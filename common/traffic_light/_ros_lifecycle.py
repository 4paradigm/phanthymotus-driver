"""Private, retryable teardown for this processor's per-instance ROS nodes.

Only executor/node ownership is handled here. No dependency on the shared
Perception service or its source checkout is needed by the driver image.
"""
from dataclasses import dataclass
import logging

log = logging.getLogger(__name__)


@dataclass
class NodeDisposal:
    removed: bool = False
    destroyed: bool = False
    error: str | None = None

    @property
    def complete(self):
        return self.removed and self.destroyed


def dispose_node(executor, node, *, label="traffic_light", progress=None):
    """Detach before destroying; retain progress and failures for a later stop.

    rclpy returns None on success. Explicit False and membership after removal
    indicate failure; a still-registered node must never be destroyed.
    """
    result = progress if progress is not None else NodeDisposal()
    if result.complete:
        return result
    result.error = None
    if not result.removed:
        try:
            get_nodes = getattr(executor, "get_nodes", None)
            if get_nodes is None or node in get_nodes():
                if executor.remove_node(node) is False:
                    raise RuntimeError("remove_node returned False")
            if get_nodes is not None and node in get_nodes():
                raise RuntimeError("node remains registered with executor")
            result.removed = True
        except Exception as exc:
            result.error = "remove_node: " + str(exc)
            log.warning("failed to remove ROS node %r: %s", label, exc)
            return result
    try:
        if node.destroy_node() is False:
            raise RuntimeError("destroy_node returned False")
        result.destroyed = True
    except Exception as exc:
        result.error = "destroy_node: " + str(exc)
        log.warning("failed to destroy ROS node %r: %s", label, exc)
    return result
