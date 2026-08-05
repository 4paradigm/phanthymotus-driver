"""Canonical MCP card contract shared by native and traditional navigation."""

from __future__ import annotations

from copy import deepcopy


CONTROLLED_SPATIAL_ACTIONS = (
    "start_mapping",
    "stop_mapping",
    "tag_place",
    "untag_place",
    "list_tags",
    "list_maps",
    "delete_map",
    "load_map",
    "navigate_to_tag",
    "navigate_to_pose",
    "wait_navigation_done",
    "pause_nav",
    "resume_nav",
    "stop_nav",
)

CONTROLLED_SPATIAL_ACTION_PARAMS = {
    "start_mapping": {
        "params": ["map_name"],
        "description": "Start SLAM mapping with given map name",
    },
    "stop_mapping": {
        "params": [],
        "description": "Stop mapping and save the map",
    },
    "tag_place": {
        "params": ["name", "description"],
        "description": "Tag current position with a semantic name",
    },
    "untag_place": {
        "params": ["name"],
        "description": "Remove a place tag",
    },
    "list_tags": {
        "params": [],
        "description": "List all tags in current map with relative positions",
    },
    "list_maps": {"params": [], "description": "List all saved maps"},
    "delete_map": {
        "params": ["map_name"],
        "description": "Delete a map and its associated data",
    },
    "load_map": {
        "params": ["map_name"],
        "description": "Load a map (robot must be at map origin)",
    },
    "navigate_to_tag": {
        "params": ["tag_name", "speed", "mode"],
        "description": (
            "Navigate to a tagged place (non-blocking). mode: "
            "1=stop-on-obstacle (default), 0=detour. MUST be followed by a "
            "separate wait_navigation_done call in the same turn to wait for "
            "arrival before proceeding."
        ),
    },
    "navigate_to_pose": {
        "params": ["x", "y", "yaw", "speed", "mode"],
        "description": (
            "Navigate to coordinates (non-blocking). mode: "
            "1=stop-on-obstacle (default), 0=detour. MUST be followed by a "
            "separate wait_navigation_done call in the same turn to wait for "
            "arrival before proceeding."
        ),
    },
    "wait_navigation_done": {
        "params": ["stall_timeout"],
        "description": (
            "Block until the previous navigate_to_tag or navigate_to_pose "
            "completes. Returns on arrival, timeout, or error. Always call "
            "after navigate_to_tag/navigate_to_pose."
        ),
    },
    "pause_nav": {"params": [], "description": "Pause navigation"},
    "resume_nav": {"params": [], "description": "Resume navigation"},
    "stop_nav": {"params": [], "description": "Stop and cancel navigation"},
}

_CONTROLLED_SPATIAL_TOOL = {
    "name": "controlled_spatial",
    "type": "actuator",
    "multiInstance": False,
    "description": (
        "Controlled mapping & navigation — user manually drives the robot "
        "during mapping. Supports: start/stop mapping, tag places, list/delete "
        "maps, load map, navigate between tags."
    ),
    "inputSchema": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": list(CONTROLLED_SPATIAL_ACTIONS),
                "description": "Action to perform",
            },
            "map_name": {
                "type": "string",
                "description": "Map name (for start_mapping, delete_map, load_map)",
            },
            "name": {"type": "string", "description": "POI tag name"},
            "description": {
                "type": "string",
                "description": "POI description",
            },
            "tag_name": {
                "type": "string",
                "description": "Target tag name for navigation",
            },
            "x": {
                "type": "number",
                "description": "Target X coordinate (meters)",
            },
            "y": {
                "type": "number",
                "description": "Target Y coordinate (meters)",
            },
            "yaw": {
                "type": "number",
                "description": "Target yaw (radians)",
            },
            "speed": {
                "type": "number",
                "description": "Navigation speed 0.2-0.8 m/s (default 0.5)",
            },
            "mode": {
                "type": "integer",
                "description": "Obstacle mode: 1=stop(default), 0=detour",
            },
            "stall_timeout": {
                "type": "number",
                "description": (
                    "Seconds without movement before declaring timeout (default 90)"
                ),
            },
        },
        "required": ["action"],
        "x-action-params": CONTROLLED_SPATIAL_ACTION_PARAMS,
    },
}


def controlled_spatial_tool_definition() -> dict:
    """Return an isolated copy so callers cannot mutate the shared contract."""
    return deepcopy(_CONTROLLED_SPATIAL_TOOL)
