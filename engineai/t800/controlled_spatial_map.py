import os
import json
import threading

from device import _MappingDB


class ControlledSpatialMapPlugin:
    PREFIX = "controlled_spatial_map"

    def __init__(self, plugin_config: dict, namespace: str, executor, *_, **__):
        db_path = plugin_config.get("db_path", "/opt/phanthy-motus/data/mapping.db")
        self._cloud_dir = plugin_config.get(
            "cloud_dir", "/opt/phanthy-motus/data/controlled_spatial_clouds"
        )
        self._map_topic = f"/{namespace}/controlled_spatial/map"
        self._db = None
        self._startup_error = None
        self._last_selected_map = None

        try:
            self._db = _MappingDB(db_path)
        except Exception as exc:
            self._startup_error = str(exc)
            print(f"[ControlledSpatialMap] startup degraded: {exc}", flush=True)
            try:
                import traceback
                traceback.print_exc()
            except Exception:
                pass

        print(f"[ControlledSpatialMap] plugin ready, topic: {self._map_topic}", flush=True)

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX,
            "type": "sensor",
            "multiInstance": False,
            "description": "Controlled spatial map view — saved maps, tags, live robot pose, and SLAM point cloud when available.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "refresh", "select_map", "list_maps"],
                        "description": "Optional map-view control action",
                    },
                    "map_name": {"type": "string", "description": "Map name for select_map"},
                    "overwrite": {"type": "boolean"},
                },
            },
            "topic_out": [{"topic": self._map_topic, "format": "sensor/mapping"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in (self.PREFIX, "start", "refresh"):
            return self._info("ready")
        if action == "stop":
            return self._info("idle")
        if action == "info":
            return self._info("running")
        if action == "list_maps":
            return self._list_maps()
        if action == "select_map":
            return self._select_map(args.get("map_name", ""))
        return None

    def _info(self, state: str) -> dict:
        maps = []
        active_map = None
        if self._db:
            try:
                maps = self._db.list_maps_with_pois()
                active_map = self._db.get_state("active_map")
            except Exception:
                maps = []
                active_map = None
        return {
            "state": state if not self._startup_error else "degraded",
            "topic_out": [{"topic": self._map_topic, "format": "sensor/mapping"}],
            "active_map": active_map,
            "cloud_dir": self._cloud_dir,
            "map_count": len(maps),
            "startup_error": self._startup_error,
        }

    def _list_maps(self) -> dict:
        if self._db is None:
            return self._info("degraded")
        try:
            maps = self._db.list_maps_with_pois()
            active_map = self._db.get_state("active_map")
            return {
                "maps": maps,
                "active_map": active_map,
                "cloud_dir": self._cloud_dir,
                "map_count": len(maps),
            }
        except Exception as exc:
            return {"error": f"failed to read maps: {exc}"}

    def _select_map(self, map_name: str) -> dict:
        if not map_name:
            return {"error": "map_name is required"}
        if self._db is None:
            return self._info("degraded")
        try:
            maps = self._db.list_maps_with_pois()
        except Exception as exc:
            return {"error": f"failed to read maps: {exc}"}

        map_names = [item["name"] for item in maps]
        if map_name not in map_names:
            return {"error": "Map not found", "maps": map_names}

        self._last_selected_map = map_name
        return self._info("selected")


def make_plugin(plugin_config, namespace, executor, client=None):
    return ControlledSpatialMapPlugin(plugin_config, namespace, executor)
