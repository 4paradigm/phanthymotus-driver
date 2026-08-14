#!/usr/bin/env python3
"""Start the T800 driver with the optional voice-gesture extension enabled.

This is an alternate entrypoint.  It subclasses the existing bundle at
runtime, so the original ``main.py`` and device plugins remain untouched.
"""

from __future__ import annotations

import main as driver_main

from voice_gesture import VoiceGesturePlugin


class VoiceGestureBundle(driver_main.T800DeviceBundle):
    """Append VoiceGesturePlugin after the regular T800 driver plugins."""

    def __init__(self, config: dict, namespace: str, ros2):
        super().__init__(config, namespace, ros2)
        plugin_config = config.get("plugins", {}).get("voice_gesture", {}) or {}
        if not plugin_config.get("enabled", False):
            return
        targets = {}
        for plugin in self._plugins:
            definitions = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for definition in definitions:
                name = definition.get("name")
                if name in ("gesture", "motion_mode"):
                    targets[name] = plugin
        self._plugins.append(VoiceGesturePlugin(config, namespace, ros2, targets))


def main() -> None:
    driver_main.T800DeviceBundle = VoiceGestureBundle
    driver_main.main()


if __name__ == "__main__":
    main()
