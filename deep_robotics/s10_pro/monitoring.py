"""Availability of the S10 device-management placeholder."""
DEVICE_ACTIONS = ("lights", "mode", "sleep", "wake", "auto_sleep", "sleep_status")
DEVICE_REASON = "This robot's S10 device protocol, version and completion feedback have not been verified. No device command is sent."


def capabilities_snapshot():
    return {"device": {"status": "unavailable", "reason": DEVICE_REASON,
        "actions": {name: {"available": False, "reason": DEVICE_REASON} for name in DEVICE_ACTIONS}}}
