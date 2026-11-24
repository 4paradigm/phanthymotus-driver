VIEWS = ("front", "left", "right")


def check_frame_group(timestamps_ms: dict[str, int], max_skew_ms: int = 100) -> dict:
    missing = [view for view in VIEWS if view not in timestamps_ms]
    if missing:
        return {"status": "missing_frames", "missing_views": missing, "skew_ms": None}

    stamps = [timestamps_ms[view] for view in VIEWS]
    skew_ms = max(stamps) - min(stamps)
    return {
        "status": "ready" if skew_ms <= max_skew_ms else "out_of_sync",
        "missing_views": [],
        "skew_ms": skew_ms,
    }
