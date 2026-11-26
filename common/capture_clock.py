"""Identity of an existing host clock domain, never a precision certificate."""
from pathlib import Path
import uuid


def host_clock_reference():
    try:
        boot = str(uuid.UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
    except (OSError, ValueError):
        return None
    return dict(clock_id='host_realtime', domain_id=boot, timestamp_basis='host_realtime')
