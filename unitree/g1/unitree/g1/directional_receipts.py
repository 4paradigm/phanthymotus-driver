"""Write the driver's own terminal evidence after right-arm handback."""
import json
import os
from pathlib import Path
import re
import tempfile


def save_terminal(directory, action_id, summary):
    if not re.fullmatch(r'[0-9a-f]{32}', action_id):
        raise ValueError('invalid_directional_receipt_id')
    folder = Path(directory)
    if not folder.is_absolute():
        raise ValueError('directional_receipt_directory_must_be_absolute')
    folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    if folder.resolve(strict=True) != folder:
        raise ValueError('invalid_directional_receipt_directory')
    record = {'source': 'driver_worker', 'action_id': action_id, 'status': summary['state'],
              'result': summary}
    fd, name = tempfile.mkstemp(dir=folder, prefix='.point-')
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(record, stream, ensure_ascii=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, folder / (action_id + '.json'))
        if os.name == 'posix':
            fd = os.open(folder, os.O_DIRECTORY | os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
    finally:
        if temporary.exists():
            temporary.unlink()
