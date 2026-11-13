"""The standalone Driver folder is a Python module root in its Docker image."""
from pathlib import Path
import sys

DRIVER = Path(__file__).resolve().parents[1]
ROOT = DRIVER.parents[1]
for path in (str(DRIVER), str(ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)
