"""Load teleop modules without exposing ambiguous bundle imports like device."""
import importlib.util
from pathlib import Path
import sys

for name in ("motion_stream", "teleop_executor"):
    path = Path(__file__).resolve().parents[1] / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
