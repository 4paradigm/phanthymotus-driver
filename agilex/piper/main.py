#!/usr/bin/env python3
"""Run from repository root: PYTHONPATH=. python3 agilex/piper/main.py."""
from common.vendor_runtime import run_driver
from agilex.piper.device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "agilex-piper-driver", "agilex-piper", build_plugins)
