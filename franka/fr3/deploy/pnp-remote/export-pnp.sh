#!/bin/sh
# Run inside the driver image to export its complete PNP support sources.
set -eu
exec tar -C /deploy/pnp-remote -czf - \
    detect_paths.py arm_config.py run_right.py run_vendor.py execution_status.py prepare_vendor_follow.py \
    stop_right.sh vendor-follow-env.sh vendor-bin/sudo install-pnp.sh \
    pnp-card-ctl pnp-fr3-right-teleop.service README.md
