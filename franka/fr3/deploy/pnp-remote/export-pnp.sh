#!/bin/sh
# Run inside the driver image to export its complete PNP support sources.
set -eu
exec tar -C /deploy/pnp-remote -czf - \
    read_vendor_masters.py read_vendor_ips.py detect_paths.py arm_config.py vendor_recordings.py playback.py vendor_alignment.py release_follow.py run_right.py run_vendor.py execution_status.py prepare_vendor_follow.py \
    stop_right.sh vendor-follow-env.sh vendor-bin/sudo install-pnp.sh \
    pnp-card-ctl pnp-fr3-right-teleop.service README.md
