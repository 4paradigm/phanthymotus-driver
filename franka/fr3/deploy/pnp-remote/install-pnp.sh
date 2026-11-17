#!/bin/sh
set -eu
cd "$(dirname "$0")"
runtime="$HOME/.local/lib/fr3-pnp"
unit=pnp-fr3-right-teleop.service
state=$(systemctl show "$unit" -p ActiveState --value)
case "$state" in inactive|failed) ;; *) echo "Stop $unit first" >&2; exit 1 ;; esac
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
python="$HOME/miniconda3/envs/polymetis/bin/python"
"$python" prepare_vendor_follow.py "$HOME/frankateleop/teleop/experiments/run_env.py" "$stage/right_run_env.py"
"$python" -m py_compile "$stage/right_run_env.py"
install -d "$runtime/vendor-bin"
install -m644 arm_config.py run_right.py run_vendor.py execution_status.py prepare_vendor_follow.py \
    stop_right.sh vendor-follow-env.sh "$stage/right_run_env.py" "$runtime/"
install -m755 vendor-bin/sudo "$runtime/vendor-bin/"
install -m755 pnp-card-ctl "$runtime/"
sudo install -m644 "$unit" "/etc/systemd/system/$unit"
sudo systemctl daemon-reload
if [ "$state" = failed ]; then sudo systemctl reset-failed "$unit"; fi
systemctl show "$unit" -p ActiveState -p RuntimeMaxUSec -p LimitMEMLOCK
