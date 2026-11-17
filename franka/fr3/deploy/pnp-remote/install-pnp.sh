#!/bin/sh
# Install the image's PNP support package on the machine attached to the masters.
set -eu
cd "$(dirname "$0")"
if [ "${PNP_INSTALL_STDIN_PASSWORD:-0}" = 1 ]; then
    IFS= read -r pnp_install_password
    sudo() {
        printf '%s\n' "$pnp_install_password" | command sudo -S -p '' "$@"
    }
fi
runtime="$HOME/.local/lib/fr3-pnp"
unit=pnp-fr3-right-teleop.service
vendor=${PNP_VENDOR_ROOT:-$HOME/frankateleop}
conda_base=${PNP_CONDA_BASE:-$HOME/miniconda3}
python="$conda_base/envs/polymetis/bin/python"
export CONDA_PREFIX="$conda_base/envs/polymetis"
export PATH="$CONDA_PREFIX/bin:$conda_base/bin:$PATH"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
state=$(systemctl show "$unit" -p ActiveState --value)
case "$state" in inactive|failed) ;; *) echo "Stop $unit first" >&2; exit 1 ;; esac
test -x "$python"
test -f "$vendor/teleop/experiments/run_env.py"
test -x "$vendor/polymetis/polymetis/build/run_server"
test -x "$vendor/polymetis/polymetis/build/franka_panda_client"
test -x "$vendor/polymetis/polymetis/build/franka_hand_client"
"$python" -c 'import polymetis, torch, grpc, dynamixel_sdk'
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
"$python" prepare_vendor_follow.py "$vendor/teleop/experiments/run_env.py" "$stage/right_run_env.py"
"$python" -m py_compile "$stage/right_run_env.py"
# Render the unit and narrowly scoped sudo rule for this login account.
"$python" - "$HOME" "$vendor" "$conda_base" "$stage" <<'RENDER'
import getpass
import os
from pathlib import Path
import re
import sys
home, vendor, conda, stage = map(Path, sys.argv[1:])
user = getpass.getuser()
if not re.fullmatch(r"[a-z_][a-z0-9_-]*", user):
    raise ValueError("Unsupported service user")
for path in (home, vendor, conda):
    if not re.fullmatch(r"/[A-Za-z0-9_./-]+", str(path)):
        raise ValueError("Service paths must use simple absolute paths")
unit = Path("pnp-fr3-right-teleop.service").read_text()
unit = unit.replace("User=pnp", f"User={user}").replace("/home/pnp", str(home))
unit = unit.replace(str(home / "frankateleop"), str(vendor))
unit = unit.replace(str(home / "miniconda3"), str(conda))
unit += f"Environment=PNP_VENDOR_ROOT={vendor}\nEnvironment=PNP_CONDA_BASE={conda}\n"
(stage / "pnp-fr3-right-teleop.service").write_text(unit)
(stage / "fr3-card").write_text(
    f"{user} ALL=(root) NOPASSWD: /usr/bin/systemctl start pnp-fr3-right-teleop.service, "
    "/usr/bin/systemctl stop pnp-fr3-right-teleop.service\n")
if len(sys.argv) != 5:
    raise ValueError("Invalid installer arguments")
RENDER
sudo -v
sudo visudo -cf "$stage/fr3-card"
sudo systemd-analyze verify "$stage/$unit"
install -d "$runtime/vendor-bin"
install -m644 read_vendor_masters.py read_vendor_ips.py arm_config.py run_right.py run_vendor.py execution_status.py prepare_vendor_follow.py \
    stop_right.sh vendor-follow-env.sh "$stage/right_run_env.py" "$runtime/"
install -m755 vendor-bin/sudo "$runtime/vendor-bin/"
install -m755 pnp-card-ctl "$runtime/"
sudo install -m644 "$stage/$unit" "/etc/systemd/system/$unit"
sudo install -m440 "$stage/fr3-card" /etc/sudoers.d/fr3-card
sudo systemctl daemon-reload
if [ "$state" = failed ]; then sudo systemctl reset-failed "$unit"; fi
systemctl show "$unit" -p ActiveState -p RuntimeMaxUSec -p LimitMEMLOCK
