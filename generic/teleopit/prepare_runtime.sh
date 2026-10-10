#!/usr/bin/env bash
# Explicit operator action, never executed by Driver or card lifecycle.
set -euo pipefail
if [[ "${1:-}" != "--accept-third-party-licenses" ]]; then
  printf '%s\n' '请先阅读 README.md 的第三方许可说明，然后传入 --accept-third-party-licenses。' >&2
  exit 2
fi
shift
teleopit_runtime_dir="${TELEOPIT_RUNTIME_DIR:-/opt/teleopit-runtime}"
teleopit_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "${teleopit_runtime_dir}/venv/bin/python" ]]; then
  python3 -m venv "${teleopit_runtime_dir}/venv"
fi
"${teleopit_runtime_dir}/venv/bin/python" "${teleopit_script_dir}/setup_teleopit.py" \
  --root "${teleopit_runtime_dir}/source" --accept-third-party-licenses "$@"
