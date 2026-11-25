#!/usr/bin/env bash
# Read-only Bumi/Jetson deployment inventory for Food Segmentation planning.
# This script installs nothing, changes no power mode, restarts no service and
# writes no files. Run on the robot host; redirect stdout yourself if desired.
# The output intentionally omits the hostname and network configuration. It can
# still contain a Jetson SKU, installed package versions, local library paths
# and model filenames; review/redact those before sharing outside the team.

set +e

section() {
  printf '\n===== %s =====\n' "$1"
}

run_if_present() {
  command_name="$1"
  shift
  if command -v "$command_name" >/dev/null 2>&1; then
    "$command_name" "$@" 2>&1
  else
    printf '%s: NOT FOUND\n' "$command_name"
  fi
}

read_if_present() {
  for path in "$@"; do
    if [ -r "$path" ]; then
      printf '%s: ' "$path"
      tr -d '\000' < "$path"
      printf '\n'
    else
      printf '%s: NOT READABLE\n' "$path"
    fi
  done
}

section "timestamp and kernel"
run_if_present date --iso-8601=seconds
run_if_present uname -srm
run_if_present uname -m
read_if_present /etc/os-release /etc/nv_tegra_release

section "Jetson identity"
read_if_present \
  /proc/device-tree/model \
  /proc/device-tree/compatible \
  /proc/device-tree/nvidia,sku \
  /sys/firmware/devicetree/base/model
if command -v dpkg-query >/dev/null 2>&1; then
  dpkg-query -W -f='${Package}\t${Version}\n' 'nvidia-l4t-*' 2>&1
else
  echo "dpkg-query: NOT FOUND"
fi

section "power and GPU observation"
run_if_present nvpmodel -q
run_if_present nvidia-smi
if command -v tegrastats >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  timeout 3s tegrastats --interval 1000 2>&1
else
  echo "tegrastats or timeout: NOT FOUND"
fi

section "CUDA"
run_if_present nvcc --version
read_if_present /usr/local/cuda/version.json /usr/local/cuda/version.txt
printf 'CUDA_HOME=%s\n' "${CUDA_HOME:-<unset>}"
printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-<unset>}"

section "TensorRT"
if command -v dpkg-query >/dev/null 2>&1; then
  dpkg-query -W -f='${Package}\t${Version}\n' \
    'tensorrt*' 'libnvinfer*' 'python3-libnvinfer*' 2>&1
fi

section "Python inference packages"
run_if_present python3 --version
run_if_present python3 -m pip --version
if command -v python3 >/dev/null 2>&1; then
python3 - <<'PY'
import importlib
import platform

print("platform:", platform.platform())
for name in ("numpy", "cv2", "torch", "torchvision", "tensorrt", "onnxruntime", "ultralytics"):
    try:
        module = importlib.import_module(name)
        print(f"{name}: version={getattr(module, '__version__', '<unknown>')} path={getattr(module, '__file__', '<builtin>')}")
        if name == "torch":
            print("torch.cuda.is_available:", module.cuda.is_available())
            print("torch.version.cuda:", module.version.cuda)
            if module.cuda.is_available():
                print("torch.cuda.device:", module.cuda.get_device_name(0))
    except Exception as exc:
        print(f"{name}: UNAVAILABLE ({type(exc).__name__}: {exc})")
PY
fi

section "memory, cgroup and disk"
run_if_present free -h
read_if_present \
  /sys/fs/cgroup/memory.max \
  /sys/fs/cgroup/memory.current \
  /sys/fs/cgroup/memory/memory.limit_in_bytes \
  /sys/fs/cgroup/memory/memory.usage_in_bytes
run_if_present df -h / /opt/phanthy-motus/data

section "Docker GPU runtime"
run_if_present docker version
if command -v docker >/dev/null 2>&1; then
  docker info --format 'DefaultRuntime={{.DefaultRuntime}} Runtimes={{json .Runtimes}}' 2>&1
  docker info --format 'Architecture={{.Architecture}} Kernel={{.KernelVersion}} CgroupDriver={{.CgroupDriver}} CgroupVersion={{.CgroupVersion}}' 2>&1
fi
run_if_present nvidia-container-cli --version

section "persistent model mount inventory"
if [ -d /opt/phanthy-motus/data ]; then
  find /opt/phanthy-motus/data -maxdepth 4 -type f \
    \( -name '*.pt' -o -name '*.onnx' -o -name '*.engine' \) \
    -printf '%p\t%s bytes\n' 2>&1
else
  echo "/opt/phanthy-motus/data: NOT FOUND"
fi

section "probe complete"
echo "No state-changing command was requested by this script."
