#!/usr/bin/env bash
# Build an isolated SDK for Jetson kernels without HID sensor/IIO support.
set -euo pipefail

root=${1:?Usage: build_realsense_rsusb.sh ABSOLUTE_BUILD_DIRECTORY}
case "$root" in /*) ;; *) echo 'Build directory must be absolute' >&2; exit 2;; esac
revision=625279aa1fe903528a818f48746ee47c1b5fefd3 # librealsense v2.59.1
digest=80ea218550602a5cfc8fcee68156fc34816248a5246edddf6d15481772554fd7
mkdir -p "$root"
if [ -d "$root/source" ]; then
    if [ "$(cat "$root/source/.verified-archive-sha256" 2>/dev/null || true)" != "$digest" ]; then
        echo 'Existing source is unverified; use a fresh build directory' >&2
        exit 1
    fi
else
    # CI cancels builds after 600 seconds without output. Keep transfer progress
    # visible and bound network stalls instead of silently waiting indefinitely.
    # At CI's observed ~90 KB/s, the 33 MB archive needs over six minutes.
    printf '[rsusb] Downloading librealsense %s\n' "$revision"
    curl -fL --proto '=https' --proto-redir '=https' --retry 2 \
        --connect-timeout 15 --max-time 900 --retry-max-time 1800 \
        --speed-limit 1024 --speed-time 30 \
        "https://codeload.github.com/realsenseai/librealsense/tar.gz/$revision" \
        -o "$root/source.tar.gz"
    echo '[rsusb] Verifying source archive'
    echo "$digest  $root/source.tar.gz" | sha256sum -c -
    mkdir -p "$root/source"
    tar -xzf "$root/source.tar.gz" -C "$root/source" --strip-components=1
    printf '%s\n' "$digest" > "$root/source/.verified-archive-sha256"
fi
# librealsense's CMake unconditionally git-clones pybind11 and pybind11_json
# during configure. The CI builder can reach codeload.github.com (used above for
# the main source archive) but not the github.com git endpoint, so replace those
# clone steps with pinned tarball downloads. Hashes are for the exact tags the
# upstream templates request (pybind11 v2.13.6, pybind11_json 0.2.15).
python3 - "$root" <<'PY'
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])
downloads = {
    'CMake/pybind11-download.cmake.in': (
        'https://codeload.github.com/pybind/pybind11/tar.gz/refs/tags/v2.13.6',
        'e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20'),
    'CMake/pybind11-json-download.cmake.in': (
        'https://codeload.github.com/pybind/pybind11_json/tar.gz/refs/tags/0.2.15',
        '9a4a1494549a2db27bc4c5a16743a5c88ebc18e13c73019c56f399ae0310baf2'),
}
for relative, (url, digest) in downloads.items():
    path = root/'source'/relative
    text = path.read_text()
    text = re.sub(r'^\s*GIT_REPOSITORY .*$', f'    URL "{url}"', text, flags=re.M)
    text = re.sub(r'^\s*GIT_TAG .*$', f'    URL_HASH SHA256={digest}\n    TIMEOUT 1800',
                  text, flags=re.M)
    text = re.sub(r'^\s*GIT_CONFIG .*$', '', text, flags=re.M)
    text = re.sub(r'^\s*GIT_SHALLOW .*$', '', text, flags=re.M)
    path.write_text(text)
PY
echo '[rsusb] Configuring SDK'
cmake -S "$root/source" -B "$root/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install" \
    -DCMAKE_INSTALL_LIBDIR=lib '-DCMAKE_INSTALL_RPATH=$ORIGIN/../../lib' \
    -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON \
    -DPython_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INSTALL_DIR="$root/install/python/pyrealsense2" \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_TOOLS=OFF \
    -DBUILD_ROSBAG2=OFF -DBUILD_WITH_DDS=OFF \
    -DENABLE_STATS=OFF -DENABLE_AI_ASSISTANT=OFF -DCHECK_FOR_UPDATES=OFF
echo '[rsusb] Building SDK Python bindings'
cmake --build "$root/build" --target pyrealsense2 pyrsutils --parallel 3
echo '[rsusb] Installing SDK'
cmake --install "$root/build"
printf 'SDK ready: PYTHONPATH=%s/install/python\n' "$root"
