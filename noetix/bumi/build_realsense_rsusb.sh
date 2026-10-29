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
    curl -fsSL --proto '=https' --proto-redir '=https' --retry 2 \
        "https://codeload.github.com/realsenseai/librealsense/tar.gz/$revision" \
        -o "$root/source.tar.gz"
    echo "$digest  $root/source.tar.gz" | sha256sum -c -
    mkdir -p "$root/source"
    tar -xzf "$root/source.tar.gz" -C "$root/source" --strip-components=1
    printf '%s\n' "$digest" > "$root/source/.verified-archive-sha256"
fi
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
cmake --build "$root/build" --target pyrealsense2 pyrsutils --parallel 3
cmake --install "$root/build"
printf 'SDK ready: PYTHONPATH=%s/install/python\n' "$root"
