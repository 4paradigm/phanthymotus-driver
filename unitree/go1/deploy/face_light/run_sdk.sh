#!/bin/sh
# Build against the operator's mounted SDK; vendor libraries stay outside the image.
set -eu
sdk_root="${FACE_LIGHT_SDK_DIR:-/opt/phanthy-motus/data/go1/faceLightSDK_Nano}"
build_root="${FACE_LIGHT_BUILD_DIR:-/tmp/go1-face-light-sdk-runtime}"
source_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ ! -f "$sdk_root/include/FaceLightClient.h" ] || [ ! -f "$sdk_root/include/LEDPixel.h" ]; then
    printf '%s\n' 'ERROR official faceLight SDK headers are missing; mount the trusted official SDK at /opt/phanthy-motus/data/go1/faceLightSDK_Nano'
    exit 1
fi
# CMake copies the matching library beside the adapter and only rebuilds changed inputs.
if ! cmake -S "$source_root" -B "$build_root" -DFACE_LIGHT_SDK_DIR="$sdk_root" >&2 ||
   ! cmake --build "$build_root" >&2; then
    printf '%s\n' 'ERROR official faceLight SDK adapter build failed; check SDK library architecture and compiler'
    exit 1
fi
exec "$build_root/face_light_sdk_adapter"
