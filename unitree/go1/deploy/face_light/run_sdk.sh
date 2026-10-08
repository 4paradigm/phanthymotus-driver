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
case "$(uname -m)" in
    aarch64|arm64) sdk_library=lib/libfaceLight_SDK_arm64.so
        sdk_hash=ffb695dbf82c48a297c63ed9f7fdc1ab959c3c2a332f43f0e122a907792728eb ;;
    x86_64|AMD64) sdk_library=lib/libfaceLight_SDK_amd64.so
        sdk_hash=68f23eb4eec631252a9119ea3e310502f33df6d6d7ea22c01316b276ac11ba79 ;;
    *) printf '%s\n' 'ERROR unsupported official faceLight SDK architecture'; exit 1 ;;
esac
# Pin every vendor input used by the build to the audited v1.0.1 SDK.
# A new SDK version requires a fresh audit and an explicit source change.
if ! (cd "$sdk_root" && sha256sum -c - >&2 <<EOF
8a90cf493e1eab1a8671f3acbf4943d0db5498da7f5175817a7f0fc28bbdeb42  include/FaceLightClient.h
3e31e73a7b473523a07e4067a354a188a2d46039831c1117cb11e9c7d885d92e  include/LEDPixel.h
18a4ef80a02626e75744e938be7030765543df34659829a282153e4ab431c25e  version.txt
$sdk_hash  $sdk_library
EOF
); then
    printf '%s\n' 'ERROR official faceLight SDK checksum verification failed; use the audited v1.0.1 SDK'
    exit 1
fi
# CMake copies the matching library beside the adapter and only rebuilds changed inputs.
if ! cmake -S "$source_root" -B "$build_root" -DFACE_LIGHT_SDK_DIR="$sdk_root" >&2 ||
   ! cmake --build "$build_root" >&2; then
    printf '%s\n' 'ERROR official faceLight SDK adapter build failed; check SDK library architecture and compiler'
    exit 1
fi
exec "$build_root/face_light_sdk_adapter"
