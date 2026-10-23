#!/bin/sh
set -eu

# Official ROS2 5.1.1 package supplies C++ headers/static libraries. We build
# only our private worker, not the vendor ROS node or its other control ports.
case "${1:-$(uname -m)}" in
  arm64|aarch64)
    package=slamware-ros2-linux-aarch64_gcc11
    checksum=3b6ffbfa3b8dcf2b21a65dcd126d1374b9854cbd6486a02d6813f72bfdadd6b4
    ;;
  amd64|x86_64)
    package=slamware-ros2-sdk_x86_64_gcc11
    checksum=49efe2c2c003398ce1e707a998aae5a12bc0212bd1f1ebe4ebbc0549e39f0465
    ;;
  *) echo "Unsupported Slamware SDK architecture: $1" >&2; exit 1 ;;
esac
sdk_build_dir=$(mktemp -d)
trap 'rm -rf "$sdk_build_dir"' EXIT
curl -fsSL --retry 3 --retry-all-errors --connect-timeout 15 --max-time 240 \
  --referer https://www.slamtec.com/en/support \
  "https://download-en.slamtec.com/api/download/$package/5.1.1-rtm?lang=netural" \
  -o "$sdk_build_dir/sdk.tar.gz"
echo "$checksum  $sdk_build_dir/sdk.tar.gz" | sha256sum -c -
mkdir "$sdk_build_dir/source"
tar -xzf "$sdk_build_dir/sdk.tar.gz" -C "$sdk_build_dir/source" --strip-components=1
sdk_root="$sdk_build_dir/source/src/slamware_sdk"
cmake -S "$(dirname "$0")" -B "$sdk_build_dir/build" \
  -DSLAMWARE_SDK_ROOT="$sdk_root" -DCMAKE_BUILD_TYPE=Release
cmake --build "$sdk_build_dir/build" --parallel 2
cmake --install "$sdk_build_dir/build" --prefix /usr/local
strip /usr/local/bin/tianyi-loco-sdk
mkdir -p /usr/share/doc/tianyi-loco-sdk
cp "$sdk_root/LICENSE" /usr/share/doc/tianyi-loco-sdk/SLAMWARE-LICENSE
# Retain bundled dependency notices together with the redistributed worker.
find "$sdk_root/include" -type f \( -iname '*license*' -o -iname '*copyright*' \) \
  -exec sh -c 'for f do printf "\n--- %s ---\n" "$f"; cat "$f"; done' sh {} + \
  > /usr/share/doc/tianyi-loco-sdk/DEPENDENCY-NOTICES
