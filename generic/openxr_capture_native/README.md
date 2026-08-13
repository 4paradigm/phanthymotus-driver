# PhanthyMotus Android OpenXR Capture

This Driver-owned module provides the native, menu-free Android OpenXR pose
endpoint for the generic teleoperation runtime. One shared codebase produces
separate Meta Quest and PICO APKs. Both send the public
`motus.teleop.rtc-frame.v1` contract directly to a Driver over WebRTC. The
Capture app does not own the Driver's teleoperation session, lease, fence,
operator authorization, start action, or robot-output configuration.

## Supported build targets

| Build target | Target device family | Application ID | Debug APK |
| --- | --- | --- | --- |
| `meta` | Meta Quest 3 | `com.phanthymotus.questcapture` | `app/build/outputs/apk/meta/debug/app-meta-debug.apk` |
| `pico` | PICO 4 / 4 Enterprise / 4 Ultra / Ultra Enterprise | `com.phanthymotus.picocapture` | `app/build/outputs/apk/pico/debug/app-pico-debug.apk` |

The application IDs intentionally remain compatible with the existing Meta and
PICO lab packages. An in-place upgrade also requires the same signing key; a
debug APK signed on another workstation may require uninstalling the old build,
which clears the stored Capture credential.

The PICO target uses the standard Khronos Android OpenXR loader, which PICO
supports starting with PICO OS 5.9.0. Update the headset to at least that version
before testing. See PICO's [standard-loader announcement][pico-loader] and
[Native SDK release notes][pico-native].

Use PICO OS 5.13.0 or newer as the shared acceptance baseline. At startup the
same PICO APK enables only the controller extensions actually exposed by the
runtime: `XR_BD_controller_interaction` for PICO 4-class controllers and
`XR_BD_ultra_controller_interaction` for PICO 4 Ultra-class controllers. If
neither current Khronos profile is available, the app stops instead of emitting
apparently valid input from an unknown controller.

The PICO APK has build and static validation, but has not yet completed hardware
acceptance on a PICO headset. Do not claim device support until install, OpenXR
session, controller profile, dual-squeeze deadman, tracking-loss HOLD, reconnect,
and end-to-end Driver frame checks pass on each target device/OS combination.

[pico-loader]: https://developer.picoxr.com/blog/muz6s63x/
[pico-native]: https://developer.picoxr.com/document/native/

## Operator flow

1. In the PhanthyMotus Canvas, use the Driver's `teleop_session` card to create
   a one-time Capture pairing.
2. Install the correct APK once and launch it with the pairing bootstrap from
   the PC. The pairing code is consumed once; the resulting credential is stored
   in that application's private Android preferences.
3. On subsequent days, start it from the PC with `launch_capture.sh --platform
   meta --resume` or `--platform pico --resume`. The app calls `xrBeginSession`
   automatically when the runtime enters `READY`; the wearer does not click a VR
   page or menu.
4. The PC operator alone performs session start, pairing/revocation,
   Pause/HOLD, emergency stop, and release through the card. The wearer still
   physically uses both squeeze controls as the motion deadman.

An Android or OpenXR system permission prompt may still appear on first install.
The application cannot collect controller poses while Android backgrounds it or
OpenXR removes `FOCUSED` input ownership; that transition closes RTC locally and
the Driver independently forces the teleoperation session into HOLD.

The pairing binds the Capture credential to the exact Driver
`wss://.../ws/teleop-capture` origin and deployment CA. Resume cannot override
either value. WebRTC uses an ordered `teleop-control` DataChannel for liveness
and an unordered, zero-retransmit `teleop-pose` DataChannel for current poses.
Focus loss, tracking/reference-space recenter, RTC failure, or pose
backpressure closes the local streaming path instead of continuing with stale
authority or coordinates.

The supplied RTC configuration is a direct-LAN path and does not configure
STUN/TURN. Cross-NAT or SSH-tunnel operation is not an accepted deployment
shape for this version.

## Build

Required versions are fixed in the project:

- JDK 17
- Android platform 35 and build-tools 35.0.1
- NDK 27.0.12077973
- CMake 3.22.1
- Gradle 8.9 (downloaded and SHA-256 verified by the build script)
- OpenXR Android loader 1.1.60 (AAR SHA-256 verified by Gradle)
- libdatachannel 0.24.3 and Mbed TLS 3.6.7 (immutable Git revisions)

构建脚本不会执行可预测路径里预先展开的 Gradle 程序，也不会在校验后重新打开共享
缓存文件；每次都会先把发行 ZIP 复制或下载到本次构建的私有临时目录，再校验并从
同一个文件解压执行。

After accepting the Android SDK/NDK licenses in your own SDK installation, build
both APKs (the default):

```sh
export JAVA_HOME=/path/to/jdk-17
export ANDROID_SDK_ROOT=/path/to/android-sdk
./scripts/build_android.sh
```

For a shorter incremental build, select one target explicitly:

```sh
./scripts/build_android.sh --platform meta
./scripts/build_android.sh --platform pico
```

The flavors pass `MOTUS_CAPTURE_HEADSET=meta` or
`MOTUS_CAPTURE_HEADSET=pico` to the native build. The Meta APK accepts only the
Oculus Touch profile; the PICO APK dynamically accepts the current PICO 4 and
PICO 4 Ultra profiles described above.

## Install and first pairing

Install the APK that matches the headset:

```sh
adb install -r app/build/outputs/apk/meta/debug/app-meta-debug.apk
# Or:
adb install -r app/build/outputs/apk/pico/debug/app-pico-debug.apk
```

Then launch and pair it from the PC:

```sh
export DRIVER_CAPTURE_WSS_URL='wss://driver-host:8443/ws/teleop-capture'
export PAIRING_ID='the-id-shown-on-the-PC'
export CA_CERT_BASE64='the-public-value-shown-on-the-PC'
./scripts/launch_capture.sh --platform meta
# Or: ./scripts/launch_capture.sh --platform pico
```

The script prompts without terminal echo for the one-time pairing code. Resume a
previously paired app with:

```sh
./scripts/launch_capture.sh --platform meta --resume
./scripts/launch_capture.sh --platform pico --resume
```

When multiple or wireless-ADB devices are visible, target one explicitly without
changing the command:

```sh
ADB_SERIAL='<headset-adb-ip>:5555' \
  ./scripts/launch_capture.sh --platform meta --resume
```

`CA_CERT_FILE=/path/to/public-chain.pem` may be used instead of the Base64 field.
The public PEM chain is limited to 32 KiB after decoding; its canonical Base64
form is therefore at most 43,692 characters. The launcher rejects both encoded
text and decoded PEM that exceed those bounds before invoking ADB.
If the Driver was reinstalled or the enrollment was revoked, generate a new
pairing from the card and run the first-pairing command again. An explicit new
bootstrap replaces the stored credential; clearing app data or using a headset
menu is not required. The exported NativeActivity requires the platform-only
`android.permission.DUMP` caller permission, so ordinary headset apps cannot
inject a pairing origin or replace an enrollment; the ADB shell is the intended
laboratory bootstrap caller. Meta and PICO use different application IDs, so
their credentials cannot be mixed accidentally. Confirm the ADB launch
permission on each target headset OS before treating that SKU as accepted.
The one-time code is intentionally passed as an `adb shell am start` argument;
it is absent from script stdout but can be observed by trusted ADB-host/device
process inspection during its 60-second, single-use lifetime.

No G1 connection or robot output is involved in build, pairing, or Shadow
validation.

## Host protocol and launcher tests

```sh
./tests/launch_capture_test.sh

NLOHMANN_JSON_INCLUDE=/path/to/nlohmann/include \
PHANTHYMOTUS_DRIVER_G1_ROOT=/path/to/teleop-driver/unitree/g1 \
./tests/run_host_tests.sh
```

After building both APKs, verify package identity, permissions, PICO-only
metadata, ABI, ELF dependencies, alignment and signature:

```sh
ANDROID_SDK_ROOT=/path/to/android-sdk \
JAVA_HOME=/path/to/jdk-17 \
python3 ./tests/verify_android_apks.py
```

The launcher test uses a fake ADB executable and covers exact Meta/PICO package
selection, `ADB_SERIAL`, resume, first pairing, secret-free output, CA PEM/Base64
size boundaries, and malformed arguments. The protocol checks cover
release-neutral-regrip, reconnect watermarks, terminal authentication/protocol
error classification, retryable operational errors, exact WSS envelopes,
duplicate-key rejection, one-shot SDP,
exported-Activity origin/CA pinning, and the real G1 Driver's strict RTC frame
parser without starting a publisher.
