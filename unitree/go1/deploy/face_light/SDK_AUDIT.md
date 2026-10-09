# Go1 face light SDK audit — SDK v1.0.1

This record identifies the inspected SDK by its version and the file hashes below.
It records historical inspection observations, not current process state or a
planned audit. Recheck the installation and processes on each target before takeover.

Read-only retrieval from the user's Go1: main controller (pi,
internal eth0 192.168.123.161), head Nano 192.168.123.13 (unitree, aarch64).
Actual SDK directory: `/home/unitree/Unitree/sdk/faceLightSDK_Nano`.
Version file: `v1.0.1: first UDP version`.
No SDK executable or light command was run on the robot. No process was stopped.

Files copied into the local ignored `.local/face-light-sdk/faceLightSDK_Nano/`.
These SHA-256 values matched both source and local copy:

| File | SHA-256 |
|---|---|
| include/FaceLightClient.h | 8a90cf493e1eab1a8671f3acbf4943d0db5498da7f5175817a7f0fc28bbdeb42 |
| include/LEDPixel.h | 3e31e73a7b473523a07e4067a354a188a2d46039831c1117cb11e9c7d885d92e |
| lib/libfaceLight_SDK_arm64.so | ffb695dbf82c48a297c63ed9f7fdc1ab959c3c2a332f43f0e122a907792728eb |
| lib/libfaceLight_SDK_amd64.so | 68f23eb4eec631252a9119ea3e310502f33df6d6d7ea22c01316b276ac11ba79 |
| main.cpp | 032b6882f24ceee506b58fea168c3ce16416bca4465841c997f8bd3e901be798 |
| version.txt | 18a4ef80a02626e75744e938be7030765543df34659829a282153e4ab431c25e |

Verified header: public constructor/destructor, `void setLedColor(uint32_t,
const uint8_t*)`, `void setAllLed(const uint8_t*)`, `void sendCmd()`; predefined
colors are RGB arrays. LEDPixel header declares SDK-internal RGB-to-GRB mapping.
Official sample sets colors then calls sendCmd. No direct packet implementation
is required by the card.

Read-only symbol/disassembly inspection of ARM64 lib verifies that sendCmd calls
sendto through the PLT, stores its result privately, reports failure with perror,
and returns void. The destructor calls close; process isolation ensures any SDK
allocation is also reclaimed on exit. No private member access in the adapter.

Observed processes: faceLightServer (root, PID 8218) and faceLightMqtt (unitree,
PID 8236). The latter's cwd is `/home/unitree/Unitree/autostart/faceLightMqtt`,
version file `1.0.0`; its TCP connection was to main controller
192.168.123.161:1883. UDP listeners included 192.168.123.13:7800; this observation
is not used to invent the SDK destination. Keep the server; a future authorized
SDK takeover must stop the MQTT writer and any other clients first. PIDs are
one-time observations and must be rechecked before future actions.

Local validation: actual vendor ARM64 library linked and loaded inside a
network-disabled Linux container. Native successful path used the official header
and a test dynamic library with Unix-domain sockets; native failure test used the
actual vendor library with no external network. No physical LED result claimed.
The head Nano is Ubuntu 18.04 (tegra 4.9.201), while local native compilation
uses the existing Ubuntu 22.04 ARM64 ros-base image. Target deployment must
recompile on the target OS or matching sysroot; local ELF loading is not proof
of Nano binary compatibility.

Vendor libraries remain private and are not committed or redistributed; license
and redistribution terms have not been established.

Target compatibility was subsequently verified by temporary compilation and card import on the Go1 main controller (Debian GCC 8.3, CMake 3.16.3, Python 3.7.3). Temporary files were removed. Original light server and MQTT writer remained running with unchanged PIDs. No light command was issued; physical and canvas acceptance remain pending the deployment image.
