# Helios chassis movement (`base_drive`)

## Source

Vendor VMR AMR SDK 2.1.51 (`VMR_AMR_SDK_arm_SDK-2.1.51_e22f4_py312`), module `nav`, example `examples/python/move_speed.py`:

- `nav.set_third_speed_ctrl()` binds SDK speed control; `nav.close_third_speed_ctrl()` releases it.
- `nav.send_third_expected_speed(Twist)`: `linear.x/y` in m/s, `angular.z` in rad/s, to be sent above 20 Hz. The example states that the robot stops automatically when speed commands stop arriving.
- RPC client `py_client_common.init_client` to the chassis controller. The SDK README documents port 9000; `move_speed.py` defaults to 9003. Both answer on 192.168.0.100. This driver uses 9003 as in the speed example (`chassis_sdk_port`).
- `exception.exception_state` publishes the whole-vehicle exception level (0 normal, 1 stop, 2 emergency stop) when it changes.

## Design

- Card contract follows `agilex/bunker-mini` `base_drive`: bounded `move`, `stop`, ACP completion, `x-resource: base`. `vyaw` is degrees/s on the card and converted to rad/s for the SDK.
- The vendor Python extension is ABI-specific and is loaded only in `chassis_sdk_worker.py`, started on the first `move` (not at Driver start). The worker owns the 50 Hz loop and the per-move deadline, and on every exit path sends zero five times and releases speed control. stdin EOF stops it.
- The Driver keeps one active move; a second `move` is refused until `stop`. A watchdog kills the worker if completion is not reported within duration + 3 s.
- A move is refused when movement is disabled (`chassis_motion_enabled` / `HELIOS_CHASSIS_MOTION_ENABLED=0`), the SDK is not mounted, the worker cannot connect, or the last reported exception level is not normal. A non-normal level during a move stops it.
- Limits: configurable `chassis_max_linear_m_s` (default 0.2) and `chassis_max_angular_deg_s` (default 15), capped in code at 0.5 m/s and 30°/s. `chassis_allow_lateral` (default false) exposes `vy`.

## Distribution (same as other drivers' vendor SDKs)

- Open-source SDKs are vendored as source (`unitree/*/unitree_sdk2py`). Closed vendor binaries are not committed; the Dockerfile downloads them from the project COS (`https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/...`) and verifies a pinned SHA256 — `agibot/A3` (AimDK wheel), `dji/base` (PSDK), `noetix/bumi` (SDK libs), G1's pinocchio wheel.
- Helios follows that: `ARG VMR_SDK_URL` / `ARG VMR_SDK_SHA256`, extracted to `/opt/vmr-sdk` in the image and import-checked at build time. The archive must contain `lib/` with the vendor's cp312 aarch64 `nav`, `py_client_common`, `exception` and `odometry` modules plus their `libclient_*.so` dependencies.
- Outstanding: a maintainer with COS write access must upload the SDK archive and provide its URL; the vendor's redistribution terms decide whether `public/` is acceptable. Until then the default is empty, clean bot builds still succeed, and `base_drive` reports the SDK as unavailable. No guessed URL is set as a default.

## Python 3.12 for the SDK worker

The vendor package 2.1.51 ships CPython 3.12 extension modules only; ROS Humble is CPython 3.10. The extensions need only glibc ≥ 2.17 and GLIBCXX ≥ 3.4.22 (Ubuntu 22.04 provides 2.35 / 3.4.30) and do not link libpython. The image therefore adds a pinned standalone CPython 3.12 (`python-build-standalone` 20261110, `cpython-3.12.16` aarch64 `install_only`, SHA256 `4792f817…3081`, PSF licence) at `/opt/sdk-python`, used only by `chassis_sdk_worker.py` (`chassis_sdk_python`). The worker's `PYTHONPATH` is limited to the SDK `lib/` and this repository so ROS's 3.10 packages never load into it. The archive is fetched by `build_fetch.py` from the npmmirror CDN first and the GitHub release second (`SDK_PYTHON_URL` / `SDK_PYTHON_FALLBACK_URL`), each with a socket timeout and progress output; the bot build host could not complete the GitHub download (killed after 10 min without output), and the pinned SHA256 is checked whichever source answers.

Verified on the robot on 2026-11-23 (files in /tmp, removed afterwards; no speed control enabled): the cp312 modules import under that interpreter, `init_client` succeeds on 192.168.0.100 ports 9000 and 9003, `has_nav_dev`, `has_exception_dev`, `has_odom_dev`, `has_location_dev` and `has_audio_dev` are true, and `odom_state` delivered 80 samples in 2 s (~40 Hz). `close_client` prints repeated "Operation aborted" lines, which are harmless.

## Odometry feedback

The worker subscribes to `odometry.odom_state` (x/y m, theta rad, vx/vy m/s, w rad/s) and forwards it to the Driver at ~2 Hz; `info` shows it with its age. Each move result carries the measured displacement in the start pose's body frame (`forward_m`, `left_m`, `distance_m`, `yaw_deg`), taken 0.3 s after zero speed, so a commanded move can be checked against what the base actually did.

## Chassis prerequisites

The vendor controller accepts SDK speed only in **auto mode on a loaded map with the robot localised**. Observed on the robot (2026-11-23/24):

- Auto mode, not localised: the chassis reports exception level 1 (`311/1` localisation timeout, `4/1` relocalisation failed, `2/2` brake) and brakes; `base_drive` refuses the move and reports the exception list.
- Vendor manual mode: `118/1` "manual control"; the SDK speed is accepted but ignored.
- After building a map in the vendor UI, applying it and relocalising (`loc_status` 4, weight 1.00), moves work. Idle motors (`2/7` "chassis resting") are re-enabled by the chassis automatically when a move starts.

The SDK documentation does not mention these conditions; they are vendor controller behaviour.

## Robot test (2026-11-24, local preview image, supervised on site)

Chassis on map `helioslab`, localised, auto mode, charger unplugged, area cleared, emergency stop at hand. Each move was 1 s; odometry is the move result's measured displacement:

| Command | Repeats | Measured (total) |
|---|---|---|
| `vx = 0.05` | 3 | forward 15.1 cm, lateral -0.1 cm, yaw -0.1° |
| `vx = -0.05` | 3 | backward 14.5 cm, lateral 0.0 cm, yaw -0.1° |
| `vyaw = +10` | 3 | yaw +26.0° (counter-clockwise, seen as left), translation 0.1 cm |
| `vyaw = -10` | 3 | yaw -27.3° (clockwise), translation 0.1 cm |
| `vy = 0.05` (lateral enabled) | — | moves sideways as commanded |
| `stop_move` during a rotation | — | base stops immediately |

Positive `vyaw` is counter-clockwise (left). Shortfall against the commanded angle is acceleration and deceleration within a 1 s move. The exception list was empty after every move.

This was a locally built preview image, not the bot image; the bot-image test follows the PR process.

## Still open

- The controller's own command timeout and whether obstacle avoidance applies during SDK speed control are not documented by the vendor.
- Charging interlock: the card does not yet refuse moves while charging; a reliable charging signal still has to be identified.
- Whether `exception_state` delivers the current level on subscription or only on change (an unknown level does not block a move).
