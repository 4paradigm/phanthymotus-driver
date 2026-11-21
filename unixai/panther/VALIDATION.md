# Panther spatial/audio validation — 2026-11-21

Branch: `feat/add_phanther_spatial`, based on main `25c56b0f`.
Repository edits are confined to `unixai/panther`.

## Audio, real hardware

An independent temporary container used the existing driver image with the new
`audio.py`, host PulseAudio socket/cookie mounts, and a private domain-42 topic.
It did not replace or restart the production driver container.

- Mic: AISpeech Mic Arry 8 USB source, S16LE mono 16000 Hz.
- 87 valid packets, 2048 bytes each, 178176 bytes / 5.568 seconds received.
- Ambient microphone signal RMS 182.04, peak 1252; zero malformed packets.
- Speaker: C-Media USB sink, 16 test packets, zero dropped packets, queue drained.
- Sink-monitor recording detected the quiet 440 Hz playback: peak 963.
- No room audio recording was saved; only signal statistics were retained.

Probe: `tests/probe_audio.py`. Physical audibility and ASR/TTS conversation
quality still need an operator's listening check.

## Mapping, real hardware, no driving

Only `/lidar_front/scan` produced actual scans. The vendor dual-lidar merger
reported `rear_missing` and published no `/scan` samples. The new card's explicit
single-lidar relay restored the vendor SLAM input without changing sensor frame,
timestamps or any existing card implementation.

The final code completed the stationary workflow:
`start_mapping -> tag_place -> stop_mapping -> load_map -> localized`.
The final map was 216 x 123 cells at 0.05 m/cell. The `stationary_start` tag,
native map artifacts, occupancy snapshot and SQLite records were saved and
reloaded. Pose came from the fresh `map -> base_link` TF chain; the vendor
`/pose` topic did not provide continuous stationary updates.

Two stationary test maps remain on the robot:
`validation_20261121_184417` and `validation_20261121_184846`.
Native files are under `/home/unix_ai/map_dir`; isolated test archives/SQLite
are under `/tmp/panther-spatial-validation`. These are interface-validation
maps of the stationary surroundings, not a commissioned navigation map.
Deployment defaults use `/opt/phanthy-motus/data/maps` and
`/opt/phanthy-motus/data/controlled_spatial.db`, matching Tianyi/G1's mount.

The tentative rear-port udev rule and device alias were removed after hardware
clarification, and the temporary rear-driver recovery process was stopped.
The driver does not install a rear-device rule or infer lidar hardware from
USB serial interface numbers.

Probe: `tests/probe_spatial.py`. It never sends a velocity or navigation goal.

## Navigation lifecycle, isolated ROS integration

`tests/probe_navigation.py` ran against a private fake Nav2 action server on
ROS domain 91, action `/panther_validation/navigate_to_pose`.
All five cases passed: succeeded, aborted, rejected, cancelled, timed out.
Each emitted exactly one completion callback with its matching action ID.
No goal was sent to the physical robot's Nav2 server.

Actual driving, obstacle avoidance, arrival accuracy and interruption during
physical movement remain to be verified on a complete map with an operator.

## Local checks

- Existing Panther regression suite: 48 passed.
- New focused tests: 19 passed (store/schema compatibility, exact tag lookup,
  stale pose/input rejection, map isolation, cancellation/timeout ownership,
  single-lidar relay, occupancy geometry and sensor/mapping binary payload).
- All 16 pre-existing MCP card declarations are identical to main.
- Python sources parse, YAML parses, `git diff --check` passes.
- Production image has not been rebuilt or deployed; Dockerfile includes the
  added message services and Nav2/TF runtime dependencies for the next build.
