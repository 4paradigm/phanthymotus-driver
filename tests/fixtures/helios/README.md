# Helios protocol fixtures

`robot_base_battery.bin` contains battery records captured from the installed chassis UI on 2026-11-18. All unrelated data_pool entries and unknown original Protobuf fields have been removed; robot_id is the generic value `1`. A synthetic unknown field 15 (value 1) is appended to check wire-compatible decoding. This file is a sanitized protocol fixture, not an untouched complete robot capture.

SHA-256: `6fcbf94ac564968300b0cbafd045ae4650510f7618eadd3ef99e34eeafa0ff44` (569 bytes).

`joint_scope.json` retains the joint/scope feedback shape for the installed runtime. Fixed fixtures are for tests only; production never reads them or falls back to them as live telemetry.
