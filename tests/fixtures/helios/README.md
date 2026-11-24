# Helios protocol test data

`battery_records.json` contains only the two battery records from the installed chassis UI (2026-11-18), with all unrelated chassis state removed. Tests generate the minimal Protobuf envelope and an unknown field in memory. No binary capture is committed.

`joint_scope.json` retains the joint/scope feedback shape for the installed runtime. Fixed fixtures are for tests only; production never reads them or falls back to them as live telemetry.

Model tests construct original abstract geometry in Python; no manufacturer model or manufacturer-derived transform fixture is included.
