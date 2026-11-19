# Helios model and protocol provenance

`helios.urdf` is the model read from the user's manufacturer-installed iNex configuration for `Rokae_Robot_2.1_AR5_v1.1` on 2026-11-18:

```text
/opt/inex/config/robot/Rokae_Robot_2.1_AR5_v1.1/urdf/Rokae_Robot_2.1_AR5_v1.1.urdf
```

SHA-256: `490202573e9bc6bb54668440a4fcf39c4b4f5b0af71548806be76de695bf726f`.

Line endings are normalized from CRLF to LF; XML content is otherwise unchanged. Repository SHA-256: `d74a81a84af381d2a6c1f57aeb9b2910e77e7924116825c573ca2251059eb758`.

The model has 34 links and 33 joints (20 revolute, 13 fixed). Its original transforms, names, axes and limits are preserved. The current frontend renders the kinematic tree, with a Helios-specific schematic upper-body display; no STL mesh assets are included or loaded. Manufacturer model rights remain with their owners; the repository's Apache license does not establish a separate redistribution license for the manufacturer model. Model release terms remain to be confirmed before upstream acceptance.

`chassis_battery.proto` is a minimal wire-compatible projection of the installed chassis UI's status protocol: `UiDatagram.robot_base` is field 8; `RobotBase.robot_id` and `data_pool` are fields 5 and 11. Unused fields are intentionally omitted. `chassis_battery_pb2.py` is generated from this projection. No manufacturer SDK binaries are bundled.
