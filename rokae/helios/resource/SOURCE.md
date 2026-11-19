# Runtime model and protocol sources

No manufacturer URDF, STL, collision/inertial geometry or SDK binary is included in the current source tree or image. The `model` resource makes read-only HTTP requests to the user's existing iNex installation at runtime:

1. `GET /api/robot_3d_model` on the configured state WebSocket endpoint's HTTP(S) origin.
2. `GET <urdf_url>` returned by that metadata, restricted to the same origin, with redirects disabled and bounded response sizes/timeouts.

The installed model is checked against the 20 joint names before being returned locally to Motus and cached per endpoint for the process lifetime. Failure to retrieve a model does not prevent joint or battery subscriptions from starting. Model files are not downloaded at build time or stored in this repository. This does not establish or grant a licence for the manufacturer's model.

Earlier PR commits included the installed model, with unconfirmed release terms. It has been removed from the current tree and build context in response to review; removal does not claim to erase earlier public Git history. No permission to redistribute that model is claimed.

Driver tests use an original generated abstract 20-axis chain with arbitrary dimensions; they do not contain manufacturer link transforms, limits, inertial values or mesh data. The Core regression suite likewise generates original synthetic geometry.

`chassis_battery.proto` is a minimal wire-compatible projection of the installed chassis UI's status protocol: `UiDatagram.robot_base` is field 8; `RobotBase.robot_id` and `data_pool` are fields 5 and 11. Unused fields are intentionally omitted. `chassis_battery_pb2.py` is generated from this projection. Battery records are stored as inspectable JSON test values, and the Protobuf envelope is generated during tests.
