# Image build inputs

Both stages use the immutable ROS base:

`ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`

Production builds target `linux/arm64`. The build stage generates `drdds`
interfaces from licensed text message definitions. Colcon/code generators are
excluded from runtime; runtime packages supply YAML, Fast DDS and ROSIDL support.
The public catalogue and Docker COPY inventory are validated by
`scripts/check_s10_release.py`; it can stage an exact context and hash manifest.

The source excludes historical TLS/adapter bindings, independent diagnostic
CLIs, policy provisioning, SQLite task/outbox implementations and process notes.
Deploy files carry operational configuration only. No credentials or site
state are included. Apt resolution is not snapshot-pinned, so identical source
and base pins do not promise byte-identical builds.

Before release, build the reviewed ARM64 source and compare candidate and deployed
baseline using the same `docker image inspect Size` measurement. Archive size
and downloaded compressed layers are different measures. Source cleanup alone
is not evidence of a completed build or hardware acceptance.
