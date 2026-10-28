# Message definitions used by the optional container

- Upstream: https://github.com/DeepRoboticsLab/deep-robotics-msg
- Revision: `a0d1a29eec5c4db5a9107595bb51e3be8122b86c`
- Archive SHA256: `1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537`
- License: BSD-3-Clause; upstream LICENSE is retained in the archive.

This is the same fixed public message archive already vendored by this repository's
M20 driver. Only interface definitions are reused. No M20 control protocol, state
codes, firmware requirements, joint mapping, or battery scaling are reused.

The tested S10 Pro host has `drdds-ros2-msgs 1.0.23`. The public archive declares
package version 1.0.0. Package versions are not evidence of wire compatibility:
before container deployment, compare the recursively used message fields/types
against the actual host interfaces. Native NOS development uses the host's
installed message package and does not require this archive.
