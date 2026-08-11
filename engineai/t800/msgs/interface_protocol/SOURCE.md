# Upstream source

- Repository: `engineai-robotics/engineai_ros2_workspace`
- Branch: `community`
- Commit: `ebd638e31709a038d3208517693d33174dbacb46`
- Path: `src/interface_protocol`
- License: BSD-3-Clause

Native SDK compatibility additions:

- Repository: `engineai-robotics/engineai_robotics_native_sdk`
- Commit: `83204a459e0e786f855235a8507197496a79acc7`
- Files: `NodeControl.msg`, `DynamicVectorDouble.msg`, `LinkInfo.msg`,
  `JointMotionPlanRequest.srv`

Message definitions are vendored so the driver image remains reproducible and
does not download moving upstream content during a hardware deployment build.
