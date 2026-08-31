# G1 LiDAR–Camera calibration

`g1_factory_nominal_lidar_camera.yaml` is derived from the official Unitree G1
URDF at commit `9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90`. The URDF defines both
`mid360_link` and `d435_link` relative to `torso_link`; the stored transform is
the composed `camera_color_optical_frame <- livox_frame` matrix.

This file is deliberately marked `factory_nominal`. It is suitable for an
initial projection overlay, but it is not a measured calibration for a
particular robot. Before changing the status to `validated_on_device`, record:

- the G1 and RealSense serial/profile;
- the exact transform and resulting `calibration_id`;
- RGB frames with projected LiDAR points at several distances and directions;
- quantitative pixel residuals and the acceptance threshold used.

If the assumptions in the YAML do not match the deployed frame convention,
leave the calibration unavailable or replace it with a measured transform.
Never relabel a manually adjusted matrix as the Unitree factory nominal value.

The navigation sensor bridge also publishes the factory-nominal static transform
`base_link -> livox_frame`. It composes the zero-pose URDF waist joints from
`pelvis` to `torso_link` with `mid360_joint`, matching the `base_link` convention
used by the navigation card in PhanthyMotus PR 141. The published rotation also
composes the inverse of `sensor_rotation_matrix`, because cloud and IMU samples
are expressed in that corrected output frame. It uses reliable,
transient-local `/tf_static` delivery through ROS 2 `StaticTransformBroadcaster`.
Replace the configured translation and rotation after per-device calibration;
do not describe the factory value as a measured extrinsic.
