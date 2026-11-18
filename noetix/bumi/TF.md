# Bumi visual transforms

Wire the pose output of a calibrated vision card such as ArUco to `vision_tf` (left). Its right-hand `data/json` port carries a ROS TFMessage-shaped document. Wire it to `tf_tree`, or to another consumer of TransformStamped JSON. The tree card also works on its own for robot kinematics. It publishes fixed links on `/tf_static` and live joints and targets on `/tf`. It has no right-hand port.

## Mount the camera in the robot model

The bundled 21-joint V3 model ends at the elbows and omits the D435i RGB optical centre. Measure the pose of that optical centre in a known URDF link (for a torso-mounted camera, generally `waist_yaw_link`). The mount's axes are forward, left and up. Supply its measured translation in metres and installation roll, pitch and yaw in radians in the driver configuration. The tree adds a fixed mount link and a second fixed joint with RPY `[-π/2, 0, -π/2]` to `bumi_camera_color_optical`. A target 1 m directly in front of a level camera therefore lies 1 m along mount +X. The transform from optical coordinates to mount coordinates is `[z, -x, -y]`. Leave the ArUco translation and rotation in the optical frame.

An alternative to direct measurement is to record at least four well-spread marker-centre positions, each measured both in optical coordinates and in the selected robot link's axes, with the camera rigidly mounted. Use metre units on both sides. In `pairs.json` each entry has `optical_m` and `link_m`, each containing three numbers. An independent ruler, fixture, or surveyed touch point must supply the link-side positions.

```sh
python3 calibrate_camera_mount.py pairs.json --parent waist_yaw_link --max-error-m 0.005
```

Here 5 mm is an example of a chosen residual limit. Copy the returned `camera_mount` object into `plugins.tf_tree` in the bundle's `config.yaml`, restart the driver to load the configuration and check a **separate** surveyed point. A fit residual measures consistency of the supplied pairs. It cannot detect a common error in the reference measurements.

- `camera_mount` contains `parent`, `xyz_m` and `rpy_rad`. If omitted, robot-only operation is available and connecting vision reports the missing calibration.
- To query a palm or fingertip, add its measured fixed transform to `hand_frames` using `name`, `parent` (e.g. `l_elbow_pitch_link`), `xyz_m` and `rpy_rad`. A physical tool centre requires its actual offset. Existing robot link frames can be queried directly.
- A deployment with a complete calibrated URDF can supply its file in `urdf_path` and list its existing optical link names under `optical_frames`. Set `plugins.vision_tf.bumi_optical_frame` to its RGB optical link when using the Bumi camera. Published robot frame names carry the driver namespace, as in `my_bumi/l_hand`.

## Vision inputs and boundaries

`vision_tf` accepts the following stamped, metric observations. A named parent is a camera **optical** frame. The Bumi camera declaration `noetix/bumi/camera_color` also identifies its RGB frame when a legacy image header is empty. For an external camera, supply its real optical frame ID with a calibrated attachment to the robot.

| Input | Object identity | Pose |
| --- | --- | --- |
| ArUco `markers[]` | Dictionary and integer marker ID | `translation_m` and Rodrigues `rotation_rvec_rad` |
| Identified ROS PoseStamped | `object_id` or `child_frame_id` alongside the message | `header`, `pose.position`, `pose.orientation` |
| Metric `objects[]` | Stable `id` per observation | `position_m` and `orientation_xyzw`, or a ROS-shaped `pose` in each object |

Translations are metres, ROS quaternions are xyzw, and the original ROS capture stamp is preserved. Pixel boxes, range alone, and unoriented points cannot specify a full pose. Multiple observations produce multiple edges. An empty detection produces an empty batch. Old sightings remain in tf2 history, so consumers should query at the observation time and check freshness when using the latest result.

The tree reads the Bumi driver's existing `/<namespace>/state/joints` stream using the 21 URDF joint names and radians. The standard `robot_state_publisher` handles kinematics. The camera parent must be present in the tree, and visual inputs cannot overwrite a robot link. For example, query a deployed hand link and tag 7 with the following command.

```sh
ros2 run tf2_ros tf2_echo my_bumi/l_hand my_bumi/bumi_camera_color_optical/objects/aruco_DICT_4X4_50_7
```

For acceptance, retain the canvas wiring and the tf2 reading at two arm postures while the observed marker stays fixed. Record the physical mount calibration and the target point used for the independent distance check.
