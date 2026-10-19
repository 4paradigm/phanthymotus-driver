# PiPER ARM64 image accounting

Measured on 2026-10-16 on a native ARM64 Jetson. These are image and dependency
checks; no test container received a host device, volume, CAN interface, or host
network. The running driver and Core were not replaced.

## Immutable inputs

- Current official release: `release.261016.cde468d`, manifest
  `sha256:37a0ea3553fe8e2e14f615cc36b80823265af93651bd038a5cc990a49236d680`,
  ARM64 config ID
  `sha256:12122bd0cd2a9eb5541393df4d7f440bcabf70dce4388de5d80e779eaab578de`.
- Previous clean-build ROS input: official multiarch index
  `ros:humble-ros-base@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19`.
- New default: project ARM64 base
  `bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f`,
  config ID
  `sha256:cf6b24578bf4e9d75812923f11d0386d3d13c03df92ff0bbf4f0672be2e58238`.
- SDK source commit `8d4dc29bd6e753066dedee235c008e90ecb2ab4a`, archive SHA-256
  `89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29`.
  The download source is unchanged while COS migration is explicitly deferred;
  see [SDK_ARTIFACT.md](SDK_ARTIFACT.md).

The project base is ARM64-only. `build-local.sh` selects the pinned official
multiarch base for `linux/amd64`, preserving the existing x86 build path. Explicit
caller build arguments follow this default and can override `ROS_BASE_IMAGE`.
A raw amd64 Docker invocation must pass the official base explicitly.

## Baseline and reuse

`docker image inspect .Size` is the logical sum of uncompressed image layer
sizes. Registry transfer size below is the sum of compressed manifest layer
descriptors; it excludes manifest/config overhead and is not unique host disk
usage. Layers shared by other project drivers are reused on the host and by the
registry. MB means 1,000,000 bytes.

| Image/input | Uncompressed bytes | Registry compressed layer bytes |
| --- | ---: | ---: |
| Previous official ROS ARM64 base | 729,051,867 | 260,565,083 |
| Current release `cde468d` | 883,830,519 | 314,593,661 |
| Shared project ROS ARM64 base | 735,868,215 | 261,578,370 |

The previous release's increment over its ROS base is 154,778,652 bytes:

| Added content | Uncompressed bytes |
| --- | ---: |
| APT layer (`python3-pip` plus ROS dependency check/install) | 13,563,111 |
| Python dependencies and SDK | 140,731,768 |
| Requirements manifest | 413 |
| Common/PiPER source, model resources, and deployment files | 483,360 |

The shared project base was checked in a container with `--network none` and no
device mappings. It already provides:

| Package | Installed version |
| --- | --- |
| `python3-pip` | `22.0.2+dfsg-1ubuntu0.7` |
| `ros-humble-rmw-fastrtps-cpp` | `6.2.10-1jammy.20260416.070458` |
| `ros-humble-sensor-msgs` | `4.9.1-1jammy.20260416.041627` |

Both `rclpy` and `sensor_msgs.msg` import successfully from this base. The new
Dockerfile reuses those packages and skips APT entirely when all are installed.
The fallback only installs missing packages when an alternate base is supplied.

The Python layer remains necessary: headless OpenCV and NumPy supply live camera
frames/JPEG processing, `python-can` and the vendor SDK supply CAN protocol
support, and PyYAML loads deployment configuration. GUI OpenCV, RealSense SDKs,
CUDA/PyTorch, compilers, and extra ROS distributions are not added by PiPER. Pip
uses `--no-cache-dir`; SDK/download temporary files are removed in the same RUN.
The existing Python dependency increment is over 10 MB and is reported here for
maintainer review rather than described as a source-only growth of a few KB.

## Source overlay limits

`Dockerfile.arm64-dev` is an operator-only development shortcut. Its default now
uses the verified immutable `cde468d` manifest above, never a mutable release tag.
It copies source over the existing release and does not reinstall dependencies.
This retains old lower-layer file contents, cannot prove a clean dependency
build, and must not replace the production `Dockerfile` in release CI. Dependency
changes require a clean production build and a new reviewed base digest before
the overlay is used again.

## Reproduction

Run from the repository root:

```bash
bash agilex/piper/build-arm64.sh agilex-piper:review-arm64
bash agilex/piper/build-local.sh agilex-piper:review-amd64
docker image inspect agilex-piper:review-arm64 --format '{{.Architecture}} {{.Size}}'
docker image history --no-trunc --human=false agilex-piper:review-arm64
docker manifest inspect bj-warehouse.tencentcloudcr.com/phanthy-motus/drivers/agilex/piper@sha256:37a0ea3553fe8e2e14f615cc36b80823265af93651bd038a5cc990a49236d680
```

For source overlays, stage the PiPER directory with the repository `common/`
directory as `common/` in the build context, select `Dockerfile.arm64-dev`, and
set `PIPER_SOURCE_REV` to the source commit. Neither build script starts services
or deploys an image. Native ARM64 build and schema/import evidence is recorded
with the candidate measurement below; amd64 command selection is tested offline.

## Clean candidate measurement

The clean production Dockerfile was built natively from the pinned project base,
with freshly installed application dependencies rather than an existing PiPER
image overlay. The APT dependency check produced a zero-byte layer because the
three required packages were already present. The unchanged SDK ZIP passed its
pinned hash check during the build.

| Candidate | Uncompressed bytes | Change versus previous release |
| --- | ---: | ---: |
| Clean project-base ARM64 candidate | 877,084,337 | -6,746,182 bytes (-6.75 MB) |
| Python dependencies and SDK layer | 140,731,768 | unchanged |
| Application source/resources/deploy files | 483,941 | +581 bytes |
| Increment over shared project base (including requirements) | 141,216,122 | — |

Local candidate config ID:
`sha256:5f31d1b997f06618d588e05e5d7c92a80780d9347d9b4b0563b366fcd6b3f697`.
It is a local verification image, not a Bot Version or pushed registry manifest.
No compressed registry-transfer size is claimed for this unpublished candidate.
The heavier shared base (+6,816,348 bytes) is offset by avoiding the previous
13,563,111-byte APT layer; the net image size is smaller. The 140.73 MB Python
layer remains disclosed for review rather than claimed to be eliminated.

The digest-pinned development overlay was also built: 884,314,460 bytes, an
increment of 483,941 bytes over its cde468d base. Its source-layer growth is
intentional for operator iteration and is not a production build-size claim.

The clean candidate passed imports for rclpy, sensor_msgs, OpenCV, NumPy, PyYAML,
python-can and piper_sdk in a read-only container with `--network=none`, all
capabilities dropped, no hardware/host-network/production mounts and only a
private temporary filesystem. Constructed metadata exposed arm_control,
piper_photo, joints, model and piper_camera_live. Production source hashes were
checked against the working tree. This checks packaging, not physical motion,
DDS interoperability or live-camera acceptance. Existing services were untouched.
