# Bumi image-size comparison (2026-10-14)

Measured on the Bumi's Docker Engine without changing running containers.
This compares published releases, not a controlled rebuild of two commits.

## Immutable images

Registry prefix: `bj-warehouse.tencentcloudcr.com/phanthy-motus/`.

| Role | Tag | Image / repository digest |
|---|---|---|
| Earlier release without local interface overlay | `drivers/noetix/bumi:release.261010.45256ff` | `sha256:6dd2948f59afa480f1fceb7e3cbadf8029d35f5fc36d2e9f7fc0df810195e5ce` |
| PR head `4d2381c` published typed-direction release | `drivers/noetix/bumi:release.261014.b74c902` | `sha256:18a5635f6a5421ce88fa7f0d4d97d006a349af46f796e1e70e74d61a09a35b56` |
| Locally cached `ros-base:latest` | `ros-base:latest` | `sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f` |

The newer release is byte-identical by image digest to the deployed
`release.261014.0b54183`; the intervening portability documentation did not
change the runtime image.

## Measured sizes

All values below are bytes. Keep the two Docker accounting methods separate.

| Metric | Earlier release | Newer release | Increase |
|---|---:|---:|---:|
| Docker Engine `image inspect` `Size` | 500,392,685 | 500,827,551 | 434,866 |
| Sum of Docker Engine image-history layer `Size` | 1,604,796,416 | 1,606,885,376 | 2,088,960 |

The history-layer increase is about 1.99 MiB (0.13%). The inspect-size increase
is about 0.415 MiB (0.087%). These release-to-release deltas include other
changed driver layers and must not be attributed entirely to the interface.
Neither metric is a universal measure of extra free disk space needed: shared
layers, compressed content, unpacked snapshots, and retained rollback images
affect Docker's actual storage usage. Both releases round to 2.11 GB in this
engine's `docker image ls` display.

The new ROS overlay's specific image-history entries are:

| Runtime layer | Size |
|---|---:|
| `COPY /bumi_ros_ws/install/ /bumi_ros_ws/install/` | 1,142,784 |
| Runtime `AudioChunk` / `SoundDirection` import verification | 4,096 |
| Combined | 1,146,880 (1.094 MiB) |

The earlier image has no `/bumi_ros_ws/install/` layer. Only generated runtime
interfaces are copied from the builder; its source, build and log trees are
excluded. No new compiler/build-tool installation is added to the final stage.
The overlay is necessary for an independent `SoundDirection` import and ROS
publisher; its cost is measured above, not claimed to be zero.

## Relationship to the previous floating base

The old Dockerfile defaulted to `ros-base:latest`. On this robot, the cached
`latest` resolves to the exact digest now pinned in the Dockerfile. Comparing
`RootFS.Layers` confirms that both measured driver releases start with the same
ordered base layers. Thus this measured comparison introduces no base-layer
replacement; the pin fixes the base already used by the comparison images.

This is evidence about these images and this cached tag. No historical registry
tag log was supplied, so it does not claim that the moving remote `latest`
always pointed to this digest or still does today.

## Reproduction

Use `docker image inspect` for the three exact references above and compare
their `Id`, `RepoDigests`, `Size`, and the ordered `RootFS.Layers` prefixes.
For exact layer bytes, query Docker Engine
`GET /images/<image-id>/history` through `/var/run/docker.sock`; sum each entry's
`Size` and separately select the overlay COPY and import-check entries.
The human-readable `docker history` output rounds values and is insufficient
for exact byte accounting. These operations do not start or stop a container.

## Message compatibility check

The bundled `audio_msgs/msg/SoundDirection.msg` and the main repository's
`deploy/ros-base/audio_msgs/msg/SoundDirection.msg` were compared byte for byte.
Both have SHA-256
`78c52d09d430df6e12e1200529d99f3b0f219abe0ffb340f6a847453a5e1016d`.
This source check does not replace deployed-image contract verification.
