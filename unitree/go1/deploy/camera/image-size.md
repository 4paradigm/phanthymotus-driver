# Go1 ARM64 image size evidence

Measured on 2026-10-08 without contacting the robot, building an image, or
running a container. Registry access used an anonymous pull-scoped token;
no user ACCESS_TOKEN or registry credential was used.

## Immutable images inspected

Registry: `bj-warehouse.tencentcloudcr.com`.

| Image | Manifest digest | Architecture | Sum of compressed layer bytes | MiB |
|---|---|---|---:|---:|
| `phanthy-motus/ros-base:latest` | `sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f` | arm64 | 261,578,370 | 249.46 |
| `phanthy-motus/drivers/unitree/go1:release.261008.c35a493` (commit `63e0dc8`) | `sha256:ac3ba96e1d864f7f8fc81000e1a9de7d93a0adf3529904278f093299548b3db6` | arm64 | 384,384,545 | 366.58 |

The Go1 image shares the base's first 15 layer digests. Its additional compressed
layers total **122,806,175 bytes (117.12 MiB)**. This includes all Go1 packages,
Python dependencies, SDK build products and source files, not only ffmpeg.
These are compressed registry layer sizes; they are not Docker's unpacked
root filesystem size or incremental host disk consumption after shared-layer reuse.

## Base encoder and dependency evidence

The base's last package-install layer is
`sha256:9cc2e3ffcdae693488a2f80232cc670251668fe9b7b6b9c2bc2cdad2063cec26`.
Its `var/lib/dpkg/status` has no installed `ffmpeg` package. Subsequent base
layers only copy/build `audio_msgs` and adjust the entrypoint. The Go1 package
layer adds `usr/bin/ffmpeg` (276,888 bytes) and `usr/bin/ffprobe` (182,816 bytes),
and lists `ffmpeg` version `7:4.4.2-0ubuntu0.22.04.1`, architecture `arm64`.
There is no base-provided packaged ffmpeg to reuse for this implementation.

The Go1 apt layer is
`sha256:24c96fb6ae030a081723472542bd9a23ba8c50d5f99fcc129bcb90b48f8b1143`,
**119,964,712 compressed bytes (114.41 MiB)**. It installs all requested Go1 apt
packages and includes package metadata/updates, so this entire layer must not
be attributed to ffmpeg.

Traversing installed `Depends` / `Pre-Depends` for ffmpeg, subtracting packages
already in the base and dependencies of the other explicitly requested Go1 apt
packages, identifies **134 additional packages**. Their dpkg `Installed-Size`
fields sum to **265,707,520 bytes (253.40 MiB)**. This is an installed-package
footprint derived from package metadata, not an isolated compressed image delta
or an A/B build measurement. Recommendations and suggestions are excluded,
consistent with `--no-install-recommends`.

## Reproduction and decision

1. Request anonymous registry pull authorization for each repository.
2. Read the image manifests by digest and their config blobs; check
   `architecture`, sum `layers[].size`, and compare the shared layer digests.
3. Read the two package layers as gzip tar streams, without executing their
   contents. Inspect `var/lib/dpkg/status` and ffmpeg/ffprobe member entries.
4. Traverse the installed dependency graph, selecting installed alternatives;
   subtract base packages and the closure of `python3-pip`, `python3-dev`,
   `cmake`, `build-essential`, `git`, `libmsgpack-dev`, `sshpass` and
   `openssh-client`. Sum `Installed-Size` for the remainder.

ffmpeg is retained because the Go1 worker invokes its MJPEG input, libx264 H.264
encoder and MP4 muxer. Removing it from this base would make `record_video`
unavailable. The package footprint is a real cost of the current encoding
choice; this PR does not claim zero image growth. A custom minimal encoder
build is outside this scoped fix. A same-base Go1 build with/without ffmpeg
has not been run because this workstation has no Docker/Podman build runtime;
the exact compressed delta attributable solely to ffmpeg remains unmeasured.
