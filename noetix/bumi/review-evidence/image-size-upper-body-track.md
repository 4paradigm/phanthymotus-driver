# Bumi `upper_body_track` pose-dependency image size (2026-11-23)

The `upper_body_track` processor is **opt-in**: the default Bumi image does not
install its YOLO runtime, so this is a delta for images built with
`./build.sh --build-arg INSTALL_POSE_DEPS=1 noetix/bumi`, not for the default
image. The default image only gains two small Python modules and two `COPY`
lines; no torch is added to it.

## What was measured, and what was not

This record measures the **download size of the exact pinned ARM64 wheels** that
`INSTALL_POSE_DEPS=1` installs. The three packages below are pinned by
`noetix/bumi/pose-requirements.txt` (which the Dockerfile installs with
`-r`), so a rebuild resolves the same versions; sizes and SHA-256 digests were
read from the PyPI JSON API. It does **not** claim a whole-image `docker inspect`
delta: this evidence was prepared on an x86_64 host with no Docker daemon, while
Bumi runs ARM64, so an end-to-end ARM64 image build could not be performed here.
Wheel bytes are the dominant and unavoidable part of the growth, but the final
image delta also depends on layer compression, already-present transitive
packages and shared layers. The exact whole-image delta must be confirmed with
the reproduction commands below on the Bumi host or in ARM64 CI before release.

## ARM64 wheels behind the growth

Python 3.10 (`cp310`) is the interpreter in the pinned `ros-base` image, and
`aarch64` is Bumi's architecture, so these are the wheels pip resolves. The
base image already provides `numpy` and headless OpenCV; the notable new
packages are torch and torchvision.

| Package (pinned) | Wheel | Download bytes | MiB |
|---|---|---:|---:|
| `torch==2.8.0` | `torch-2.8.0-cp310-cp310-manylinux_2_28_aarch64.whl` | 102,052,793 | 97.3 |
| `torchvision==0.23.0` | `torchvision-0.23.0-cp310-cp310-manylinux_2_28_aarch64.whl` | 2,394,251 | 2.3 |
| `ultralytics==8.4.216` | `ultralytics-8.4.216-py3-none-any.whl` | 1,472,402 | 1.4 |
| **Subtotal (dominant wheels)** | | **105,919,446** | **101.0** |

`ultralytics==8.4.216` additionally pulls `matplotlib`, `pillow`, `polars`,
`psutil`, `requests`, `ultralytics-thop`, `nvidia-ml-py` and their transitive
dependencies. Those are comparatively small (tens of MiB in total) and are not
enumerated byte-for-byte here; they are included in the whole-image measurement
below.

**On hash-locking the transitive dependencies:** the three dominant wheels are
version-pinned, but pip still resolves their smaller transitives at build time,
so two builds days apart can differ in those packages. That is acceptable for
this optional card and is deliberately **not** turned into a `--require-hashes`
lock file here: a correct hash lock is architecture- and index-specific, must be
generated on ARM64 against the exact PyPI mirror the image uses (`PYPI_MIRROR`),
and would go stale the moment the mirror re-syncs. Committing an x86-generated
one would give a false sense of reproducibility and could break the pose build.
If byte-identical pose images are later required, generate the lock on the ARM64
builder with `pip-compile --generate-hashes` against that mirror and add
`--require-hashes` to the Dockerfile install; until then this file records the
residual as known.

### Is there a smaller alternative?

- **ARM64 already uses the small torch wheel.** The same `torch==2.8.0` on
  x86_64 is an 846.9 MiB wheel because it bundles CUDA/cuDNN; the `aarch64`
  wheel is 97.3 MiB precisely because those GPU runtimes are not shipped for
  ARM64. There is no smaller official CPU-only wheel to select on Bumi — the
  platform already gives us the minimal one.
- **A separate processor image was considered and rejected.** The card runs
  inside the bundle process and consumes in-container ROS topics; splitting it
  into a second image would add cross-container transport and a second release
  unit for a single optional card. The build-time switch keeps default images
  unchanged, which is the property that matters for the fleet.
- **Model weights are not baked into the image.** `yolo11n-pose.pt` is fetched
  on first use and cached, or supplied via `model_path`, so no weight bytes are
  added at build time. The download is pinned to the official Ultralytics
  `v8.3.0` release asset (6,255,593 bytes) with SHA-256
  `869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0`; the
  card checks that digest before YOLO opens either a downloaded or a
  pre-downloaded file. The digest was computed from the asset bytes retrieved
  through the GitHub releases API (the asset itself has no published digest).

## Reproduction (on an ARM64 builder or the Bumi host)

Build the two images with the same commit, one default and one pose-enabled,
then compare Docker's exact byte accounting:

```bash
./build.sh noetix/bumi
./build.sh --build-arg INSTALL_POSE_DEPS=1 noetix/bumi
docker image inspect <default-tag> --format '{{.Size}}'
docker image inspect <pose-tag>    --format '{{.Size}}'
# exact per-layer bytes (history rounds values, so use the Engine API):
curl -s --unix-socket /var/run/docker.sock \
  http://localhost/images/<pose-image-id>/history
```

Record both `Size` values and the sum of image-history layer `Size` entries, as
`review-evidence/image-size-2026-10-14.md` does for the earlier overlay. Until
that is run, the 101.0 MiB of pinned ARM64 wheels above is the defensible lower
bound for the growth, not a measured whole-image delta.

## Release gate

This PR does **not** publish a pose-enabled image: the default build does not
install the runtime and is the only image produced by the review build. Before
anyone publishes an `INSTALL_POSE_DEPS=1` image, run the two builds and the
`docker inspect` comparison above on an ARM64 builder, append the two byte
counts and the layer-sum delta to this file, and only then release. The default
image has no such gate beyond the usual review build.
