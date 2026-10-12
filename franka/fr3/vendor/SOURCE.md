# libfranka source archives

The Docker build downloads these unmodified official source archives from the
project's Tencent COS bucket and verifies their SHA-256 before extraction.
Archives are not committed to Git. The Dockerfile's `LIBFRANKA_URL` and
`LIBFRANKA_COMMON_URL` arguments can select another mirror of the same bytes.

## libfranka 0.18.1

- Commit: `182c9c4f7ffb512d4b2e2efbc86643e751389771` (release tag `0.18.1`).
- Archive: `libfranka-0.18.1.tar.gz`.
- Mirror: <https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/franka/libfranka-0.18.1.tar.gz>.
- Source: <https://codeload.github.com/frankarobotics/libfranka/tar.gz/182c9c4f7ffb512d4b2e2efbc86643e751389771>.
- SHA-256: `b682dbb2c6c564608b7b3ec443f332545dde94dee6969237f182441f21c23951`.

## libfranka-common

- Commit: `d96c221a32c343384cbe2f77c61116ed58b84231`, the `common` submodule
  pinned by the libfranka commit above.
- Archive: `libfranka-0.18.1-common.tar.gz` (named for its parent SDK release).
- Mirror: <https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/franka/libfranka-0.18.1-common.tar.gz>.
- Source: <https://codeload.github.com/frankarobotics/libfranka-common/tar.gz/d96c221a32c343384cbe2f77c61116ed58b84231>.
- SHA-256: `75f26fc8794e4ae149086b59dda9de9d61cba845c375a51d13dd886ba7a9c104`.

The submodule archive is extracted into the SDK's `common/` directory because
GitHub's parent-repository archive does not contain submodule files.

## Build adaptation and licenses

The archives retain their original contents, including each project's
Apache-2.0 `LICENSE` and `NOTICE`. Both projects' license and notice files are
also included in the runtime image.

After extraction, `cmake/libfranka-offline-version.patch` replaces the SDK's
Git-based package-version lookup with its declared release version. The upstream
lookup requires `.git` even when tests and examples are disabled. This marked
build-only modification changes CPack metadata generation; it does not change
SDK C++ code, the library version, or robot runtime behavior.

When updating the SDK, update both pinned commits, obtain fresh official
archives, verify the parent repository's submodule reference, publish them to
COS, and update the URLs and checksums in this file and the Dockerfile. Revalidate
the patch, ARM64 build, runtime dependencies and robot firmware compatibility
together.
