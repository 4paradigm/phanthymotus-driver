# Pinned PiPER SDK artifact and COS migration dependency

The SDK ZIP is an appropriate project COS build artifact. The current implementation does not claim it has already migrated.

## Verified artifact

- Official source: https://github.com/agilexrobotics/piper_sdk/archive/8d4dc29bd6e753066dedee235c008e90ecb2ab4a.zip
- SDK commit: 8d4dc29bd6e753066dedee235c008e90ecb2ab4a
- ZIP size: 732413 bytes
- SHA256: 89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29
- Suggested object filename: piper_sdk-8d4dc29bd6e753066dedee235c008e90ecb2ab4a.zip
- Project artifact prefix: https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/

The local archive was rechecked against the above size/hash on 2026-10-14. Upload it as the identical bytes; do not repackage the SDK or change the pinned commit during this gate fix. Its license remains in the source distribution. Do not commit the ZIP or credentials.

## Concrete outstanding dependency

No existing approved COS URL for this exact object, authenticated upload method or bucket write authorization has been supplied to this task. The available repository examples provide public download URLs for other vendors, not PiPER's ZIP or a means to upload it. The local environment has no coscli/tccli executable. Knowing a public bucket prefix does not establish that this specific object exists or grant an authenticated write path. The user previously scoped SDK COS work as separate maintenance; a verified uploaded URL has now been requested in response to review.

This is an access/artifact-publication dependency, not a claim that COS technically cannot host the ZIP. The P0 motion-gate repair can be independently reviewed while this remains outstanding. A maintainer with bucket upload access should publish the verified object and provide its public URL. No guessed or unverified URL has been installed as a build default, because that would break clean builds.

## Completing the migration

1. Upload the exact pinned ZIP through the project's authorized artifact publication workflow.
2. Download the public object and verify the SHA256 above before using it.
3. Replace the Dockerfile PIPER_SDK_URL default and requirements.txt direct URL with that verified address; keep the existing hash.
4. Retain the Dockerfile build ARG so infrastructure can choose the approved artifact; keep no-cache installation and same-layer temporary-file cleanup.
5. Request a clean Bot build and check its architecture before any robot deployment.

Current builds retain the working pinned official source and hash verification. An uploaded object can be exercised using PIPER_SDK_URL without changing robot control or adding a dependency. This review response documents a deferral; it does not mark the COS finding fixed.
