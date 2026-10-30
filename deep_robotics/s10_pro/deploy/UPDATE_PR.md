# Updating the existing S10 PR

The target is PR #451 in `4paradigm/phanthymotus-driver`, from the original
`wowwwwwwwwwwwwwwwwwww/phanthymotus-driver` fork and
`feat/s10-base-posture-timed-move` branch. The updater does not create a new PR,
change branches, merge, rebase, amend, reset or force-push.

## Development setup

Use Python 3.12 or later in a development environment, outside the runtime image:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r scripts/s10-review-requirements.txt
```

Review the intended changes and use the check-only command first:

```bash
.venv/bin/python scripts/update_s10_pr.py
```

It checks the original fork and branch against the live PR ref, validates the
public catalogue, JSON schemas and Docker COPY inputs, checks service isolation,
and runs the release gate listed in `scripts/s10_pr_tests.txt`.
`--offline` runs the local checks without querying GitHub and cannot publish.
The script never opens a robot connection, restarts a container or sends motion.

## One-command update

```bash
.venv/bin/python scripts/update_s10_pr.py --publish
```

This explicitly commits and pushes the S10 changes after the checks pass. Before
committing, the updater also verifies that GitHub reports the PR open and on the
expected original branch/repository. Existing `gh` or Git credential-helper
authentication is reused without logging credentials. `GH_TOKEN` or
`GITHUB_TOKEN` can also supply API authentication; never put tokens in this repo.
Normal Git credentials must permit pushing the fork. API rate limits or missing
authentication stop publication rather than guessing the target.

Only Driver sources/docs, `tests/test_s10_*.py`, and the named S10 release helper
files are eligible. Local databases, credentials, image archives and unrelated
changes are excluded. Pre-staged changes or unrelated unpushed commits stop the
updater. There is no automatic stash or cleanup. A failed push leaves the local
commit available to inspect and retry; concurrent remote updates are never
replaced. Updating the branch updates the same PR. This helper does not post bot
commands, trigger a separate build request, or deploy an image.

## Building the reviewed source

Stage exactly the Docker COPY inputs outside the checkout:

```bash
.venv/bin/python scripts/check_s10_release.py \
  --context /tmp/s10-release-context --report /tmp/s10-release-manifest.json
docker build --platform linux/arm64 -t s10-pro:review /tmp/s10-release-context
```

The context destination must not exist. The manifest records hashes of the
staged inputs. The Dockerfile retains its pinned ROS base, message-generation
build stage, runtime dependencies and existing deployment hardening. Passing a
COPY/schema check is not proof of a complete image build or hardware acceptance.

## Validation limits

The release gate covers the current public API, merged actions, bounded and
relative movement, authorization/Prepare, stopping, DDS, durable tasks,
packaging and updater failure boundaries using synthetic feedback.
Historical tests that expect the retired 13-monitor catalogue or manual Prepare
UI are not part of this gate. The earlier full S10 run had 41 failures also
present at PR head; they must be migrated separately, not reported as passing.
The gate is explicitly a selected regression set, not the whole repository suite.
The `loco_move` interface exposes forward/backward and in-place turns; no lateral
entry was added. Stop diagnostics record missing evidence without relaxing
confirmation thresholds or clearing protection.
