# Updating the existing S10 PR

The target is existing PR #451 in `4paradigm/phanthymotus-driver`, from
`wowwwwwwwwwwwwwwwwwww/phanthymotus-driver` branch
`feat/s10-base-posture-timed-move`. Continue updating that branch; do not create
a replacement PR. See the [release notes](RELEASE_20261030.md).

## Development setup and checks

Use Python 3.12 or later outside the runtime image:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r scripts/s10-review-requirements.txt
.venv/bin/python scripts/update_s10_pr.py --offline
```

Run the helper on the original PR branch. It validates the public catalogue,
JSON schemas, Docker COPY inputs and service isolation. The S10 test suite,
captured fixture and selected-test manifest have been removed at the
maintainer's request; the helper does not run regression tests. These checks
never connect to the robot, restart containers or send motion.

## Publishing an S10-only update

```bash
.venv/bin/python scripts/update_s10_pr.py --publish
```

The helper verifies the original fork, branch and live open PR before committing
and making a normal push. It never changes branches, merges, rebases, amends,
resets, force-pushes or creates PRs. Existing GitHub/Git credential-helper
authentication is reused without logging credentials. `GH_TOKEN` or
`GITHUB_TOKEN` may supply API authentication; do not store tokens in this repo.

Only Driver sources/docs and the named S10 release helpers are eligible. Local
databases, credentials, image archives and unrelated changes are excluded.
Pre-staged changes or unrelated unpushed commits stop publication. Integrating
upstream changes or removing files outside that allowlist requires a separate
reviewed Git update of the same branch. The helper never automatically stashes
or cleans the workspace. Concurrent remote updates are never replaced.

Updating the existing branch updates PR #451. The helper does not post bot
commands, request a separate build or deploy an image.

## Building the reviewed source

Stage exactly the Docker COPY inputs outside the checkout:

```bash
.venv/bin/python scripts/check_s10_release.py \
  --context /tmp/s10-release-context --report /tmp/s10-release-manifest.json
docker build --platform linux/arm64 -t s10-pro:review /tmp/s10-release-context
```

The context destination must not exist. The manifest records hashes of staged
inputs. Passing these checks does not prove an image build or hardware
acceptance. See the [test asset audit](TEST_ASSET_AUDIT.md) for removed coverage
and historical results.
