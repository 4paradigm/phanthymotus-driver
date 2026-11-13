# Bumi fixed-point patrol baseline

`patrol_baseline` gives Agent Core an operator-defined inspection baseline and
turns the agent's assessment into an **unsent report draft**. It checks evidence
metadata and chooses the configured review/work-order route. Agent Core and the
capture chain remain responsible for interpreting the image and establishing
its provenance. The card does not capture images, navigate, command hardware,
write reports to disk, or send notifications.

## Enable the optional card

The plugin is disabled in the shipped `config.yaml`. For an approved test image,
enable it in the driver configuration:

```yaml
plugins:
  patrol_baseline:
    enabled: true
    baseline_path: examples/patrol-baseline-bumi-fixed.json
```

Relative paths resolve beside `patrol_baseline.py`, including in the container
(`/work`). An operator may supply an absolute path to a JSON profile already
mounted in the driver container. The profile is read and validated once at
startup; restart with a new revision to change it. The MCP call interface cannot
select a path or change the baseline. Invalid or missing profiles fail loading
instead of silently substituting example observations.

`examples/patrol-baseline-bumi-fixed.json` is a **rehearsal template** with one
point (`fixed-passage`), a fixed view (`bumi-front-fixed`), and a 90-second evidence
window. These identifiers describe an operator-confirmed stationary test, not
registered navigation coordinates. The normal state and camera coverage must
be confirmed on site. `examples/patrol-baseline-demo.json` is a fictional
multi-point office profile for offline replay.

## Card calls

The tool is a `processor` with no DDS topics. It supports the normal canvas
`start`, `stop`, and `info` lifecycle; those calls only change/report readiness.

| Action | Required arguments | Result |
| --- | --- | --- |
| `plan` | none | Ordered points, site, revision, mode, evidence age limit |
| `read` | `point_id` | Normal state, expected evidence kind, view and configured route for each check |
| `prepare_report` | `run_id`, `revision`, `point_id`, `check_id`, `decision` | Structured unsent deviation or review draft |

Use a stable lowercase `run_id` for one inspection round and pass the exact
revision returned by `plan`. An unknown point/check or changed revision is an
error. `decision` is `deviation` or `needs_review`; a deviation also needs a
nonblank `assessment`, an optional `priority` (`routine` or `urgent`), and evidence:

```json
{
  "point_id": "fixed-passage",
  "view_id": "bumi-front-fixed",
  "kind": "visual",
  "captured_at": "<actual capture time, ISO 8601 with timezone>",
  "reference": "<actual frame reference>",
  "observed": "<visible facts from that frame>"
}
```

The placeholders above are a field guide, not executable observations. The
Dashboard manual form may paste the evidence as JSON text. Missing, malformed,
future, stale, wrong-point, wrong-view, or wrong-kind evidence yields
`needs_review`, priority `review`, and the configured `review_route`. An agent
can also request review while retaining a fresh but ambiguous observation.
Evidence ages are compared with the driver's UTC clock, so validate clock
alignment and the capture timestamp's meaning before using live observations.

A draft contains the baseline, point/check, observation, assessment, route,
`report_key`, and `sent: false`. The route is taken only from the operator's
baseline. Rehearsal drafts have `delivery: rehearsal_only`; site profiles return
`delivery: pending`. There is no sender or persistent deduplication store in this
card. A separately authorized sender must handle `report_key`, delivery receipts,
and ambiguous delivery outcomes. A report is not evidence of notification delivery.

## Baseline format

Use the included JSON files as complete examples. The schema is
`motus.patrol-baseline/1`; its top-level fields are `schema`, `site_id`, `revision`,
`mode` (`rehearsal` or `site`), `max_age_seconds` (1–300), `review_route`, `routes`,
and `points`. Route values are labels, not network destinations. A point has
`point_id`, `label`, `waypoint_id`, `view_id`, and `checks`; each check has
`check_id`, `normal`, `evidence_kind` (`visual` or `sensor`), and `route_id`.
Identifiers are lowercase ASCII letters/digits with `_` or `-`, at most 64
characters. Profiles allow at most 64 points, 16 checks per point, and 32 routes.

## Agent instructions and stationary acceptance

The optional [`examples/office-baseline-patrol-skill.json`](examples/office-baseline-patrol-skill.json)
is an importable Skill example, not an automatically installed or published Skill.
Connect this card to Agent Core, then provide trustworthy observations through
the existing camera/capture workflow. Keep `rehearsal` mode for the first test.
The instructions cover multiple points, but the included Bumi profile limits
the first run to one stationary point. Navigation requires a separate approved
route, registered waypoints, localization, and obstacle handling.

1. Confirm the allocated Bumi test slot, stationary pose, fixed view, and the
   entire passage area that must be visible. Confirm the baseline with the operator.
2. Read `plan` and `read`; verify one point and the expected revision/view/check.
3. Obtain a new image using the existing `camera` / `vision_capture` path.
   Preserve the returned reference and timing metadata. At base commit
   `814b3ca953d33eba0aca9a61dcefa773f5da483b`, `capture_photo.captured_at` is
   **save time without a timezone**, and `frame_age_s` describes received-frame
   age. A recent cached frame may be saved. Passing that timestamp directly to
   this card produces `invalid_evidence`; adding a timezone alone does not prove
   the exposure time. Use a separately verified capture-time source, or keep
   the check as `needs_review`. `file_path` names a file on the robot/container;
   it does not automatically retrieve the image for Agent Core.
4. Compare Agent Core's description and coverage with the actual image. Only
   mark a check normal if the evidence covers it. Replay missing/stale evidence
   and check that the draft is routed to review with `sent: false`.
5. Record the driver commit/image, baseline revision, input reference, observed
   result, and operator verification. Keep route navigation and notification
   delivery as separate acceptance items.

## Offline verification and limits

From the repository root, run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_bumi_patrol_baseline.py tests/test_bumi_bundle.py -q
```

The tests use synthetic evidence and host-side hardware stubs. They check the
core rules, plugin lifecycle, optional bundle registration and MCP responses.
They do not establish camera availability, visual judgement quality, on-robot
container startup, navigation, or notification delivery. **Fixed-point Bumi
hardware acceptance is pending.**
