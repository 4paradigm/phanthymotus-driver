"""motus.odom/1 — what a robot reports about its own motion.

**This is the measurement dual of `motus.control/1`'s `twist` mode.** Same six
axes, same order, same body frame, same SI units. A consumer that commands
`values = [vx, vy, vz, wx, wy, wz]` and reads back `twist = [vx, vy, vz, wx, wy,
wz]` compares the two with a subtraction and no lookup table. That is the entire
reason this shape was chosen, and the reason it is not a sixth dialect.

── Why a new format, rather than adopting one that exists ────────────────────

The same physical quantity already has five shapes in this repository, and the
disagreement is not cosmetic — it is in the container type, in where the units
live, and in whether an unmeasured axis is reported at all:

| driver | linear | angular |
|---|---|---|
| Unitree R1/G1/Go2 | `velocity: [x,y,z]` | `yaw_speed`, a scalar |
| Unitree Go1 | `velocity_body_mps: {forward, lateral}` + `velocity_index_2_raw` | `yaw_speed_rad_s` |
| Booster K1 | `linear_velocity: [...]` | `angular_velocity: [...]` |
| EngineAI T800 | `linear_velocity: {x,y,z}` + `speed_m_s` + `valid` | `yaw_rate_rad_s` |
| EngineAI T800, elsewhere in the same file | `linear_velocity: [...]` | — |

Two of those deserve pointing at. Go1's third component is named
`velocity_index_2_raw` — the field name is an admission that nobody knows what it
means. And T800 grew a `valid` flag on its own, which means somebody already hit
the failure below and fixed it locally, for one driver.

── The four rules, each answering one of those ───────────────────────────────

**1. An axis that was not measured is `null`, never `0.0`.** This is the one
that matters. A robot that does not report speed, reporting zero, is
indistinguishable from a robot that is standing still — and that is precisely
the failure mode of everything that consumes this. A stuck-detector ("commanded
0.3 m/s, measured nothing, therefore we have hit something") fires on every robot
that simply has no odometry. The same reasoning as `descriptor.force_torque`
being required even as `null`: a protection that is missing must be visible, not
assumed. Per **axis** rather than one `valid` flag for the sample, because
Go1-shaped partial knowledge — two axes trustworthy, the third meaningless — is
the normal case, not an edge case.

**2. Units live in `units`, not in field names.** `yaw_speed_rad_s`,
`speed_m_s`, `position_m` — that convention is not even self-consistent within
one vendor, and it cannot survive a driver that reports degrees.

**3. `frame` is required.** Body frame versus world frame is the most dangerous
ambiguity here and not one of the five existing shapes states which it is. A
consumer that assumes wrong gets plausible numbers with the sign of the lateral
axis flipped whenever the robot is not facing along the world x-axis.

**4. Everything vendor-specific goes in `vendor`, untouched.** R1's `mode`,
`gait_type` and `body_height`; a wheeled base's battery and wheel RPM; a drone's
barometric altitude. Nothing is lost by adopting this format, and a consumer
written against the core cannot be broken by a driver adding to `vendor`. This
is the extensible half: **extensions grow into `vendor`, the core stays frozen.**

── Declaration and data are separate, as in motus.control/1 ─────────────────

The `null`s in a sample say "not in this frame". A consumer also needs to know,
*at start*, that a robot never reports speed at all, so it can degrade or refuse
then and there rather than discovering it at 10 Hz. So a driver's state card
returns an `OdomInterface` from `info()` — read once, negotiated against — while
each message carries the data. Exactly the `control_interface` / `ControlSink`
split, and agent-core's existing `info()` path needs no changes to carry it.

Free of ROS, of any vendor SDK, and of the bundles, for the same reason
`common/control` is: what arrives here is a plain dict, so this is testable on a
laptop. See tests/test_odom_format.py.

Spec for driver authors: README_dev.md § "Robot Odometry (motus.odom/1)".
"""

from __future__ import annotations

from dataclasses import dataclass, field

SCHEMA = "motus.odom/1"

# The six axes, in `motus.control/1` twist order. Do not reorder: the whole
# point of this format is that index i here means index i there.
AXES = ("vx", "vy", "vz", "wx", "wy", "wz")

# How much a `pose` can be trusted. Stated rather than implied, because the
# difference decides whether anything may accumulate it.
#
#   none       no pose reported
#   unbounded  legged dead reckoning — fine for "how far since a moment ago",
#              useless as an absolute position, must not be mapped against
#   bounded    wheel odometry with a correction source, or a SLAM pose
DRIFT_KINDS = ("none", "unbounded", "bounded")

# The only frame a motion consumer can use without a transform. `world` is
# accepted in the format but a consumer is entitled to refuse it — see
# `OdomInterface.usable_for_control`.
BODY_FRAME = "body"
FRAMES = (BODY_FRAME, "world")

DEFAULT_UNITS = {"linear": "m/s", "angular": "rad/s", "length": "m"}


class OdomError(ValueError):
    """An odometry declaration or sample that cannot be trusted."""


@dataclass(frozen=True)
class OdomInterface:
    """What a driver declares it reports about its own motion.

    Frozen for the same reason `Descriptor` is: consumers cache decisions made
    from it at start (whether stuck detection is even possible, say), and a
    declaration that changed underneath would invalidate them silently.
    """

    frame: str
    provides: tuple[str, ...]
    rate_hz: float = 0.0
    pose_drift: str = "none"
    units: dict = field(default_factory=lambda: dict(DEFAULT_UNITS))
    raw: dict = field(default_factory=dict, repr=False)

    def has(self, axis: str) -> bool:
        return axis in self.provides

    @property
    def usable_for_control(self) -> bool:
        """Whether a motion consumer may use this without a transform.

        A `world`-frame report is real odometry and worth publishing, but
        comparing it against a body-frame command needs the robot's heading and
        a rotation — which is a different feature. Consumers should check this
        and say so, rather than quietly treating world as body.
        """
        return self.frame == BODY_FRAME


def parse_interface(raw: dict) -> OdomInterface:
    """Validate a driver's odometry declaration, naming the offending field."""
    if not isinstance(raw, dict):
        raise OdomError("odom_interface must be an object")

    schema = raw.get("schema")
    if schema != SCHEMA:
        raise OdomError(f"odom_interface.schema must be {SCHEMA!r}, got {schema!r}")

    frame = raw.get("frame")
    if frame not in FRAMES:
        raise OdomError(
            f"odom_interface.frame must be one of {', '.join(FRAMES)}, got {frame!r} — "
            "which frame the numbers are in is not something a consumer can infer"
        )

    provides = raw.get("provides")
    if not isinstance(provides, (list, tuple)):
        raise OdomError("odom_interface.provides must be a list of axis names")
    unknown = [a for a in provides if a not in AXES]
    if unknown:
        raise OdomError(
            f"odom_interface.provides names {', '.join(map(repr, unknown))}, "
            f"which are not axes — the six are {', '.join(AXES)}"
        )

    drift = raw.get("pose_drift", "none")
    if drift not in DRIFT_KINDS:
        raise OdomError(
            f"odom_interface.pose_drift must be one of {', '.join(DRIFT_KINDS)}, "
            f"got {drift!r}"
        )

    rate_hz = raw.get("rate_hz", 0.0) or 0.0
    if isinstance(rate_hz, bool) or not isinstance(rate_hz, (int, float)) or rate_hz < 0:
        raise OdomError("odom_interface.rate_hz must be a non-negative number")

    units = raw.get("units") or dict(DEFAULT_UNITS)
    if not isinstance(units, dict) or not units:
        raise OdomError("odom_interface.units must be a non-empty object")

    return OdomInterface(
        frame=frame,
        provides=tuple(provides),
        rate_hz=float(rate_hz),
        pose_drift=drift,
        units=dict(units),
        raw=dict(raw),
    )


def build_interface(*, provides, frame: str = BODY_FRAME, rate_hz: float = 0.0,
                    pose_drift: str = "none", units: dict = None) -> dict:
    """The declaration a driver's state card returns from `info()`.

    Goes through `parse_interface` before being returned, so a driver cannot
    ship a declaration that its own consumers would reject — the mistake is
    caught in that driver's unit test rather than on a robot.
    """
    raw = {
        "schema": SCHEMA,
        "frame": frame,
        "provides": list(provides),
        "rate_hz": float(rate_hz),
        "pose_drift": pose_drift,
        "units": dict(units or DEFAULT_UNITS),
    }
    parse_interface(raw)
    return raw


def build_sample(*, stamp_ms: int, twist, frame: str = BODY_FRAME,
                 pose: dict = None, contact: dict = None, vendor: dict = None,
                 units: dict = None) -> dict:
    """One odometry sample.

    `twist` is six entries in `AXES` order; use `None` for an axis this robot
    does not measure. Passing `0.0` for such an axis is the mistake this whole
    module exists to prevent, so the length is checked and the values are passed
    through without being coerced — a `None` stays a `None` all the way to JSON's
    `null`.
    """
    values = list(twist)
    if len(values) != len(AXES):
        raise OdomError(
            f"twist has {len(values)} entries, expected {len(AXES)} "
            f"({', '.join(AXES)}) — use None for an axis this robot does not measure"
        )
    out = {
        "schema": SCHEMA,
        "stamp_ms": int(stamp_ms),
        "frame": frame,
        "units": dict(units or DEFAULT_UNITS),
        "twist": [None if v is None else float(v) for v in values],
        "pose": pose,
    }
    if contact is not None:
        out["contact"] = contact
    if vendor:
        out["vendor"] = vendor
    return out


def axis(sample: dict, name: str):
    """One axis out of a sample, or `None` if absent or unmeasured.

    Use this rather than indexing `sample["twist"]` directly. It is the single
    place that keeps "the robot is not moving" and "the robot does not know"
    apart, and every consumer needs that distinction — reading the list by index
    makes it one `or 0.0` away from being lost.
    """
    try:
        index = AXES.index(name)
    except ValueError:
        raise OdomError(f"{name!r} is not an axis; the six are {', '.join(AXES)}")
    values = (sample or {}).get("twist") or []
    if index >= len(values):
        return None
    return values[index]


def is_fresh(sample: dict, now_ms: int, max_age_ms: int) -> bool:
    """Whether a sample is recent enough to act on.

    A sample with no `stamp_ms` is **not** fresh. A driver that forgets the
    timestamp would otherwise hand a consumer an arbitrarily old reading that
    looks current, which for a stuck-detector means acting on the speed the
    robot had before it stopped.
    """
    stamp = (sample or {}).get("stamp_ms")
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
        return False
    return 0 <= (now_ms - stamp) <= max_age_ms


# ── producing a sample from a high-rate vendor topic ─────────────────────────
#
# Every driver adopting this format faces the same two problems, because every
# vendor publishes its state far faster than 10 Hz and stamps it with its own
# clock. Both are solved here rather than in each bundle: the mistakes are
# identical in all of them, and both are silent.


def mean_twist(readings) -> list:
    """Average a burst of twist readings into one, per axis.

    **Downsample by averaging, not by picking.** A vendor publishing at 495 Hz
    and a card publishing at 10 Hz means 49 readings are discarded per sample,
    and taking the first one of each window is an unfiltered decimation — the
    noise that averaging would have removed is aliased into the output instead.
    That matters for exactly the consumer this format was shaped around: a
    stuck-detector comparing a commanded speed against a measured one is reading
    a threshold crossing, and a noisy sample crosses thresholds it should not.

    **`None` survives.** An axis is averaged over the readings that carry a
    number, and stays `None` when none of them do — never 0.0, which is the one
    rule this whole module exists to protect. An axis the robot reports
    intermittently (Go1-shaped partial knowledge) averages over what arrived,
    because the alternative is discarding the only measurements there are.

    An empty burst gives all-`None`: nothing arrived, so nothing is known. That
    is the honest answer and not the same as a robot standing still.
    """
    totals = [0.0] * len(AXES)
    counts = [0] * len(AXES)
    for reading in readings or ():
        values = list(reading or ())
        for index in range(min(len(values), len(AXES))):
            value = values[index]
            if value is None or isinstance(value, bool):
                continue
            if not isinstance(value, (int, float)):
                continue
            totals[index] += float(value)
            counts[index] += 1
    return [totals[i] / counts[i] if counts[i] else None
            for i in range(len(AXES))]


# How far a robot's own clock may sit from ours before we stop quoting it. Two
# seconds is far wider than any transport delay and far narrower than the offset
# an unsynchronised clock shows, so it separates the two cases without needing
# to know which robot this is.
MAX_CLOCK_SKEW_MS = 2000


def resolve_stamp_ms(*, vendor_ms, received_ms, max_skew_ms=MAX_CLOCK_SKEW_MS):
    """Pick the timestamp to publish, and say where it came from.

    Returns `(stamp_ms, provenance)`; put the provenance straight into the
    sample's `vendor` block.

    The spec asks for **when the reading was taken**, and a vendor message
    normally carries that, so quote it. But `stamp_ms` has a second job the spec
    does not spell out: `is_fresh` subtracts it from the *consumer's* clock. A
    robot whose clock is not synchronised with ours satisfies the first job and
    destroys the second — every sample reads as minutes old, or as arriving from
    the future, and a consumer that trusts it stops the robot for blindness it
    does not have. Neither failure says anything about a clock.

    So a vendor stamp is used only when it is plausible as a wall clock *and*
    close to ours; otherwise the arrival time is published, which is at least
    comparable, and the reason is recorded where a reader will find it. Falling
    back is not a silent approximation — `stamp_source` distinguishes the two,
    and nothing downstream has to guess which it got.

    Deliberately **not** offset-corrected. Subtracting a measured offset would
    let a skewed clock be quoted, and on a robot whose clock drifts rather than
    merely sits offset it would decay into a wrong answer that still looks
    principled. Measure the skew first — `stamp_skew_ms` is reported for that
    purpose — and add correction only if it turns out to be constant.
    """
    received = int(received_ms)
    # NaN and infinity are in this list because a vendor SDK deserialising a
    # float field can produce them, and `int(nan)` raises — a driver's odometry
    # would then stop on a bad reading rather than publish a fallback.
    if (vendor_ms is None or isinstance(vendor_ms, bool)
            or not isinstance(vendor_ms, (int, float))
            or vendor_ms != vendor_ms or vendor_ms in (float("inf"), float("-inf"))):
        return received, {"stamp_source": "received",
                          "stamp_reason": "vendor message carried no usable stamp"}
    vendor = int(vendor_ms)
    # A boot-relative clock (seconds since power-on) lands here: a small number
    # that is a perfectly good relative time and a nonsensical absolute one.
    if vendor < 1_000_000_000_000:
        return received, {"stamp_source": "received",
                          "stamp_reason": "vendor stamp is not a wall clock "
                                          f"({vendor} ms)",
                          "stamp_skew_ms": received - vendor}
    skew = received - vendor
    if abs(skew) > int(max_skew_ms):
        return received, {"stamp_source": "received",
                          "stamp_reason": f"vendor clock is {skew} ms off ours",
                          "stamp_skew_ms": skew}
    return vendor, {"stamp_source": "robot", "stamp_skew_ms": skew}
