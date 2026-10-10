"""motus.camera/1 — what a camera says about its own optics.

**A camera's parameters belong to the camera.** They travel from the card that
owns the lens, along the canvas connections, to whoever needs them — and each
processor in between rewrites the parts its own processing changed, rather than
inventing a fresh copy and waiting for a human to fill it in.

── Why this exists ──────────────────────────────────────────────────────────

Nothing in a depth map carries geometry. It is 640x480 numbers, each a distance,
and nothing in it says how wide the lens was. But the decisions made from it are
metric: "is there room for my 0.36 m shoulders". So somewhere a pixel column has
to become a lateral offset in metres, and that conversion needs the field of
view.

Until this module, the number lived in the *navigation policy's* config file,
typed in by hand. On r1_sz it read 0.55 rad (~63 deg full, an ordinary lens)
while the actual lens measures 0.888 (~102 deg, ultra-wide). The avoidance
corridor is metric — half-width 0.329 m — so every frame it converted that width
back into a column range, and with the field of view understated the slice came
out too wide: at 1 m it sampled 84% of the picture's half-width, which really
spans +-0.93 m. **The corridor was 1.86 m wide, wider than any door.** The frame
of every doorway therefore counted as dead ahead, the clearance reading was the
distance to the door plane rather than through the opening, and the robot turned
away 0.6 m short of a door it could fit through.

The value had in fact been measured on that robot a day earlier. It went into a
report and not into the config, and nothing anywhere noticed. That is the shape
of failure this module is built against: **the camera knew, and had no way to
say so.**

Note how it failed. A wrong deadband makes a robot stutter and you see it
immediately. A wrong field of view makes a robot refuse doorways *while the
depth map reports clear ahead* — visible only through its consequences.

── The four rules ───────────────────────────────────────────────────────────

**1. A quantity that is not known is `null`, never a guess.** Reporting a
plausible number makes the consumer believe it knows, which is exactly what
happened above. A consumer must be able to tell "nobody told me" from "I was
told", so it can fall back conservatively *and say that it did*. This is the
same rule as `motus.odom/1`'s null axes and `descriptor.force_torque`: a
protection that is missing has to be visible.

**2. Meaning and units are fixed here, not in field names.** `half_fov_rad` is
the **horizontal half** field of view, in radians. The vertical one is a
separate field and **cannot be derived from the aspect ratio** — a processor
that resizes 1280x720 into 640x480 stretches the picture, so the pixels are no
longer square and the two angles are no longer related by the frame's shape.

**3. `source` is required.** Whether a number can be trusted is mostly a
question of where it came from, and `measured` / `vendor-spec` /
`derived-from-K` / `inherited` / `manual` are four very different kinds of
number. `unknown` is a legitimate answer and must be stated rather than omitted.

**4. `id` survives the whole chain; `width`/`height` are rewritten at each
stop; `pipeline` records who touched it.** Together these let a consumer look
up its own tables by camera identity, know which *image* the geometry describes,
and see who changed what when the number turns out wrong.

── Shape: ROS first, convenience second ─────────────────────────────────────

The core is `sensor_msgs/CameraInfo`'s: `width`, `height`, `distortion_model`,
`D`, `K`. Anything with a real calibration can fill those mechanically, and a
fisheye's distortion is a thing only `D` can express — `tan(theta)` overstates
the lateral offset towards the edges of a wide lens, so the pinhole model this
project uses today is an approximation that holds near the centre.

`half_fov_rad` sits alongside as a derived convenience, because what consumers
actually need is an angle and because a tape measure produces one directly while
producing no `K` at all. When both are present, `K` wins — see `resolve()`.

── Implementing this for another driver ─────────────────────────────────────

1. For each camera output port, call `build()` once and return the list under
   `camera_info` from that tool's `info()`. It is a declaration, not runtime
   state, so it can be answered whether or not the camera is streaming.
2. `id` is `"<vendor>/<model>/<port>"` and must be stable across reboots and
   across every unit of that model — downstream cards use it as a lookup key.
3. Report `half_fov_rad=None, source="unknown"` for a lens nobody has measured.
   Filling in a plausible number is the failure this module exists to prevent.
4. To measure one: `phanthymotus/actucore/tools/measure_fov.py`, a plane of
   known width at a tape-measured distance.

The contract is **this document plus the shapes below, not a shared package**.
perception implements its own copy against the same spec, exactly as with
`motus.control/1` — a shared library would couple the two repositories'
release cadence, which the `schema` field exists to keep separate.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

SCHEMA = "motus.camera/1"

# Where a geometry number came from. `unknown` is required rather than optional:
# a driver that has not measured its lens must say so, and omitting `source`
# would let that read as an oversight instead of a statement.
SOURCES = ("measured", "vendor-spec", "derived-from-K", "inherited", "manual",
           "unknown")

# `pinhole` is what this project's consumers assume. `fisheye` is declared
# honestly even while `D` is null, because it tells a consumer that the pinhole
# approximation it is about to make degrades towards the edges of the frame.
DISTORTION_MODELS = ("pinhole", "fisheye", "equidistant", "unknown")


class CameraInfoError(ValueError):
    """A camera declaration that cannot be trusted."""


@dataclass(frozen=True)
class CameraInfo:
    """One output port's optics.

    Frozen for the same reason `OdomInterface` is: consumers make decisions at
    start from it — how wide the avoidance corridor is, which calibration table
    row applies — and a declaration that changed underneath would invalidate
    them with nothing logged.
    """

    topic: str
    id: str
    format: str = ""
    width: int = None
    height: int = None
    distortion_model: str = "unknown"
    D: tuple = None
    K: tuple = None
    half_fov_rad: float = None
    half_fov_v_rad: float = None
    source: str = "unknown"
    measured_on: str = ""
    pipeline: tuple = ()
    vendor: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict, repr=False)

    @property
    def known(self) -> bool:
        """Whether this says anything usable about the horizontal geometry.

        The question every consumer asks first, and the one that decides
        between "use it" and "fall back conservatively and report that I did".
        """
        return self.half_fov_rad is not None or self.K is not None


def _positive_int(value, name: str):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise CameraInfoError(f"camera_info.{name} must be a positive integer or null, "
                              f"got {value!r}")
    return int(value)


def _angle(value, name: str, distortion_model: str):
    """A half field of view, or None.

    The upper bound is not decoration. `tan()` is what turns this into a lateral
    offset, and it diverges at pi/2 — a pinhole declaration at or past that is
    not a wide lens, it is a number that will produce infinities in a consumer
    that did nothing wrong. Genuinely hemispherical optics are allowed to say so
    as long as they do not also claim to be a pinhole.
    """
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise CameraInfoError(f"camera_info.{name} must be a positive number of "
                              f"radians or null, got {value!r}")
    value = float(value)
    if value >= math.pi:
        raise CameraInfoError(
            f"camera_info.{name} is {value:.3f} rad — a *half* field of view of "
            "180 deg or more is not a field of view. Note this field is the half "
            "angle, not the full one")
    if value >= math.pi / 2 and distortion_model not in ("fisheye", "equidistant"):
        # **The likeliest mistake in this whole format is passing the full angle.**
        # For r1_sz's 102 deg lens that is 1.78 rad, which clears the pi bound
        # above and would have been accepted — while making every corridor twice
        # as wide as intended, which is the exact bug this format was written
        # against. A genuine >=180 deg lens exists, but its owner knows it is a
        # fisheye and can say so; "unknown" plus a hemispherical angle is far
        # more likely to be a factor of two than a rare lens.
        raise CameraInfoError(
            f"camera_info.{name} is {value:.3f} rad, i.e. a full field of view "
            f"of {math.degrees(value) * 2:.0f} deg. This field is the **half** "
            f"angle — {value / 2:.3f} would be the value for that lens. If the "
            "lens really is hemispherical or wider, declare distortion_model "
            "'fisheye' or 'equidistant'; consumers convert this with tan(), "
            "which diverges at 90 deg")
    return value


def _matrix(value, name: str, size: int):
    if value is None:
        return None
    try:
        out = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        raise CameraInfoError(f"camera_info.{name} must be a list of numbers or null")
    if size and len(out) != size:
        raise CameraInfoError(f"camera_info.{name} must have {size} entries, "
                              f"got {len(out)}")
    return out


def parse(raw: dict) -> CameraInfo:
    """Validate one declaration, naming the offending field."""
    if not isinstance(raw, dict):
        raise CameraInfoError("camera_info entry must be an object")

    schema = raw.get("schema")
    if schema != SCHEMA:
        raise CameraInfoError(f"camera_info.schema must be {SCHEMA!r}, got {schema!r}")

    topic = str(raw.get("topic") or "")
    if not topic:
        raise CameraInfoError(
            "camera_info.topic is required — it is the key consumers join on, "
            "because a card may publish several ports and a consumer may have "
            "several inputs")

    ident = str(raw.get("id") or "")
    if not ident:
        raise CameraInfoError(
            "camera_info.id is required — downstream cards look up their own "
            "tables by it (a depth calibration, say), and it must stay the same "
            "all the way down the chain")

    model = raw.get("distortion_model", "unknown")
    if model not in DISTORTION_MODELS:
        raise CameraInfoError(
            f"camera_info.distortion_model must be one of "
            f"{', '.join(DISTORTION_MODELS)}, got {model!r}")

    source = raw.get("source", "unknown")
    if source not in SOURCES:
        raise CameraInfoError(
            f"camera_info.source must be one of {', '.join(SOURCES)}, got "
            f"{source!r} — how a number was arrived at is most of what says "
            "whether it can be trusted")

    half_fov = _angle(raw.get("half_fov_rad"), "half_fov_rad", model)
    half_fov_v = _angle(raw.get("half_fov_v_rad"), "half_fov_v_rad", model)
    K = _matrix(raw.get("K"), "K", 9)

    if (half_fov is not None or K is not None) and source == "unknown":
        raise CameraInfoError(
            "camera_info declares geometry but source is 'unknown' — say where "
            "the number came from, or report null")
    if half_fov is None and K is None and source not in ("unknown", "inherited"):
        raise CameraInfoError(
            f"camera_info declares no geometry but claims source {source!r} — "
            "a declaration with nothing in it is 'unknown'")

    pipeline = raw.get("pipeline") or ()
    if not isinstance(pipeline, (list, tuple)):
        raise CameraInfoError("camera_info.pipeline must be a list of stage names")

    return CameraInfo(
        topic=topic,
        id=ident,
        format=str(raw.get("format") or ""),
        width=_positive_int(raw.get("width"), "width"),
        height=_positive_int(raw.get("height"), "height"),
        distortion_model=model,
        D=_matrix(raw.get("D"), "D", 0),
        K=K,
        half_fov_rad=half_fov,
        half_fov_v_rad=half_fov_v,
        source=source,
        measured_on=str(raw.get("measured_on") or ""),
        pipeline=tuple(str(s) for s in pipeline),
        vendor=dict(raw.get("vendor") or {}),
        raw=dict(raw),
    )


def build(*, topic: str, id: str, format: str = "", width: int = None,
          height: int = None, distortion_model: str = "unknown",
          D=None, K=None, half_fov_rad: float = None,
          half_fov_v_rad: float = None, source: str = "unknown",
          measured_on: str = "", pipeline=(), vendor: dict = None) -> dict:
    """One output port's declaration, for a tool's `info()`.

    Runs through `parse()` before returning, so a driver cannot ship a
    declaration its own consumers would reject — the mistake surfaces in that
    driver's unit test rather than on a robot, which for this particular number
    means "rather than at a doorway".
    """
    raw = {
        "schema": SCHEMA,
        "topic": topic,
        "id": id,
        "format": format,
        "width": width,
        "height": height,
        "distortion_model": distortion_model,
        "D": list(D) if D is not None else None,
        "K": list(K) if K is not None else None,
        "half_fov_rad": half_fov_rad,
        "half_fov_v_rad": half_fov_v_rad,
        "source": source,
        "measured_on": measured_on,
        "pipeline": list(pipeline),
        "vendor": dict(vendor or {}),
    }
    parse(raw)
    return raw


def for_topic(declarations, topic: str) -> CameraInfo:
    """The declaration covering `topic`, or None.

    How a consumer joins: it knows which topic it bound as its depth input, and
    looks that topic up. Not by position in the list and not by card name —
    a card may publish several ports, and naming the upstream card would undo
    the whole point of dispatching inputs by what they carry.
    """
    for entry in declarations or ():
        try:
            parsed = parse(entry)
        except CameraInfoError:
            continue
        if parsed.topic == topic:
            return parsed
    return None


def resolve_half_fov(info: CameraInfo) -> tuple:
    """`(half_fov_rad, source)` for the horizontal axis, or `(None, 'unknown')`.

    **`K` wins over the declared angle when both are present.** A calibration
    matrix is solved from many observations; the angle beside it is usually a
    tape measure and a bit of trigonometry. Preferring the coarser number
    because it happens to be in a more convenient field would be backwards.
    """
    if info is None:
        return None, "unknown"
    if info.K is not None and info.width:
        fx = info.K[0]
        if fx > 0:
            return math.atan((info.width / 2.0) / fx), "derived-from-K"
    if info.half_fov_rad is not None:
        return info.half_fov_rad, info.source
    return None, "unknown"
