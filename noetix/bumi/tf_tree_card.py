"""TfTreePlugin — the `tf_tree` card for the Bumi bundle.

A processor card that turns the robot into a world of coordinate frames. It
eats three things and publishes one:

  * the URDF (parsed once, at `/work/resource/bumi_model.urdf` by default) —
    the link/joint topology and the `fixed` joints;
  * the live joint state — a `std_msgs/String` carrying JSON
    ``{"joints": [{"name", "q", ...}, ...]}`` on ``/{ns}/state/joints`` at
    10 Hz, not a `sensor_msgs/JointState`;
  * the vision transform JSON from the `vision_tf` card (optional) — a
    ``data/json`` batch of ``parent → child`` transforms.

From those it computes forward kinematics itself (no `robot_state_publisher`
subprocess) and publishes the whole tree on `/tf` (the 21 revolute joints) and
`/tf_static` (the fixed joints, latched). A downstream card that wants "the
pose of A relative to C" subscribes with `tf2_ros` and calls `lookup_transform`;
it never goes through this card.

**It commands nothing and publishes no pose data.** The only topic this card
owns is `/{ns}/tf_tree/status`, and that is a *health* report about the tree —
frame counts, publish rate, joint-sample age — not a pose source. Do not read
`status` as a transform.

Timestamp policy
----------------
A transform's `header.stamp` is the joint sample's own time, or the receive
time when the sample carries none — never `now()`. A joint sample older than
`max_joint_age_s` stops the dynamic `/tf` stream and reports
`degraded: joints_stale`; a silent `now()` would instead make a two-second-old
pose look fresh, so a downstream hand pose would be wrong with nothing anywhere
raising an error. This is the same "say it is stale rather than pretend it is
fresh" stance the `media_status` card takes for unreadable states.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import tf_tree_spec as spec

# The joint-state publisher (device.py `_LOW_LAT_QOS`) is BEST_EFFORT with
# depth 200. A default RELIABLE depth-10 subscriber silently receives nothing,
# so the subscription must declare the same profile.
_JOINT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

# Only the newest vision batch matters; a queue only adds age.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_DEFAULT_URDF_PATH = "/work/resource/bumi_model.urdf"


class _TfTreeNode(Node):
    """Owns the tf broadcasters, the joint/vision subscriptions and the status
    topic. Purely mechanical; the decisions live in `tf_tree_spec`."""

    def __init__(self, namespace: str, urdf_path: str, publish_rate_hz: float,
                 max_joint_age_s: float, vision_ttl_ms: int):
        super().__init__("bumi_tf_tree")
        self._namespace = namespace
        self._joints_topic = f"/{namespace}/state/joints"
        self._status_topic = f"/{namespace}/tf_tree/status"
        self._urdf_path = urdf_path
        self._publish_rate_hz = publish_rate_hz
        self._max_joint_age_s = max_joint_age_s
        self._vision_ttl_ms = vision_ttl_ms

        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        # The two broadcasters. `/tf_static` must be a StaticTransformBroadcaster
        # (latched) or a late subscriber never sees the fixed chain; `/tf` is a
        # plain TransformBroadcaster whose entries expire naturally in tf2's cache.
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        self._static_bc = StaticTransformBroadcaster(self)
        self._dynamic_bc = TransformBroadcaster(self)

        self._tree = None
        self._parse_error = None
        self._joint_values = {}
        self._last_joint_at = None  # monotonic seconds of the newest sample
        self._last_stamp_ns = 0

        # Vision state: the live child→transform entries, keyed by child frame.
        # The ROS callback and the status/TTL thread touch these from different
        # threads, so all reads/writes go through _vision_lock; snapshots are
        # taken under it before broadcasting.
        self._vision_lock = threading.RLock()
        self._vision = {}          # child -> {parent, translation, rotation, ...}
        self._vision_parent = {}   # child -> parent, for conflict detection
        self._conflicting_children = set()  # transient: cleared when a child is unambiguous again
        self._rejected_unknown_parent = 0
        self._expired_vision = 0
        self._rejected_bad_payload = 0
        self._rejected_bad_joints = 0
        self._stale_joint_samples = 0
        self._conflicting_parent = 0  # cumulative counter for the status topic
        self._camera_extrinsic_assumed = False  # zero mount translation = unmeasured

        self._running = False
        self._generation = 0
        self._stopped = threading.Event()
        self._thread = None

        self._joints_sub = None
        self._vision_sub = None
        self._vision_topic = ""

        self._tf_hz = 0.0
        self._publish_count = 0
        self._hz_window_at = 0.0
        self._last_status_at = 0.0

        self._load_urdf()

    # ── URDF ─────────────────────────────────────────────────────────────────

    def _load_urdf(self) -> None:
        """Parse the URDF once. A failure is a parse-time error to report, not
        a reason to crash the card: the node keeps running and `start` answers
        with an actuator-style `error`."""
        self._parse_error = None
        try:
            text = Path(self._urdf_path).read_text(encoding="utf-8")
        except OSError as exc:
            self._parse_error = f"cannot read URDF {self._urdf_path}: {exc}"
            self._tree = None
            return
        # A zero camera-mount translation is a declared placeholder, not a
        # measurement. Surface that as a machine-readable flag on the health
        # topic so consumers do not mistake the published static transform for
        # a calibrated extrinsic.
        self._camera_extrinsic_assumed = False
        try:
            self._tree = spec.parse_urdf(text)
            camera_joint = next(
                (j for j in self._tree["joints"].values()
                 if j["child"] == "camera_link"), None)
            if camera_joint is not None:
                xyz = camera_joint["origin"][0]
                self._camera_extrinsic_assumed = all(abs(v) < 1e-12 for v in xyz)
        except ValueError as exc:
            self._parse_error = str(exc)
            self._tree = None

    # ── lifecycle ───────────────────────────────────────────────────────────

    @property
    def status_topic(self) -> str:
        return self._status_topic

    @property
    def running(self) -> bool:
        return self._running

    @property
    def parse_error(self) -> str | None:
        return self._parse_error

    @property
    def frame_counts(self) -> tuple:
        """→ (link_count, dynamic_frames, static_frames) or (0, 0, 0) when unparsed."""
        if self._tree is None:
            return (0, 0, 0)
        joints = self._tree["joints"]
        dynamic = sum(1 for j in joints.values() if j["type"] == "revolute")
        static = sum(1 for j in joints.values() if j["type"] == "fixed")
        return (len(self._tree["links"]), dynamic, static)

    def start(self, input_topic: str) -> None:
        # The tree is enabled by default and starts at boot with no vision
        # topic (start_all -> plugin.start -> start("")). Later the canvas
        # dispatches start with the resolved input_topic. An already-running
        # tree must therefore reconfigure its vision subscription rather than
        # no-op — otherwise the advertised vision_tf -> tf_tree link never
        # actually subscribes, and no vision edge is ever published.
        if self._running:
            if input_topic:
                self._subscribe_vision(input_topic)
            else:
                # A canvas rewire to "no input" must drop the previous source's
                # transforms now, not let them linger for a TTL window.
                self._teardown_vision()
                self._clear_vision_state()
                self._vision_topic = ""
            return
        self._stopped.clear()
        self._running = True
        self._generation += 1

        # Publish the static chain now (and again if the URDF is reloaded).
        self._publish_static()

        self._subscribe_joints()
        if input_topic:
            self._subscribe_vision(input_topic)

        self._thread = threading.Thread(
            target=self._loop, args=(self._generation,),
            daemon=True, name="bumi_tf_tree")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._stopped.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._teardown_subs()
        self._clear_vision_state()

    def reload_urdf(self) -> bool:
        """Re-parse and re-publish the static chain. Returns True on success."""
        self._load_urdf()
        if self._tree is not None and self._running:
            self._publish_static()
        return self._tree is not None

    # ── subscriptions ───────────────────────────────────────────────────────

    def _subscribe_joints(self) -> None:
        if self._joints_sub is not None:
            return
        self._joints_sub = self.create_subscription(
            String, self._joints_topic, self._on_joints, _JOINT_QOS)

    def _clear_vision_state(self) -> None:
        """Forget all vision edges when the input source goes away or changes.

        Stale edges from a disconnected source would otherwise be broadcast by
        the loop for up to ``vision_ttl_ms`` (or interleaved with a new source)
        — attaching objects from a source the operator no longer wired in.
        The next real batch repopulates the state; TTL still handles targets
        that simply walk out of frame within one source.
        """
        with self._vision_lock:
            self._vision.clear()
            self._vision_parent.clear()
            self._conflicting_children.clear()

    def _subscribe_vision(self, topic: str) -> None:
        self._teardown_vision()
        self._clear_vision_state()
        self._vision_topic = topic
        self._vision_sub = self.create_subscription(
            String, topic, self._on_vision, _LATEST_QOS)

    def _teardown_subs(self) -> None:
        for attr in ("_joints_sub", "_vision_sub"):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:
                    pass
                setattr(self, attr, None)

    def _teardown_vision(self) -> None:
        if self._vision_sub is not None:
            try:
                self.destroy_subscription(self._vision_sub)
            except Exception:
                pass
            self._vision_sub = None

    # ── callbacks ───────────────────────────────────────────────────────────

    def _on_joints(self, msg: String) -> None:
        """Parse the joint JSON and publish one round of dynamic tf.

        Timestamp from the producer's read-completion stamp (device.py now
        emits `stamp_ns`), not from callback time: the joint topic is a deep
        best-effort queue, so callback time could present a seconds-old pose as
        current. A sample already older than max_joint_age_s when read is
        rejected outright rather than broadcast and then declared stale.
        """
        try:
            data = json.loads(msg.data or "{}")
        except (ValueError, TypeError):
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        sample_ns = data.get("stamp_ns")
        try:
            sample_ns = int(sample_ns)
        except (TypeError, ValueError):
            sample_ns = None
        if sample_ns is not None:
            age_s = (now_ns - sample_ns) / 1e9
            if age_s < -0.001 or age_s > self._max_joint_age_s:
                # Future-dated or already-stale at receipt: never publish it.
                self._stale_joint_samples += 1
                return

        joints = data.get("joints") or []
        values = {}
        bad_joint_values = 0
        for joint in joints:
            name = joint.get("name")
            if name is None:
                continue
            try:
                q = float(joint.get("q", 0.0))
            except (TypeError, ValueError, OverflowError):
                bad_joint_values += 1
                continue
            # A NaN/Inf from a bad SDK reading must not enter forward kinematics:
            # it produces invalid TF values and can divide by zero in
            # quat_from_matrix, aborting this callback and freezing the tree.
            # Drop the sample rather than publish a poisoned pose.
            if not math.isfinite(q):
                bad_joint_values += 1
                continue
            values[name] = q
        if bad_joint_values:
            self._rejected_bad_joints += bad_joint_values
            # Reject the WHOLE sample, not just the bad joint. A partial update
            # would leave the bad joint to FK's `.get(name, 0.0)` default — an
            # unsignalled snap to q=0, which is worse than freezing at the last
            # good pose. If bad readings persist, the age check reports stale.
            return
        if not values:
            return
        self._joint_values = values
        self._last_joint_at = time.monotonic()
        # Stamp the TF edges with the sample's own read-completion time.
        self._last_stamp_ns = sample_ns if sample_ns is not None else now_ns

        if self._tree is None:
            return
        self._publish_dynamic()

    def _on_vision(self, msg: String) -> None:
        """Merge one vision batch through the tree's single broadcaster."""
        try:
            data = json.loads(msg.data or "{}")
        except (ValueError, TypeError):
            return
        now_ms = int(time.time() * 1000)
        live, report = spec.merge_vision(
            data.get("transforms"), now_ms, self._vision_ttl_ms)

        with self._vision_lock:
            self._rejected_unknown_parent += report["unknown_parent"]
            self._expired_vision += report["expired"]
            self._rejected_bad_payload += report["invalid_payload"]

            # The tf2 constraint is *simultaneous* parents for one child, not a
            # parent changing between batches (a tracked target can legitimately
            # re-appear under another frame). Detect conflicts within this batch;
            # accept a changed parent from the previous batch as new truth.
            batch_parents = {}
            for entry in live:
                batch_parents.setdefault(entry.get("child"), set()).add(
                    entry.get("parent"))
            for child, parents in batch_parents.items():
                if len(parents) > 1:
                    self._conflicting_parent += 1
                    self._conflicting_children.add(child)
                else:
                    self._conflicting_children.discard(child)

            # Only unambiguous children are published; on conflict keep the last
            # known transform rather than picking an arbitrary parent.
            for entry in live:
                child = entry.get("child")
                if child in self._conflicting_children:
                    continue
                self._vision_parent[child] = entry.get("parent")
                self._vision[child] = entry

            # TTL is the single expiry mechanism. A target that walked out of
            # frame ages out on its own; relying on "missing in the new batch"
            # would break an upstream that publishes only changed entries.
            self._prune_vision(now_ms)
            # Snapshot under the lock; the broadcaster may be called from the
            # status thread concurrently, so never iterate the live dict there.
            to_publish = [self._vision_transform(entry)
                          for entry in self._vision.values()]
        if to_publish:
            self._dynamic_bc.sendTransform(to_publish)

    def _prune_vision(self, now_ms: int) -> None:
        """Drop live vision entries whose own stamp aged past the TTL."""
        expired = []
        for child, entry in self._vision.items():
            stamp_ms = entry.get("stamp_ms")
            if stamp_ms is not None and now_ms - float(stamp_ms) > self._vision_ttl_ms:
                expired.append(child)
        for child in expired:
            self._vision.pop(child, None)
            self._vision_parent.pop(child, None)
            self._conflicting_children.discard(child)
        self._expired_vision += len(expired)

    def _vision_transform(self, entry):
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time
        t = TransformStamped()
        # Use the vision sample's own stamp, not publish time, for the same
        # reason as the joint path: a late delivery must not look fresh.
        stamp_ms = entry.get("stamp_ms")
        if stamp_ms is not None:
            t.header.stamp = Time(nanoseconds=int(float(stamp_ms) * 1_000_000)).to_msg()
        else:
            t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = entry["parent"]
        t.child_frame_id = entry["child"]
        translation = entry.get("translation") or [0.0, 0.0, 0.0]
        rotation = entry.get("rotation") or [0.0, 0.0, 0.0, 1.0]
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])
        return t

    # ── publishing ──────────────────────────────────────────────────────────

    @staticmethod
    def _stamped_transform(joint, xyz, quat, stamp):
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = stamp
        # TF is a parent→child *local* transform. xyz/quat must be local to
        # `parent`, never the root-relative FK pose (see joint_local_pose).
        t.header.frame_id = joint["parent"]
        t.child_frame_id = joint["child"]
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def _publish_static(self) -> None:
        """Publish every fixed joint's local transform on the latched `/tf_static`."""
        if self._tree is None:
            return
        # Static transforms are structural: they are independent of joint values,
        # so an arriving joint sample never needs to re-send them.
        stamp = self.get_clock().now().to_msg()
        stamped = [
            self._stamped_transform(
                joint, *spec.joint_local_pose(joint), stamp)
            for joint in self._tree["joints"].values()
            if joint["type"] == "fixed"
        ]
        if stamped:
            self._static_bc.sendTransform(stamped)

    def _publish_dynamic(self) -> None:
        """Publish the 21 revolute joints' local transforms on `/tf`.

        The stamp is the joint sample's own time, not publish time: a late
        callback would otherwise make a stale pose look current to every
        downstream `lookup_transform` caller.
        """
        if self._tree is None or self._last_stamp_ns == 0:
            return
        from rclpy.time import Time
        stamp = Time(nanoseconds=self._last_stamp_ns).to_msg()
        stamped = []
        for name, joint in self._tree["joints"].items():
            if joint["type"] != "revolute":
                continue
            q = float(self._joint_values.get(name, 0.0))
            xyz, quat = spec.joint_local_pose(joint, q)
            stamped.append(self._stamped_transform(joint, xyz, quat, stamp))
        if stamped:
            self._dynamic_bc.sendTransform(stamped)
        self._publish_count += 1

    # ── the loop ────────────────────────────────────────────────────────────

    def _joint_age_s(self) -> float | None:
        if self._last_joint_at is None:
            return None
        return time.monotonic() - self._last_joint_at

    def _degraded(self) -> str:
        if self._parse_error is not None:
            return "urdf_unparsed"
        if self._last_joint_at is None:
            return "no_joints"
        with self._vision_lock:
            conflicting = bool(self._conflicting_children)
        if conflicting:
            return "conflicting_parent"
        if self._joint_age_s() > self._max_joint_age_s:
            return "joints_stale"
        return ""

    def _loop(self, generation: int) -> None:
        """The backstop timer: publish the status topic at `publish_rate_hz` and
        let a stale joint sample surface as `joints_stale` instead of a pose that
        pretends to be current."""
        interval = 1.0 / self._publish_rate_hz
        self._hz_window_at = time.monotonic()
        self._publish_count = 0
        while self._running and generation == self._generation:
            # Vision TTL must decay even when the upstream card went silent —
            # no incoming batch means no other chance to sweep stale frames.
            with self._vision_lock:
                had = len(self._vision)
                self._prune_vision(int(time.time() * 1000))
                to_publish = ([self._vision_transform(entry)
                               for entry in self._vision.values()]
                              if len(self._vision) != had and self._vision else [])
            if to_publish:
                self._dynamic_bc.sendTransform(to_publish)
            self._publish_status()
            if self._stopped.wait(interval):
                break

    def _publish_status(self) -> None:
        now = time.monotonic()
        # A rolling rate over the window since the last status publish.
        elapsed = now - self._hz_window_at
        if elapsed > 0:
            self._tf_hz = self._publish_count / elapsed
        self._hz_window_at = now
        self._publish_count = 0

        link_count, dynamic, static = self.frame_counts
        age = self._joint_age_s()
        with self._vision_lock:
            vision_targets = len(self._vision)
            rejected = self._rejected_unknown_parent
            expired = self._expired_vision
            bad_payload = self._rejected_bad_payload
            bad_joints = self._rejected_bad_joints
            conflicting = self._conflicting_parent
        payload = spec.status_payload(
            state="running" if self._running else "idle",
            stamp_ms=int(time.time() * 1000),
            root_frame=self._tree["root"] if self._tree else "",
            link_count=link_count,
            dynamic_frames=dynamic,
            static_frames=static,
            tf_hz=self._tf_hz,
            last_joint_age_ms=int((age * 1000) if age is not None else -1),
            joints_stale=(age is not None and age > self._max_joint_age_s),
            vision_targets=vision_targets,
            rejected_unknown_parent=rejected,
            expired_vision=expired,
            rejected_bad_payload=bad_payload,
            rejected_bad_joints=bad_joints,
            conflicting_parent=conflicting,
            degraded=self._degraded(),
            last_error=self._parse_error or "",
            camera_extrinsic_assumed=self._camera_extrinsic_assumed,
        )
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(message)

    # ── info snapshot ───────────────────────────────────────────────────────

    def snapshot(self) -> dict:
        link_count, dynamic, static = self.frame_counts
        age = self._joint_age_s()
        with self._vision_lock:
            vision_targets = len(self._vision)
        return {
            "state": "running" if self._running else "idle",
            "root_frame": self._tree["root"] if self._tree else "",
            "link_count": link_count,
            "dynamic_frames": dynamic,
            "static_frames": static,
            "tf_hz": round(self._tf_hz, 1),
            "last_joint_age_ms": int((age * 1000) if age is not None else -1),
            "vision_targets": vision_targets,
            "camera_extrinsic_assumed": self._camera_extrinsic_assumed,
            "degraded": self._degraded(),
            "last_error": self._parse_error or "",
        }


class TfTreePlugin:
    PREFIX = "tf_tree"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        cfg = plugin_config or {}
        urdf_path = str(cfg.get("urdf_path", _DEFAULT_URDF_PATH))
        publish_rate_hz = float(cfg.get("publish_rate_hz", 10.0))
        max_joint_age_s = float(cfg.get("max_joint_age_s", 0.5))
        vision_ttl_ms = int(cfg.get("vision_ttl_ms", 500))

        if not 1.0 <= publish_rate_hz <= 30.0:
            raise ValueError(f"publish_rate_hz must be in [1.0, 30.0], got {publish_rate_hz}")
        if not 0.1 <= max_joint_age_s <= 5.0:
            raise ValueError(f"max_joint_age_s must be in [0.1, 5.0], got {max_joint_age_s}")
        if not 0 <= vision_ttl_ms <= 60000:
            raise ValueError(f"vision_ttl_ms must be in [0, 60000], got {vision_ttl_ms}")

        self._node = _TfTreeNode(
            namespace, urdf_path, publish_rate_hz, max_joint_age_s, vision_ttl_ms)
        executor.add_node(self._node)

    def _topic_out(self) -> list:
        return [{"topic": self._node.status_topic, "format": "data/json"}]

    def get_tool(self) -> dict:
        return {
            "name": "tf_tree",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Bumi 实时 tf 树。吃 URDF + 关节状态 + 左联的视觉变换 JSON，"
                "在 /tf 与 /tf_static 上发布整棵坐标系树（根 base_link）。"
                "只发布 tf，不输出位姿数据、不驱动任何硬件、不发 odom→base_link。"
                "下游求相对位姿请自行用 tf2 订阅 /tf，不要读本卡的 status 话题"
                "（status 是树的健康度，不是位姿源）。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "卡片生命周期或实时状态查询",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": (
                            "可选：T2 vision_tf 卡输出的 data/json 视觉变换话题。"
                            "缺省时本卡仍能建树（只用 URDF + 关节状态）。"
                        ),
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["input_topic"],
                        "description": "订阅关节状态与（可选）视觉变换，开始发布 tf 树",
                    },
                    "stop": {"params": [], "description": "停止发布，/tf 不再有新样本"},
                    "info": {"params": [], "description": "树的健康度：帧数、发布率、关节数据年龄、降级原因"},
                },
            },
            "topic_in": [{"format": "data/json"}],
            "topic_out": self._topic_out(),
        }

    def start(self) -> None:
        # Start the structural tree (URDF + joints) even with no visual input:
        # `start_all()` calls this, and a robot with the vision card off still
        # needs /tf for its own links. dispatch("start") later attaches the
        # optional vision topic the canvas resolved.
        if not self._node.running:
            self._node.start("")

    def stop(self) -> None:
        self._node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "start":
            return self._start(args)
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            info = self._node.snapshot()
            info["topic_out"] = self._topic_out()
            return info
        return None

    def _start(self, args: dict) -> dict:
        input_topic = str(args.get("input_topic") or "").strip()

        if self._node.parse_error is not None:
            # actuator-style error: a broken URDF is a startup failure, not a
            # sensor "unavailable".
            return {"state": "error", "error": self._node.parse_error}

        self._node.start(input_topic)
        return {"state": "running"}
