"""Development harness; does not start the robot driver or issue motion commands."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

try:
    # This harness may be used with --serve in a driver container. Protect its
    # concurrent per-frame JSON output before any runtime output is produced.
    from common import logsafe
    logsafe.install()
except ImportError:
    # Local developer virtual environments need not have the repository common/
    # package on sys.path.
    pass

from pose_check import POSES, check_pose
from pose_estimator import MediaPipePoseEstimator


def draw_preview(frame, keypoints, result):
    """Draw on a copy so the preview never changes the inference input."""
    import cv2
    from pose_check import normalize_keypoints

    canvas = frame.copy()
    height, width = canvas.shape[:2]
    points = normalize_keypoints(keypoints)
    pixels = {}
    for name, (x, y, visibility) in points.items():
        if name not in MediaPipePoseEstimator.LANDMARKS:
            continue
        # Out-of-frame landmarks must not look like reliable points on an edge.
        if not (0 <= x < 1 and 0 <= y < 1):
            continue
        pixels[name] = (int(x * width), int(y * height))
    for a, b in (("left_shoulder", "right_shoulder"),
                 ("left_shoulder", "left_elbow"),
                 ("left_elbow", "left_wrist"),
                 ("right_shoulder", "right_elbow"),
                 ("right_elbow", "right_wrist"),
                 ("left_shoulder", "left_hip"), ("right_shoulder", "right_hip"),
                 ("left_hip", "right_hip"),
                 ("left_hip", "left_knee"), ("left_knee", "left_ankle"),
                 ("right_hip", "right_knee"), ("right_knee", "right_ankle")):
        if a in pixels and b in pixels:
            cv2.line(canvas, pixels[a], pixels[b], (180, 180, 180), 2)
    for name, pixel in pixels.items():
        visibility = points[name][2]
        color = (0, 220, 0) if visibility >= 0.5 else (0, 0, 255)
        cv2.circle(canvas, pixel, 6, color, -1)
        label = name.replace("left_", "L ").replace("right_", "R ")
        cv2.putText(canvas, f"{label} {visibility:.2f}",
                    (max(0, min(pixel[0] + 8, width - 170)), max(18, pixel[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    status = ("MATCH" if result.get("matched") else
              "NOT MATCHED" if result.get("detected") else "KEYPOINTS NOT RELIABLE")
    color = (0, 220, 0) if result.get("matched") else (0, 180, 255)
    if "session_id" in result:
        progress = result.get("progress", {})
        if progress.get("mode") == "repetitions":
            status = (f"{result['state']} / {progress.get('phase', '?')} / "
                      f"{progress.get('repetitions', 0)}/{progress.get('target_repetitions', '?')}")
        else:
            status = f"{result['state']} / {result['status']}"
        color = (0, 220, 0) if result["state"] == "completed" else (0, 180, 255)
    cv2.rectangle(canvas, (0, 0), (width, 65), (20, 20, 20), -1)
    cv2.putText(canvas, f"{result['pose']}: {status}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
    cv2.putText(canvas, "Green: reliable  Red: low confidence  Q/ESC: quit", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Pose Landmarker .task file")
    parser.add_argument("--source", default="0", help="Camera index, video file or stream URL")
    parser.add_argument("--image", help="Check a single image instead of opening a camera")
    parser.add_argument("--pose", choices=POSES, default="hands_up")
    parser.add_argument("--frames", type=int, default=0, help="0 means until Ctrl-C")
    parser.add_argument("--preview", action="store_true", help="Show camera and keypoints; Q/Esc to quit")
    parser.add_argument("--frame-independent", action="store_true",
                        help="Disable video tracking for comparison")
    parser.add_argument("--serve", action="store_true", help="Expose perception MCP on loopback")
    parser.add_argument("--port", type=int, default=15740)
    parser.add_argument("--practice", action="store_true", help="Start a timed practice session locally")
    parser.add_argument("--hold-seconds", type=float, default=3)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--timeout-seconds", type=float, default=45)
    args = parser.parse_args()
    if args.frames < 0:
        parser.error("--frames must be nonnegative")
    if args.preview and args.image:
        parser.error("--preview is for camera/video input; omit --image")
    if args.image and (args.serve or args.practice):
        parser.error("--serve / --practice requires a camera/video source")
    from pose_mcp import PoseCard, make_server
    from pose_session import PoseSession
    card = PoseCard()
    if args.practice:
        # Validate before loading the model; start the timer after camera startup.
        PoseSession(args.pose, args.hold_seconds, args.repetitions, args.timeout_seconds)

    import cv2
    estimator = MediaPipePoseEstimator(
        args.model, video=not args.image and not args.frame_independent)
    capture = None
    server = None
    server_thread = None
    try:
        if args.image:
            with open(args.image, "rb") as stream:
                print(json.dumps(check_pose(estimator.estimate(stream.read()), args.pose),
                                 ensure_ascii=False), flush=True)
            return 0
        source = int(args.source) if args.source.isdecimal() else args.source
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            raise RuntimeError("Cannot open camera/source; check camera permission and source")
        if args.practice:
            card.session = PoseSession(args.pose, args.hold_seconds, args.repetitions, args.timeout_seconds)
        if args.serve:
            server = make_server(card, port=args.port)
            server_thread = threading.Thread(target=server.serve_forever, daemon=True)
            server_thread.start()
            print(f"MCP: http://127.0.0.1:{args.port}/mcp", file=sys.stderr)
        count = 0
        while not args.frames or count < args.frames:
            ok, frame = capture.read()
            captured_at = time.monotonic()
            if not ok:
                raise RuntimeError("Camera/source returned no frame (or video ended)")
            ok, jpeg = cv2.imencode(".jpg", frame)
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            try:
                keypoints = estimator.estimate(jpeg.tobytes())
            except Exception:
                card.fail("model_failed")
                raise
            card.ingest(keypoints, captured_at)
            with card.lock:
                result = (card.session.status() if card.session else check_pose(keypoints, args.pose))
            print(json.dumps(result, ensure_ascii=False), flush=True)
            count += 1
            if args.preview:
                cv2.imshow("pose_check", draw_preview(frame, keypoints, result))
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break
                if cv2.getWindowProperty("pose_check", cv2.WND_PROP_VISIBLE) < 1:
                    break
    finally:
        card.fail("camera_stopped")
        if server:
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=2)
        if capture is not None:
            capture.release()
        estimator.close()
        if args.preview:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as exc:
        print(json.dumps({"error": "local_pose_failed", "detail": str(exc)},
                         ensure_ascii=False), file=sys.stderr)
        sys.exit(1)
