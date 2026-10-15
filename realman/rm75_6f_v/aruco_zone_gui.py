#!/usr/bin/env python3
"""Visual ArUco zone setup for RealMan. This program never controls the arm."""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk


ROLES = {"waiting": "待分拣区", "sorting_1": "分拣区 1", "sorting_2": "分拣区 2"}
DRAW_COLORS = {"waiting": "#e0b000", "sorting_1": "#20a060", "sorting_2": "#287de0"}
DICTIONARIES = ("DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50")
CANVAS_WIDTH, CANVAS_HEIGHT = 960, 600


def detect_markers(image, cv2, dictionary_name):
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        raise RuntimeError("当前 OpenCV 没有 ArUco 模块；请安装 opencv-contrib-python")
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    markers = []
    if ids is not None:
        for marker_id, quad in zip(ids.flatten().tolist(), corners):
            points = [[float(x), float(y)] for x, y in quad.reshape(4, 2)]
            markers.append({"id": int(marker_id), "corners_px": points,
                            "center_px": [sum(p[0] for p in points) / 4,
                                          sum(p[1] for p in points) / 4]})
    return markers


def marker_in_rect(marker, rect, width, height):
    x, y = marker["center_px"]
    return rect[0] <= x / width <= rect[2] and rect[1] <= y / height <= rect[3]


def rectangles_overlap(a, b):
    return max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])


def image_transform(width, height):
    scale = min(CANVAS_WIDTH / width, CANVAS_HEIGHT / height)
    shown_width, shown_height = round(width * scale), round(height * scale)
    return scale, (CANVAS_WIDTH - shown_width) // 2, (CANVAS_HEIGHT - shown_height) // 2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="先用静态照片规划区域")
    source.add_argument("--camera", action="store_true", help="直接读取上位机 RealSense 实时画面")
    parser.add_argument("--serial", help="RealSense 序列号；多台相机时必须指定")
    parser.add_argument("--output", type=Path, default=Path("aruco_zones.json"))
    return parser.parse_args()


class CameraWorker(threading.Thread):
    def __init__(self, serial, frame_queue):
        super().__init__(name="aruco-zone-camera", daemon=True)
        self.serial = serial
        self.frame_queue = frame_queue
        self.stop_requested = threading.Event()

    def _put(self, kind, value):
        try:
            self.frame_queue.put_nowait((kind, value))
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait((kind, value))

    def run(self):
        pipeline = None
        try:
            import numpy as np
            import pyrealsense2 as rs

            devices = list(rs.context().query_devices())
            serials = [device.get_info(rs.camera_info.serial_number) for device in devices]
            if self.serial:
                if self.serial not in serials:
                    raise RuntimeError(f"未找到序列号为 {self.serial} 的 RealSense")
                serial = self.serial
            elif len(serials) == 1:
                serial = serials[0]
            else:
                raise RuntimeError("请用 --serial 指定唯一的 RealSense 相机")
            device = devices[serials.index(serial)]
            usb = device.get_info(rs.camera_info.usb_type_descriptor)
            usb3 = usb.startswith("3")
            width, height, fps = (1280, 720, 15) if usb3 else (640, 480, 6)
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            pipeline = rs.pipeline()
            pipeline.start(config)
            self._put("status", f"相机 {serial} 已启动：{width}×{height} @ {fps} fps")
            while not self.stop_requested.is_set():
                frames = pipeline.wait_for_frames(1000)
                frame = frames.get_color_frame()
                if frame:
                    self._put("frame", np.asanyarray(frame.get_data()).copy())
        except Exception as exc:
            self._put("error", str(exc))
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass


class ZoneApp:
    def __init__(self, root, args, cv2):
        self.root, self.args, self.cv2 = root, args, cv2
        self.root.title("RealMan ArUco 区域规划（仅视觉）")
        self.frame_queue = queue.Queue(maxsize=2)
        self.worker = None
        self.image = None
        self.markers = []
        self._last_detected_ids = None
        self.zones = {}
        self.drag_start = None
        self.photo = None
        self.role = tk.StringVar(value="waiting")
        self.dictionary = tk.StringVar(value="DICT_4X4_50")
        self.status = tk.StringVar(value="在画面上拖拽划定区域；ArUco ID 自动按码中心绑定。")
        self.colors = {"sorting_1": tk.StringVar(value="red"),
                       "sorting_2": tk.StringVar(value="blue")}
        self._build_ui()
        if args.image:
            image = cv2.imread(str(args.image))
            if image is None:
                raise RuntimeError(f"无法读取照片：{args.image}")
            self.image = image
        else:
            self.worker = CameraWorker(args.serial, self.frame_queue)
            self.worker.start()
        self.root.after(40, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self):
        toolbar = ttk.Frame(self.root, padding=8)
        toolbar.pack(fill="x")
        for role, title in ROLES.items():
            ttk.Radiobutton(toolbar, text=title, variable=self.role, value=role).pack(side="left", padx=5)
        ttk.Label(toolbar, text="码字典").pack(side="left", padx=(20, 4))
        ttk.Combobox(toolbar, textvariable=self.dictionary, values=DICTIONARIES,
                     state="readonly", width=14).pack(side="left")
        ttk.Button(toolbar, text="清除当前区域", command=self._clear).pack(side="right", padx=4)
        ttk.Button(toolbar, text="保存区域", command=self._save).pack(side="right", padx=4)

        self.canvas = tk.Canvas(self.root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
                                bg="#222222", highlightthickness=0)
        self.canvas.pack(padx=8, pady=4)
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._drag)
        self.canvas.bind("<ButtonRelease-1>", self._release)

        footer = ttk.Frame(self.root, padding=8)
        footer.pack(fill="x")
        for role in ("sorting_1", "sorting_2"):
            ttk.Label(footer, text=f"{ROLES[role]} 代表颜色").pack(side="left", padx=(8, 4))
            ttk.Combobox(footer, textvariable=self.colors[role],
                         values=("red", "green", "blue", "yellow"),
                         state="readonly", width=9).pack(side="left", padx=(0, 14))
        ttk.Label(self.root, textvariable=self.status, padding=8).pack(fill="x")

    def _position(self, event):
        if self.image is None:
            return None
        height, width = self.image.shape[:2]
        scale, offset_x, offset_y = image_transform(width, height)
        x = (event.x - offset_x) / (width * scale)
        y = (event.y - offset_y) / (height * scale)
        if not 0 <= x <= 1 or not 0 <= y <= 1:
            return None
        return (x, y)

    def _press(self, event):
        self.drag_start = self._position(event)

    def _drag(self, event):
        if self.drag_start is None or self.image is None:
            return
        self._draw()
        pos = self._position(event)
        if pos is None:
            return
        height, width = self.image.shape[:2]
        scale, ox, oy = image_transform(width, height)
        x1, y1 = self.drag_start
        x2, y2 = pos
        self.canvas.create_rectangle(ox + x1 * width * scale, oy + y1 * height * scale,
                                     ox + x2 * width * scale, oy + y2 * height * scale,
                                     outline=DRAW_COLORS[self.role.get()], width=2, dash=(5, 3),
                                     tags="drag_preview")

    def _release(self, event):
        end = self._position(event)
        start, self.drag_start = self.drag_start, None
        if start is None or end is None:
            return
        x1, x2 = sorted((start[0], end[0]))
        y1, y2 = sorted((start[1], end[1]))
        if (x2 - x1) * (y2 - y1) < 0.001:
            self.status.set("区域太小；请拖出更大的矩形。")
            return
        self.zones[self.role.get()] = [x1, y1, x2, y2]
        self._draw()

    def _clear(self):
        self.zones.pop(self.role.get(), None)
        self._draw()

    def _zone_data(self):
        if self.image is None:
            return {}
        height, width = self.image.shape[:2]
        result = {}
        for role, rect in self.zones.items():
            contained = [marker["id"] for marker in self.markers
                         if marker_in_rect(marker, rect, width, height)]
            marker_id = contained[0] if len(contained) == 1 else None
            result[role] = {"label": ROLES[role], "rect_normalized": [round(v, 6) for v in rect],
                            "rect_px": [round(rect[0] * width), round(rect[1] * height),
                                        round(rect[2] * width), round(rect[3] * height)],
                            "marker_id": marker_id,
                            "marker_ids_in_region": contained,
                            "color": self.colors[role].get() if role in self.colors else None}
        return result

    def _save(self):
        import numpy as np

        if self.image is None:
            self.status.set("尚无相机画面。")
            return
        if len(self.zones) != 3:
            self.status.set("请先框出待分拣区、分拣区 1 和分拣区 2。")
            return
        zone_rects = list(self.zones.items())
        if any(rectangles_overlap(rect, other) for index, (_, rect) in enumerate(zone_rects)
               for _, other in zone_rects[index + 1:]):
            self.status.set("三个区域不能重叠，请重新框选。")
            return
        zones = self._zone_data()
        if any(zone["marker_id"] is None for zone in zones.values()):
            self.status.set("每个区域需恰好包含一个可见 ArUco 码中心。")
            return
        ids = [zone["marker_id"] for zone in zones.values()]
        if len(set(ids)) != 3:
            self.status.set("三个区域必须对应不同的 ArUco ID。")
            return
        height, width = self.image.shape[:2]
        payload = {"version": 1, "dictionary": self.dictionary.get(),
                   "image_width": width, "image_height": height,
                   "coordinate_system": "image_top_left_normalized_0_to_1",
                   "zones": zones, "motion_enabled": False}
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                                    encoding="utf-8")
        preview = self.image.copy()
        for marker in self.markers:
            quad = np.rint(marker["corners_px"]).astype(np.int32)
            self.cv2.polylines(preview, [quad], True, (0, 0, 255), 2)
            cx, cy = map(round, marker["center_px"])
            self.cv2.putText(preview, f"ID {marker['id']}", (cx, cy - 12),
                             self.cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        bgr_colors = {"waiting": (0, 220, 255), "sorting_1": (60, 180, 50),
                      "sorting_2": (240, 120, 30)}
        for role, zone in zones.items():
            x1, y1, x2, y2 = zone["rect_px"]
            color = bgr_colors[role]
            self.cv2.rectangle(preview, (x1, y1), (x2, y2), color, 3)
            self.cv2.putText(preview, f"{role} / ID {zone['marker_id']}",
                             (x1 + 4, y1 + 25), self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        preview_path = self.args.output.with_name(self.args.output.stem + "_preview.jpg")
        if not self.cv2.imwrite(str(preview_path), preview):
            self.status.set(f"JSON 已保存，但预览图保存失败：{preview_path}")
            return
        self.status.set(f"区域配置已保存：{self.args.output}；预览图：{preview_path}")
        messagebox.showinfo("已保存", str(self.args.output))

    def _draw(self):
        if self.image is None:
            return
        height, width = self.image.shape[:2]
        scale, ox, oy = image_transform(width, height)
        shown = self.cv2.resize(self.image, (round(width * scale), round(height * scale)))
        success, png = self.cv2.imencode(".png", shown)
        if not success:
            return
        self.photo = tk.PhotoImage(data=base64.b64encode(png.tobytes()).decode("ascii"), format="png")
        self.canvas.delete("all")
        self.canvas.create_image(ox, oy, anchor="nw", image=self.photo)
        for marker in self.markers:
            pts = [coord for x, y in marker["corners_px"]
                   for coord in (ox + x * scale, oy + y * scale)]
            self.canvas.create_polygon(pts, outline="#ff5050", fill="", width=2)
            cx, cy = marker["center_px"]
            self.canvas.create_text(ox + cx * scale, oy + cy * scale - 15,
                                    text=f"ID {marker['id']}", fill="#ff5050")
        zones = self._zone_data()
        for role, zone in zones.items():
            x1, y1, x2, y2 = zone["rect_px"]
            coords = (ox + x1 * scale, oy + y1 * scale, ox + x2 * scale, oy + y2 * scale)
            self.canvas.create_rectangle(*coords, outline=DRAW_COLORS[role], width=3)
            suffix = f" · ID {zone['marker_id']}" if zone["marker_id"] is not None else " · 未绑定码"
            self.canvas.create_text(coords[0] + 5, coords[1] + 5, anchor="nw",
                                    text=ROLES[role] + suffix, fill=DRAW_COLORS[role])

    def _poll(self):
        try:
            while True:
                kind, value = self.frame_queue.get_nowait()
                if kind == "frame":
                    self.image = value
                elif kind == "error":
                    self.status.set("相机错误：" + value)
                    self.worker = None
                elif kind == "status":
                    self.status.set(value)
        except queue.Empty:
            pass
        if self.image is not None:
            try:
                self.markers = detect_markers(self.image, self.cv2, self.dictionary.get())
                ids = tuple(sorted(marker["id"] for marker in self.markers))
                if ids != self._last_detected_ids:
                    self.status.set("检测到 ArUco ID：" + (", ".join(map(str, ids)) if ids else "无"))
                    self._last_detected_ids = ids
                self._draw()
            except Exception as exc:
                self.status.set(f"ArUco 识别失败：{exc}")
        self.root.after(120, self._poll)

    def _close(self):
        if self.worker is not None:
            self.worker.stop_requested.set()
        self.root.destroy()


def main():
    args = parse_args()
    try:
        import cv2
        if getattr(cv2, "aruco", None) is None:
            raise RuntimeError("OpenCV 缺少 ArUco 模块，请安装 opencv-contrib-python")
        root = tk.Tk()
        ZoneApp(root, args, cv2)
        root.mainloop()
        return 0
    except (ImportError, OSError, RuntimeError, tk.TclError) as exc:
        print(f"无法启动 ArUco 区域规划：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
