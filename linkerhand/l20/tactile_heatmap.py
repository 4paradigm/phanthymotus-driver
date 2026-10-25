"""Tactile Matrix Heatmap Renderer for LinkerHand L20.

Vectorized rendering of 5-finger 12x6 tactile matrix arrays into a sleek,
high-contrast JPEG stream for real-time visualization on the Phanthymotus
Web Dashboard.
"""

from __future__ import annotations

import io
from typing import Any, Dict, Optional
import numpy as np
from PIL import Image, ImageDraw


# 256-level Plasma/Turbo Colormap LUT (RGB uint8)
def _build_colormap() -> np.ndarray:
    stops = [
        (0.00, (26, 34, 52)),      # 0: Slate dark navy (idle/no contact)
        (0.05, (20, 60, 110)),     # 12: Deep navy blue
        (0.15, (14, 116, 144)),    # 38: Cyan / Teal
        (0.35, (16, 185, 129)),    # 89: Emerald green
        (0.55, (234, 179, 8)),     # 140: Amber / Yellow
        (0.75, (249, 115, 22)),    # 191: Orange
        (0.90, (239, 68, 68)),     # 230: Red
        (1.00, (244, 63, 94)),     # 255: Hot Rose / Magenta
    ]
    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        t = i / 255.0
        for k in range(len(stops) - 1):
            t0, c0 = stops[k]
            t1, c1 = stops[k + 1]
            if t0 <= t <= t1:
                ratio = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                lut[i] = [
                    int(c0[0] + (c1[0] - c0[0]) * ratio),
                    int(c0[1] + (c1[1] - c0[1]) * ratio),
                    int(c0[2] + (c1[2] - c0[2]) * ratio),
                ]
                break
        else:
            lut[i] = stops[-1][1]
    return lut


COLORMAP_LUT = _build_colormap()
FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]
FINGER_NAMES_DISPLAY = {
    "thumb": "THUMB",
    "index": "INDEX",
    "middle": "MID",
    "ring": "RING",
    "pinky": "PINKY",
}


def render_heatmap_jpeg(
    matrix_data: Optional[dict[str, Any]] = None,
    scalar_data: Optional[dict[str, Any]] = None,
    side: str = "left",
    quality: int = 80,
) -> bytes:
    """Render 5-finger 12x6 tactile matrix arrays into a JPEG buffer."""
    width, height = 660, 330
    # Canvas with dark slate background (15, 23, 42)
    canvas = np.full((height, width, 3), (15, 23, 42), dtype=np.uint8)

    matrix_data = matrix_data or {}
    scalar_data = scalar_data or {}

    cell_size = 14
    cols, rows = 6, 12
    block_w = cols * cell_size  # 84 px
    block_h = rows * cell_size  # 168 px
    start_x = 24
    start_y = 50
    spacing = 108

    peaks = {}
    missing_matrix = set()

    for idx, finger in enumerate(FINGER_ORDER):
        bx = start_x + idx * spacing
        by = start_y

        # Extract 12x6 array
        f_raw = matrix_data.get(finger)
        if not (isinstance(f_raw, list) and len(f_raw) == rows and
                all(isinstance(row, list) and len(row) == cols for row in f_raw)):
            missing_matrix.add(finger)
        m_arr = np.zeros((rows, cols), dtype=np.uint8)
        if f_raw and isinstance(f_raw, list):
            for r in range(min(rows, len(f_raw))):
                row = f_raw[r]
                if isinstance(row, list):
                    for c in range(min(cols, len(row))):
                        val = row[c]
                        if isinstance(val, (int, float)) and val is not None:
                            m_arr[r, c] = max(0, min(255, int(val)))

        peaks[finger] = int(m_arr.max())

        # Map through colormap LUT -> (12, 6, 3)
        rgb_block = COLORMAP_LUT[m_arr]

        # Vectorized upscale by cell_size
        upscaled = np.repeat(np.repeat(rgb_block, cell_size, axis=0), cell_size, axis=1)

        # Add 1px dark grid lines
        upscaled[::cell_size, :, :] = (10, 15, 30)
        upscaled[:, ::cell_size, :] = (10, 15, 30)
        upscaled[-1, :, :] = (10, 15, 30)
        upscaled[:, -1, :] = (10, 15, 30)

        canvas[by : by + block_h, bx : bx + block_w] = upscaled

    # Convert to PIL Image for labels and text
    img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(img)

    # 1. Header
    title = f"LINKERHAND L20 · TACTILE MATRIX ({side.upper()})"
    draw.text((20, 14), title, fill=(56, 189, 248))  # Sky Blue
    missing_scalar = any(scalar_data.get(finger) is None for finger in FINGER_ORDER)
    status = "NO DATA" if len(missing_matrix) == 5 and missing_scalar else "PARTIAL DATA" if missing_matrix or missing_scalar else "LIVE STREAM"
    draw.text((545, 14), status, fill=(251, 146, 60) if status != "LIVE STREAM" else (16, 185, 129))

    # 2. Labels and Scalar Bar per finger
    for idx, finger in enumerate(FINGER_ORDER):
        bx = start_x + idx * spacing
        by = start_y
        label = FINGER_NAMES_DISPLAY.get(finger, finger.upper())
        peak = peaks.get(finger, 0)
        peak_rgb = tuple(int(c) for c in COLORMAP_LUT[min(255, peak)]) if peak > 0 else (148, 163, 184)

        # Name & Peak
        draw.text((bx + 2, by - 18), label, fill=(226, 232, 240))
        draw.text((bx + 54, by - 18), " --" if finger in missing_matrix else f"{peak:3d}", fill=peak_rgb)
        if finger in missing_matrix:
            draw.rectangle([bx, by, bx + block_w - 1, by + block_h - 1], fill=(30, 41, 59))
            draw.text((bx + 12, by + block_h // 2), "NO DATA", fill=(251, 146, 60))

        # Scalar Tip Pressure Bar
        s_val = scalar_data.get(finger)
        bar_y = by + block_h + 10
        draw.rectangle([bx, bar_y, bx + block_w, bar_y + 5], fill=(30, 41, 59))
        if s_val is not None and isinstance(s_val, (int, float)) and s_val > 0:
            s_clamp = max(0, min(255, int(s_val)))
            fill_w = int((s_clamp / 255.0) * block_w)
            if fill_w > 0:
                bar_color = tuple(int(c) for c in COLORMAP_LUT[s_clamp])
                draw.rectangle([bx, bar_y, bx + fill_w, bar_y + 5], fill=bar_color)
            draw.text((bx + 2, bar_y + 9), f"Tip: {s_clamp}", fill=(148, 163, 184))
        else:
            draw.text((bx + 2, bar_y + 9), "Tip: --" if s_val is None else "Tip: 0", fill=(100, 116, 139))

    # 3. Color Scale Legend on the right
    leg_x = start_x + 5 * spacing + 10
    leg_y = start_y
    leg_h = block_h
    leg_w = 12

    for y in range(leg_h):
        ratio = 1.0 - (y / float(leg_h))
        lut_idx = min(255, max(0, int(ratio * 255.0)))
        c = tuple(int(v) for v in COLORMAP_LUT[lut_idx])
        draw.line([(leg_x, leg_y + y), (leg_x + leg_w - 1, leg_y + y)], fill=c)

    draw.rectangle([leg_x, leg_y, leg_x + leg_w - 1, leg_y + leg_h - 1], outline=(71, 85, 105))
    draw.text((leg_x + leg_w + 6, leg_y - 3), "255", fill=(244, 63, 94))
    draw.text((leg_x + leg_w + 6, leg_y + leg_h // 2 - 5), "128", fill=(234, 179, 8))
    draw.text((leg_x + leg_w + 6, leg_y + leg_h - 8), "0", fill=(100, 116, 139))

    # 4. Footer
    draw.line([(20, height - 28), (width - 20, height - 28)], fill=(30, 41, 59))
    draw.text((20, height - 20), "12×6 Taxels / Finger (360 total)", fill=(100, 116, 139))
    draw.text((450, height - 20), "Matrix Pressure Range: 0–255", fill=(100, 116, 139))

    # Encode to JPEG
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()
