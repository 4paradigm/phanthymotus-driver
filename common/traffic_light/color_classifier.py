"""Emitting components, not box means; adapted from the read-only HSV prototype.

The original hue gates and red wrap live in ColorThresholds. Here each connected
component is checked separately, preserving simultaneous lamps instead of choosing
one dominant colour for the housing. Shape classification never uses lamp position.
"""
import math

import numpy as np

from .schemas import ColorEvidence, ColorThresholds, ShapeEvidence, bbox_of


_MIN_EDGE_GRADIENT = 160
_MAX_ARROW_SOLIDITY = .90


def _templates(cv2):
    mask = np.zeros((64, 64), np.uint8)
    points = np.array([[31, 0], [63, 29], [43, 29], [43, 63],
                       [20, 63], [20, 29], [0, 29]], np.int32)
    cv2.fillPoly(mask, [points], 1)
    return {"straight": mask, "left": np.rot90(mask), "right": np.rot90(mask, -1)}


def _symbol(mask, contour, cv2, min_size, edge_gradient):
    h, w = mask.shape
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    hull = cv2.contourArea(cv2.convexHull(contour))
    if not perimeter or not hull:
        return "unknown", 0., ShapeEvidence()
    circularity = 4 * math.pi * area / perimeter**2
    solidity = area / hull
    is_circle = .80 <= w/h <= 1.25 and circularity >= .80 and solidity >= .94
    normal = cv2.resize(mask, (64, 64), interpolation=cv2.INTER_NEAREST) > 0
    direction_scores = {}
    for direction, template in _templates(cv2).items():
        reference = template > 0
        score = np.count_nonzero(normal & reference) / max(1, np.count_nonzero(normal | reference))
        direction_scores[direction] = float(score)
    ranked = sorted(direction_scores, key=lambda direction: (direction_scores[direction], direction),
                    reverse=True)
    best = ranked[0]
    score = direction_scores[best]

    # Circle evidence rewards roundness, convexity and a square aspect ratio.
    # Arrow evidence combines the best template overlap with shaft/head concavity.
    circle_fit = min(1., circularity) * solidity * min(h, w) / max(h, w)
    arrow_fit = score * min(1., (1. - solidity) / (1. - _MAX_ARROW_SOLIDITY))
    silhouette = np.zeros_like(mask)
    x, y, _, _ = cv2.boundingRect(contour)
    cv2.drawContours(silhouette, [contour], -1, 1, cv2.FILLED, offset=(-x, -y))
    # Score the shape at the available resolution, with softer edges and holes
    # from missing or washed-out pixels lowering confidence for either type.
    coverage = min(1., cv2.countNonZero(mask) / max(1, cv2.countNonZero(silhouette)))
    size_quality = min(1., min(h, w) / max(1, min_size))
    edge_quality = min(1., edge_gradient / (2 * _MIN_EDGE_GRADIENT))
    quality = coverage * size_quality * edge_quality
    if arrow_fit > circle_fit:
        shape = ShapeEvidence("arrow", arrow_fit * quality, best, score, direction_scores)
    else:
        shape = ShapeEvidence("circle", circle_fit * quality, direction_scores=direction_scores)

    # Accept the legacy symbol only with the original geometry and image-quality gates.
    if min(h, w) >= min_size and edge_gradient >= _MIN_EDGE_GRADIENT:
        if is_circle:
            return "circle", round(min(.99, circularity), 3), shape
        if (.5 <= solidity <= _MAX_ARROW_SOLIDITY and score >= .82
                and score - direction_scores[ranked[1]] >= .15):
            return "arrow_" + best, round(score, 3), shape
    return "unknown", 0., shape


def classify_color(frame_bgr, bbox=None, thresholds=None):
    """Return independent colour/shape evidence and all emitting lamp regions."""
    import cv2

    limits = thresholds or ColorThresholds()
    if (not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8
            or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3):
        return ColorEvidence(reason="invalid_image")
    height, width = frame_bgr.shape[:2]
    box = bbox_of(bbox) if bbox is not None else [0, 0, width, height]
    if box is None:
        return ColorEvidence(reason="invalid_bbox")
    x1, y1, x2, y2 = box
    x1, x2 = max(0, min(width, int(x1))), max(0, min(width, int(math.ceil(x2))))
    y1, y2 = max(0, min(height, int(y1))), max(0, min(height, int(math.ceil(y2))))
    roi = frame_bgr[y1:y2, x1:x2]
    if not roi.size:
        return ColorEvidence(reason="empty_roi")
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    eligible = (s >= limits.min_saturation) & (v >= limits.min_value)
    masks = {
        "red": ((h <= limits.red_hue_low) | (h >= limits.red_hue_high)) & eligible,
        "yellow": (h >= limits.yellow_hue_low) & (h <= limits.yellow_hue_high) & eligible,
        "green": (h >= limits.green_hue_low) & (h <= limits.green_hue_high) & eligible,
    }
    bulbs = []
    gradient = None
    rh, rw = roi.shape[:2]
    for color, selected in masks.items():
        count, labels, stats, _ = cv2.connectedComponentsWithStats(selected.astype(np.uint8), 8)
        for index in range(1, min(count, 128)):
            x, y, w, hh, pixels = map(int, stats[index])
            if (pixels < limits.min_pixels or min(w, hh) < 4
                    or pixels > roi.shape[0]*roi.shape[1]*.5):
                continue
            # A clipped region cannot establish its luminous outline.
            if x <= 0 or y <= 0 or x+w >= rw or y+hh >= rh:
                continue
            component = (labels == index).astype(np.uint8)
            ring = cv2.dilate(component, np.ones((5, 5), np.uint8)) - component
            values = v[component > 0]
            background = v[ring > 0]
            if not len(background) or float(values.mean()) - float(np.median(background)) < 30:
                continue
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = max(contours, key=cv2.contourArea)
            local = component[y:y+hh, x:x+w]
            # A broad transition is weak shape evidence even if HSV still finds colour.
            if gradient is None:
                gradient = cv2.magnitude(cv2.Sobel(v, cv2.CV_32F, 1, 0),
                                         cv2.Sobel(v, cv2.CV_32F, 0, 1))
            edge = component - cv2.erode(component, np.ones((3, 3), np.uint8))
            edge_gradient = float(np.median(gradient[edge > 0])) if np.any(edge) else 0.
            symbol, symbol_confidence, shape = _symbol(local, contour, cv2, limits.min_symbol_size,
                                                       edge_gradient)
            confidence = min(.99, .5 + .25*float(s[component > 0].mean())/255
                             + .25*float(values.mean())/255)
            bulbs.append({"bbox": [x1+x, y1+y, x1+x+w, y1+y+hh], "color": color,
                          "color_confidence": round(confidence, 3), "symbol": symbol,
                          "symbol_confidence": symbol_confidence, "pixels": pixels,
                          **shape.to_dict()})
    bulbs.sort(key=lambda b: (b["bbox"][1], b["bbox"][0]))
    bulbs = bulbs[:16]
    if not bulbs:
        return ColorEvidence()
    colors = {b["color"] for b in bulbs}
    color = next(iter(colors)) if len(colors) == 1 else "unknown"
    symbol = bulbs[0]["symbol"] if len(bulbs) == 1 else "unknown"
    return ColorEvidence(color=color,
                         color_confidence=min(b["color_confidence"] for b in bulbs) if color != "unknown" else 0.,
                         symbol=symbol, symbol_confidence=bulbs[0]["symbol_confidence"] if len(bulbs) == 1 else 0.,
                         bulbs=bulbs, reason="ambiguous_colors" if len(colors) > 1 else "emitting_regions")
