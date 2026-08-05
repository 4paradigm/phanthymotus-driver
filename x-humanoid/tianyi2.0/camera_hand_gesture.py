#!/usr/bin/env python3
"""OpenCV-DNN hand landmark inference and rock-paper-scissors classification.

The palm detector and hand-pose networks are the Apache-2.0 licensed
MediaPipe conversions published by OpenCV Zoo.  The compact preprocessing and
postprocessing below follows the corresponding OpenCV Zoo Python examples,
while generating the SSD anchors instead of storing a large literal table.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2 as cv
import numpy as np


@dataclass(frozen=True)
class HandObservation:
    """One detected hand and its rock-paper-scissors classification."""

    gesture: str
    confidence: float
    handedness: str
    handedness_confidence: float
    bbox: list[int]
    landmarks: list[list[float]]
    extended_fingers: dict[str, bool]
    finger_scores: dict[str, float]
    palm_confidence: float
    landmark_confidence: float


class PalmDetector:
    """MediaPipe palm detector converted to ONNX by OpenCV Zoo."""

    _INPUT_SIZE = np.array([192, 192], dtype=np.float32)  # width, height

    def __init__(self, model_path: str, score_threshold: float = 0.60,
                 nms_threshold: float = 0.30):
        self._score_threshold = float(score_threshold)
        self._nms_threshold = float(nms_threshold)
        self._net = cv.dnn.readNet(model_path)
        self._anchors = self._generate_anchors()

    @staticmethod
    def _generate_anchors() -> np.ndarray:
        # MediaPipe palm detector: 24x24 with 2 anchors per cell followed by
        # 12x12 with 6 anchors per cell (2016 anchors in total).
        anchors: list[tuple[float, float]] = []
        for grid_size, repeats in ((24, 2), (12, 6)):
            for y in range(grid_size):
                for x in range(grid_size):
                    center = ((x + 0.5) / grid_size, (y + 0.5) / grid_size)
                    anchors.extend([center] * repeats)
        return np.asarray(anchors, dtype=np.float32)

    def infer(self, image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        scale = min(192.0 / height, 192.0 / width)
        resized_w = max(1, int(width * scale))
        resized_h = max(1, int(height * scale))
        resized = cv.resize(image, (resized_w, resized_h), interpolation=cv.INTER_AREA)
        pad_w, pad_h = 192 - resized_w, 192 - resized_h
        left, top = pad_w // 2, pad_h // 2
        padded = cv.copyMakeBorder(
            resized, top, pad_h - top, left, pad_w - left,
            cv.BORDER_CONSTANT, value=(0, 0, 0))
        blob = cv.cvtColor(padded, cv.COLOR_BGR2RGB).astype(np.float32) / 255.0

        self._net.setInput(blob[np.newaxis, ...])
        outputs = self._net.forward(self._net.getUnconnectedOutLayersNames())
        box_output, score_output = self._identify_outputs(outputs)

        scores = score_output[0, :, 0].astype(np.float64)
        scores = 1.0 / (1.0 + np.exp(-np.clip(scores, -100.0, 100.0)))
        deltas = box_output[0]
        image_scale = max(width, height)
        centers = deltas[:, :2] / self._INPUT_SIZE + self._anchors
        sizes = deltas[:, 2:4] / self._INPUT_SIZE
        boxes = np.concatenate((centers - sizes / 2, centers + sizes / 2), axis=1)
        boxes *= image_scale
        pad_bias = np.array([left / scale, top / scale] * 2, dtype=np.float32)
        boxes -= pad_bias

        # OpenCV NMSBoxes expects [x, y, width, height], not xyxy.
        nms_boxes = np.c_[boxes[:, :2], boxes[:, 2:] - boxes[:, :2]].tolist()
        keep = cv.dnn.NMSBoxes(
            nms_boxes, scores.tolist(), self._score_threshold,
            self._nms_threshold, top_k=20)
        if len(keep) == 0:
            return np.empty((0, 19), dtype=np.float32)
        keep = np.asarray(keep).reshape(-1)

        landmarks = deltas[keep, 4:].reshape(-1, 7, 2)
        landmarks = landmarks / self._INPUT_SIZE + self._anchors[keep, np.newaxis, :]
        landmarks *= image_scale
        landmarks -= np.array([left / scale, top / scale], dtype=np.float32)
        return np.c_[boxes[keep], landmarks.reshape(-1, 14), scores[keep]].astype(np.float32)

    @staticmethod
    def _identify_outputs(outputs: tuple[np.ndarray, ...] | list[np.ndarray]):
        box_output = next((out for out in outputs if out.shape[-1] == 18), None)
        score_output = next((out for out in outputs if out.shape[-1] == 1), None)
        if box_output is None or score_output is None:
            shapes = [tuple(out.shape) for out in outputs]
            raise RuntimeError(f"unexpected palm detector outputs: {shapes}")
        return box_output, score_output


class HandLandmarker:
    """MediaPipe 21-point hand-pose network converted by OpenCV Zoo."""

    _INPUT_SIZE = np.array([224, 224], dtype=np.float32)
    _PALM_BASE = 0
    _MIDDLE_BASE = 2

    def __init__(self, model_path: str, confidence_threshold: float = 0.60):
        self._confidence_threshold = float(confidence_threshold)
        self._net = cv.dnn.readNet(model_path)

    @staticmethod
    def _crop_and_pad(image: np.ndarray, bbox: np.ndarray,
                      shift: tuple[float, float], enlarge: float,
                      diagonal: bool = False):
        bbox = bbox.astype(np.float32).copy()
        size = bbox[1] - bbox[0]
        bbox += np.asarray(shift, dtype=np.float32) * size
        center = bbox.mean(axis=0)
        bbox = np.asarray([center - size * enlarge / 2, center + size * enlarge / 2])
        bbox = bbox.astype(np.int32)
        bbox[:, 0] = np.clip(bbox[:, 0], 0, image.shape[1])
        bbox[:, 1] = np.clip(bbox[:, 1], 0, image.shape[0])
        crop = image[bbox[0, 1]:bbox[1, 1], bbox[0, 0]:bbox[1, 0]]
        if crop.size == 0:
            raise ValueError("empty hand crop")
        side = int(np.linalg.norm(crop.shape[:2]) if diagonal else max(crop.shape[:2]))
        pad_h, pad_w = side - crop.shape[0], side - crop.shape[1]
        left, top = pad_w // 2, pad_h // 2
        crop = cv.copyMakeBorder(
            crop, top, pad_h - top, left, pad_w - left,
            cv.BORDER_CONSTANT, value=(0, 0, 0))
        bias = bbox[0] - np.array([left, top])
        return crop, bbox, bias

    def infer(self, image: np.ndarray, palm: np.ndarray):
        palm_bbox = palm[:4].reshape(2, 2)
        crop, palm_bbox, pad_bias = self._crop_and_pad(
            image, palm_bbox, (0.0, 0.0), 4.0, diagonal=True)
        crop = cv.cvtColor(crop, cv.COLOR_BGR2RGB)

        local_bbox = palm_bbox - pad_bias
        palm_landmarks = palm[4:18].reshape(7, 2) - pad_bias
        p1 = palm_landmarks[self._PALM_BASE]
        p2 = palm_landmarks[self._MIDDLE_BASE]
        radians = math.pi / 2 - math.atan2(-(p2[1] - p1[1]), p2[0] - p1[0])
        radians -= 2 * math.pi * math.floor((radians + math.pi) / (2 * math.pi))
        angle = math.degrees(radians)
        center = local_bbox.mean(axis=0)
        rotation = cv.getRotationMatrix2D(tuple(center), angle, 1.0)
        rotated = cv.warpAffine(crop, rotation, (crop.shape[1], crop.shape[0]))

        homogeneous = np.c_[palm_landmarks, np.ones(7)]
        rotated_palm = np.c_[homogeneous @ rotation[0], homogeneous @ rotation[1]]
        rotated_bbox = np.asarray([rotated_palm.min(axis=0), rotated_palm.max(axis=0)])
        hand_crop, rotated_bbox, _ = self._crop_and_pad(
            rotated, rotated_bbox, (0.0, -0.4), 3.0)
        blob = cv.resize(hand_crop, (224, 224), interpolation=cv.INTER_AREA)
        blob = blob.astype(np.float32) / 255.0

        self._net.setInput(blob[np.newaxis, ...])
        outputs = self._net.forward(self._net.getUnconnectedOutLayersNames())
        landmark_blob, conf_blob, handedness_blob, world_blob = self._identify_outputs(outputs)
        confidence = float(conf_blob.reshape(-1)[0])
        if confidence < self._confidence_threshold:
            return None

        landmarks = landmark_blob.reshape(21, 3).astype(np.float32)
        world = world_blob.reshape(21, 3).astype(np.float32)
        size = rotated_bbox[1] - rotated_bbox[0]
        scale = float(max(size / self._INPUT_SIZE))
        landmarks[:, :2] = (landmarks[:, :2] - self._INPUT_SIZE / 2) * scale
        landmarks[:, 2] *= scale

        derotate = cv.getRotationMatrix2D((0, 0), angle, 1.0)[:, :2]
        rotated_xy = landmarks[:, :2] @ derotate.T
        rotated_world_xy = world[:, :2] @ derotate.T

        rotation_component = np.array([
            [rotation[0, 0], rotation[1, 0]],
            [rotation[0, 1], rotation[1, 1]],
        ])
        translation = rotation[:, 2]
        inverse_translation = -rotation_component @ translation
        inverse_rotation = np.c_[rotation_component, inverse_translation]
        hand_center = np.r_[rotated_bbox.mean(axis=0), 1.0]
        original_center = inverse_rotation @ hand_center
        landmarks[:, :2] = rotated_xy + original_center + pad_bias
        world[:, :2] = rotated_world_xy

        bbox = np.asarray([landmarks[:, :2].min(axis=0), landmarks[:, :2].max(axis=0)])
        bbox_size = bbox[1] - bbox[0]
        bbox += np.asarray([0.0, -0.1]) * bbox_size
        bbox_center = bbox.mean(axis=0)
        bbox = np.asarray([bbox_center - bbox_size * 1.65 / 2,
                           bbox_center + bbox_size * 1.65 / 2])
        handedness = float(handedness_blob.reshape(-1)[0])
        return bbox, landmarks, world, handedness, confidence

    @staticmethod
    def _identify_outputs(outputs: tuple[np.ndarray, ...] | list[np.ndarray]):
        # OpenCV Zoo's exported order is screen landmarks, hand-presence
        # confidence, handedness probability, and world landmarks.
        if (len(outputs) != 4 or outputs[0].size != 63 or outputs[1].size != 1
                or outputs[2].size != 1 or outputs[3].size != 63):
            shapes = [tuple(out.shape) for out in outputs]
            raise RuntimeError(f"unexpected hand landmarker outputs: {shapes}")
        return outputs[0], outputs[1], outputs[2], outputs[3]


class RockPaperScissorsRecognizer:
    """Detect one primary hand and classify only rock, paper, or scissors."""

    _FINGERS = {
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "little": (17, 18, 19, 20),
    }

    def __init__(self, palm_model_path: str, hand_model_path: str,
                 palm_threshold: float = 0.60,
                 landmark_threshold: float = 0.60,
                 gesture_threshold: float = 0.68):
        self._palm = PalmDetector(palm_model_path, palm_threshold)
        self._hand = HandLandmarker(hand_model_path, landmark_threshold)
        self._gesture_threshold = float(gesture_threshold)

    def infer(self, image: np.ndarray):
        palms = self._palm.infer(image)
        if len(palms) == 0:
            return None, 0

        observations: list[HandObservation] = []
        for palm in palms[:2]:
            result = self._hand.infer(image, palm)
            if result is None:
                continue
            bbox, landmarks, world, handedness_value, landmark_confidence = result
            gesture, gesture_confidence, states, scores = self._classify(world)
            height, width = image.shape[:2]
            bbox[:, 0] = np.clip(bbox[:, 0], 0, width - 1)
            bbox[:, 1] = np.clip(bbox[:, 1], 0, height - 1)
            handedness = "right" if handedness_value >= 0.5 else "left"
            handedness_confidence = handedness_value if handedness == "right" else 1.0 - handedness_value
            observations.append(HandObservation(
                gesture=gesture,
                confidence=round(gesture_confidence * float(landmark_confidence), 4),
                handedness=handedness,
                handedness_confidence=round(float(handedness_confidence), 4),
                bbox=[int(v) for v in bbox.reshape(-1)],
                landmarks=[[round(float(x), 2), round(float(y), 2), round(float(z), 4)]
                           for x, y, z in landmarks],
                extended_fingers=states,
                finger_scores={key: round(value, 3) for key, value in scores.items()},
                palm_confidence=round(float(palm[-1]), 4),
                landmark_confidence=round(float(landmark_confidence), 4),
            ))

        if not observations:
            return None, len(palms)
        primary = max(
            observations,
            key=lambda item: max(0, item.bbox[2] - item.bbox[0])
            * max(0, item.bbox[3] - item.bbox[1]))
        return primary, len(observations)

    def _classify(self, landmarks: np.ndarray):
        scores = {}
        states = {}
        wrist = landmarks[0]
        for name, (mcp, pip, dip, tip) in self._FINGERS.items():
            pip_angle = self._angle(landmarks[mcp], landmarks[pip], landmarks[dip])
            dip_angle = self._angle(landmarks[pip], landmarks[dip], landmarks[tip])
            palm_length = max(np.linalg.norm(landmarks[mcp] - wrist), 1e-6)
            reach = np.linalg.norm(landmarks[tip] - wrist) / palm_length
            straightness = min(
                self._ramp(pip_angle, 125.0, 165.0),
                self._ramp(dip_angle, 120.0, 160.0),
                self._ramp(reach, 1.15, 1.65),
            )
            scores[name] = float(straightness)
            states[name] = straightness >= 0.55

        extended = [name for name, value in states.items() if value]
        if not extended:
            gesture = "rock"
            confidence = float(np.mean([1.0 - value for value in scores.values()]))
        elif states == {"index": True, "middle": True, "ring": False, "little": False}:
            gesture = "scissors"
            confidence = float(np.mean([
                scores["index"], scores["middle"],
                1.0 - scores["ring"], 1.0 - scores["little"],
            ]))
        elif len(extended) == 4:
            gesture = "paper"
            confidence = float(np.mean(list(scores.values())))
        else:
            gesture = "unknown"
            confidence = max(0.0, 1.0 - min(abs(len(extended) - n) for n in (0, 2, 4)) / 2.0)

        if confidence < self._gesture_threshold:
            gesture = "unknown"
        return gesture, round(confidence, 4), states, scores

    @staticmethod
    def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        ba, bc = a - b, c - b
        denominator = np.linalg.norm(ba) * np.linalg.norm(bc)
        if denominator < 1e-8:
            return 0.0
        cosine = float(np.clip(np.dot(ba, bc) / denominator, -1.0, 1.0))
        return math.degrees(math.acos(cosine))

    @staticmethod
    def _ramp(value: float, low: float, high: float) -> float:
        return float(np.clip((value - low) / (high - low), 0.0, 1.0))
