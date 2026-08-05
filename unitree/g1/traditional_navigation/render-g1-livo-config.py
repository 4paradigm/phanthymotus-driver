#!/usr/bin/env python3
"""Render a runnable FAST-LIVO2 RGB config from a verified G1 calibration.

The input is the sensor-collector calibration snapshot.  This renderer is
deliberately fail-closed: factory camera intrinsics alone are insufficient.
The snapshot must also carry a verified LiDAR-to-color transform and measured
camera/LiDAR time alignment for the exact runtime color profile.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


ALLOWED_EVIDENCE_STATUS = {"calibrated", "measured", "verified"}
ALLOWED_TIMESTAMP_SOURCES = {
    "realsense_global_time",
    "realsense_hardware_clock",
}
NOMINAL_PREVIEW_EXTRINSIC_STATUS = "nominal_public_urdf"
NOMINAL_PREVIEW_ALIGNMENT_STATUS = "preview_only"
NOMINAL_PREVIEW_TIMESTAMP_SOURCE = "callback_arrival_preview"


def _mapping(value: Any, path: str, blockers: list[str]) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    blockers.append(f"{path} 缺失或不是 object")
    return {}


def _finite_number(value: Any, path: str, blockers: list[str]) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        blockers.append(f"{path} 必须是有限数值")
        return 0.0
    if not math.isfinite(result):
        blockers.append(f"{path} 必须是有限数值")
        return 0.0
    return result


def _number_list(value: Any, size: int, path: str, blockers: list[str]) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        blockers.append(f"{path} 必须包含 {size} 个数值")
        return [0.0] * size
    return [_finite_number(item, f"{path}[{index}]", blockers) for index, item in enumerate(value)]


def _validate_rotation(rotation: list[float], blockers: list[str]) -> None:
    rows = [rotation[0:3], rotation[3:6], rotation[6:9]]
    tolerance = 5e-3
    for index, row in enumerate(rows):
        norm = math.sqrt(sum(value * value for value in row))
        if abs(norm - 1.0) > tolerance:
            blockers.append(f"LiDAR→相机旋转矩阵第 {index + 1} 行未归一化")
    for left in range(3):
        for right in range(left + 1, 3):
            dot = sum(rows[left][axis] * rows[right][axis] for axis in range(3))
            if abs(dot) > tolerance:
                blockers.append("LiDAR→相机旋转矩阵不正交")
                return
    determinant = (
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6])
    )
    if abs(determinant - 1.0) > tolerance:
        blockers.append("LiDAR→相机旋转矩阵 determinant 必须接近 +1")


def validate_snapshot(
    snapshot: dict[str, Any],
    expected_width: int,
    expected_height: int,
    expected_fps: int,
    allow_nominal_preview: bool = False,
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    data = _mapping(snapshot.get("data"), "data", blockers)

    encoded = json.dumps(
        data,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    actual_calibration_id = "sha256:" + hashlib.sha256(encoded).hexdigest()
    declared_calibration_id = str(snapshot.get("calibration_id", ""))
    if declared_calibration_id != actual_calibration_id:
        blockers.append("calibration_id 与 data 的规范化 SHA256 不一致")

    d435i = _mapping(data.get("d435i"), "data.d435i", blockers)
    identity = _mapping(d435i.get("identity"), "data.d435i.identity", blockers)
    serial = str(identity.get("serial", "")).strip()
    if not serial:
        blockers.append("D435i serial 缺失")

    profiles = _mapping(d435i.get("profiles"), "data.d435i.profiles", blockers)
    color = _mapping(profiles.get("color"), "data.d435i.profiles.color", blockers)
    width = int(_finite_number(color.get("width"), "color.width", blockers))
    height = int(_finite_number(color.get("height"), "color.height", blockers))
    fps = int(_finite_number(color.get("fps"), "color.fps", blockers))
    if (width, height, fps) != (expected_width, expected_height, expected_fps):
        blockers.append(
            "D435i 标定 profile "
            f"{width}x{height}@{fps} 与运行 profile "
            f"{expected_width}x{expected_height}@{expected_fps} 不一致"
        )

    intrinsics = _mapping(color.get("intrinsics"), "color.intrinsics", blockers)
    fx = _finite_number(intrinsics.get("fx"), "color.intrinsics.fx", blockers)
    fy = _finite_number(intrinsics.get("fy"), "color.intrinsics.fy", blockers)
    cx = _finite_number(
        intrinsics.get("cx", intrinsics.get("ppx")),
        "color.intrinsics.cx/ppx",
        blockers,
    )
    cy = _finite_number(
        intrinsics.get("cy", intrinsics.get("ppy")),
        "color.intrinsics.cy/ppy",
        blockers,
    )
    if fx <= 0 or fy <= 0:
        blockers.append("D435i fx/fy 必须为正数")
    if not 0 <= cx < max(width, 1) or not 0 <= cy < max(height, 1):
        blockers.append("D435i cx/cy 必须位于标定图像范围内")
    coefficients = _number_list(
        intrinsics.get("coeffs"), 5, "color.intrinsics.coeffs", blockers
    )
    distortion_model = str(intrinsics.get("distortion_model", "")).lower()
    if distortion_model == "inverse_brown_conrady" and any(
        abs(value) > 1e-9 for value in coefficients
    ):
        blockers.append("非零 inverse_brown_conrady 系数不能直接写入 FAST-LIVO2 Pinhole 模型")
    elif distortion_model not in {"brown_conrady", "inverse_brown_conrady", "none"}:
        blockers.append(f"不支持的 D435i 畸变模型: {distortion_model or 'missing'}")

    ground_truth_value = data.get("ground_truth")
    ground_truth = ground_truth_value if isinstance(ground_truth_value, dict) else {}
    if not ground_truth:
        blockers.append("data.ground_truth 缺失，尚无 RGB-LiDAR 联合标定")
    transforms_value = ground_truth.get("transforms")
    transforms = transforms_value if isinstance(transforms_value, dict) else {}
    lidar_to_camera_value = transforms.get("lidar_to_camera")
    lidar_to_camera = (
        lidar_to_camera_value if isinstance(lidar_to_camera_value, dict) else {}
    )
    transform_status = str(lidar_to_camera.get("status", ""))
    nominal_preview = (
        allow_nominal_preview
        and transform_status == NOMINAL_PREVIEW_EXTRINSIC_STATUS
    )
    if transform_status not in ALLOWED_EVIDENCE_STATUS and not nominal_preview:
        blockers.append("LiDAR→D435i 外参状态必须是 calibrated/measured/verified")
    rotation_value = lidar_to_camera.get("rotation_row_major")
    rotation = _number_list(
        rotation_value,
        9,
        "lidar_to_camera.rotation_row_major",
        blockers,
    )
    translation_value = lidar_to_camera.get("translation_m")
    translation = _number_list(
        translation_value,
        3,
        "lidar_to_camera.translation_m",
        blockers,
    )
    if isinstance(rotation_value, list) and len(rotation_value) == 9:
        _validate_rotation(rotation, blockers)
    if isinstance(translation_value, list) and len(translation_value) == 3:
        translation_norm = math.sqrt(sum(value * value for value in translation))
        if not 0.01 <= translation_norm <= 2.0:
            blockers.append("LiDAR→D435i 平移模长必须在 0.01–2.0 m 内")

    alignments_value = ground_truth.get("time_alignment")
    alignments = alignments_value if isinstance(alignments_value, dict) else {}
    alignment_value = alignments.get("d435i_color_to_mid360")
    alignment = alignment_value if isinstance(alignment_value, dict) else {}
    alignment_status = str(alignment.get("status", ""))
    preview_alignment = (
        nominal_preview
        and alignment_status == NOMINAL_PREVIEW_ALIGNMENT_STATUS
    )
    if alignment_status not in ALLOWED_EVIDENCE_STATUS and not preview_alignment:
        blockers.append("D435i↔MID360 时间对齐状态必须是 calibrated/measured/verified")
    timestamp_source = str(alignment.get("timestamp_source", ""))
    preview_timestamp = (
        preview_alignment and timestamp_source == NOMINAL_PREVIEW_TIMESTAMP_SOURCE
    )
    if timestamp_source not in ALLOWED_TIMESTAMP_SOURCES and not preview_timestamp:
        blockers.append("相机时间戳必须来自 realsense hardware/global time，不能用回调到达时间")
    if "img_time_offset_s" in alignment:
        image_time_offset = _finite_number(
            alignment.get("img_time_offset_s"), "time_alignment.img_time_offset_s", blockers
        )
        if abs(image_time_offset) > 0.5:
            blockers.append("img_time_offset_s 绝对值必须不超过 0.5 s")
    else:
        image_time_offset = 0.0
        blockers.append("缺少实测 img_time_offset_s")
    if preview_alignment and alignment.get("p95_abs_skew_ms") is None:
        p95_skew_ms = -1.0
    elif "p95_abs_skew_ms" in alignment:
        p95_skew_ms = _finite_number(
            alignment.get("p95_abs_skew_ms"), "time_alignment.p95_abs_skew_ms", blockers
        )
        if not 0 <= p95_skew_ms <= 20.0:
            blockers.append("D435i↔MID360 绝对 skew p95 必须不超过 20 ms")
    else:
        p95_skew_ms = 0.0
        blockers.append("缺少 D435i↔MID360 skew p95 证据")

    values = {
        "calibration_id": actual_calibration_id,
        "serial": serial,
        "width": width,
        "height": height,
        "fps": fps,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "distortion": coefficients[:4],
        "rotation": rotation,
        "translation": translation,
        "image_time_offset": image_time_offset,
        "timestamp_source": timestamp_source,
        "p95_skew_ms": p95_skew_ms,
        "evidence_status": transform_status,
        "preview_only": nominal_preview,
    }
    return values, blockers


def _yaml_float(value: float) -> str:
    """Render a finite YAML number with an unambiguous floating-point type."""
    rendered = format(value, ".12g")
    if "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return rendered


def _yaml_list(values: list[float]) -> str:
    return "[" + ", ".join(_yaml_float(value) for value in values) + "]"


def render_yaml(values: dict[str, Any]) -> str:
    distortion = values["distortion"]
    p95_skew = (
        f"{values['p95_skew_ms']:.3f}"
        if values["p95_skew_ms"] >= 0
        else "unverified"
    )
    return f"""# Generated by render-g1-livo-config.py; do not edit by hand.
# calibration_id: {values['calibration_id']}
# d435i_serial: {values['serial']}
# color_profile: {values['width']}x{values['height']}@{values['fps']}
# evidence_tier: {values['evidence_status']}
# timestamp_source: {values['timestamp_source']}; p95_skew_ms: {p95_skew}
# preview_only: {str(values['preview_only']).lower()}
/**:
  ros__parameters:
    common:
      img_topic: "/left_camera/image"
      lid_topic: "/livox/lidar"
      imu_topic: "/livox/imu"
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false
      enable_image_processing: true

    camera:
      model: Pinhole
      width: {values['width']}
      height: {values['height']}
      scale: 1.0
      fx: {_yaml_float(values['fx'])}
      fy: {_yaml_float(values['fy'])}
      cx: {_yaml_float(values['cx'])}
      cy: {_yaml_float(values['cy'])}
      d0: {_yaml_float(distortion[0])}
      d1: {_yaml_float(distortion[1])}
      d2: {_yaml_float(distortion[2])}
      d3: {_yaml_float(distortion[3])}

    extrin_calib:
      extrinsic_T: [0.0, 0.0, 0.0]
      extrinsic_R: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
      Rcl: {_yaml_list(values['rotation'])}
      Pcl: {_yaml_list(values['translation'])}

    time_offset:
      imu_time_offset: 0.0
      img_time_offset: {_yaml_float(values['image_time_offset'])}
      exposure_time_init: 0.0

    preprocess:
      point_filter_num: 1
      filter_size_surf: 0.1
      lidar_type: 7
      scan_line: 4
      blind: 0.5

    vio:
      max_iterations: 5
      outlier_threshold: 1000
      img_point_cov: 100
      patch_size: 8
      patch_pyrimid_level: 4
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: false
      inv_expo_cov: 0.1

    imu:
      imu_en: true
      imu_int_frame: 30
      acc_cov: 0.8
      gyr_cov: 0.5
      b_acc_cov: 0.001
      b_gyr_cov: 0.001

    lio:
      max_iterations: 5
      dept_err: 0.02
      beam_err: 0.15
      min_eigen_value: 0.005
      voxel_size: 0.5
      max_layer: 2
      max_points_num: 50
      layer_init_num: [5, 5, 5, 5, 5]

    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0

    uav:
      imu_rate_odom: false
      gravity_align_en: true

    publish:
      dense_map_en: true
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0

    evo:
      seq_name: "g1_rgb_livo_shadow"
      pose_output_en: false

    image_save:
      img_save_en: false
      interval: 1

    pcd_save:
      pcd_save_en: false
      colmap_output_en: false
      filter_size_pcd: 0.02
      interval: -1
"""


def write_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, prefix=path.name + ".", delete=False
    ) as handle:
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--expected-width", type=int, default=1920)
    parser.add_argument("--expected-height", type=int, default=1080)
    parser.add_argument("--expected-fps", type=int, default=15)
    parser.add_argument(
        "--allow-nominal-preview",
        action="store_true",
        help="render a static-only config from explicitly nominal public URDF evidence",
    )
    args = parser.parse_args()

    with args.calibration.open(encoding="utf-8") as handle:
        snapshot = json.load(handle)
    if not isinstance(snapshot, dict):
        raise SystemExit("calibration snapshot must be a JSON object")

    values, blockers = validate_snapshot(
        snapshot,
        args.expected_width,
        args.expected_height,
        args.expected_fps,
        args.allow_nominal_preview,
    )
    if blockers:
        for blocker in blockers:
            print(f"BLOCKER: {blocker}")
        print(f"rgb_livo_calibration=blocked blocker_count={len(blockers)}")
        return 1

    state = "preview" if values["preview_only"] else "ready"
    print(
        f"rgb_livo_calibration={state} "
        f"calibration_id={values['calibration_id']} "
        f"serial={values['serial']} "
        f"profile={values['width']}x{values['height']}@{values['fps']}"
    )
    if args.output is not None:
        write_atomic(args.output, render_yaml(values))
        print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
