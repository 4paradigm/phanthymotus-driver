"""Reproduce left-arm collision covers from explicitly supplied, pinned STLs.

This offline tool does not download meshes, contact a robot, or alter runtime
geometry. Use --check to compare with the committed evidence without writing.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct


COMMIT = '157ff950351e5f535e03c5d85a16b7d594b3dbcc'
SPECS = (
    ('left_shoulder_yaw_link.STL', 2, [-.114, -.060, -.020, .032],
     '1aa97e9748e924336567992181f78c7cd0652fd52a4afcca3df6b2ef6f9e712e'),
    ('left_elbow_link.STL', 0, [-.030, .010, .053, .101],
     'fa752198accd104d5c4c3a01382e45165b944fbbc5acce085059223324e5bed3'),
    ('left_wrist_roll_rubber_hand.STL', 0, [0, .060, .125, .190, .254],
     'e81030abd023bd9e4a308ef376d814a2c12d684d8a7670c335bbd5cd7809c909'),
)
DEFAULT_REPORT = Path(__file__).with_name('point-left-collision-cover.json')


def read_triangles(path, expected_hash):
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_hash:
        raise ValueError('source_sha256_mismatch:' + path.name)
    if len(raw) < 84:
        raise ValueError('invalid_binary_stl:' + path.name)
    count = struct.unpack_from('<I', raw, 80)[0]
    if not count or len(raw) != 84 + 50 * count:
        raise ValueError('binary_stl_size_mismatch:' + path.name)
    triangles = []
    for index in range(count):
        values = struct.unpack_from('<12fH', raw, 84 + 50 * index)
        vertices = tuple(tuple(values[3 + 3 * j + i] for i in range(3))
                         for j in range(3))
        if not all(math.isfinite(value) for point in vertices for value in point):
            raise ValueError('nonfinite_mesh_vertex:' + path.name)
        triangles.append(vertices)
    return triangles


def build_report(mesh_dir):
    report = dict(
        schema='g1.point.left-collision-cover/1',
        source_repository='unitreerobotics/unitree_ros',
        source_commit=COMMIT,
        units='m',
        method='Triangle AABB intersection with closed slabs, outward rounded to whole millimetres; no triangle omitted.',
        meshes=[],
    )
    for filename, axis, nominal_breaks, expected_hash in SPECS:
        triangles = read_triangles(mesh_dir / filename, expected_hash)
        lower = [min(point[i] for tri in triangles for point in tri) for i in range(3)]
        upper = [max(point[i] for tri in triangles for point in tri) for i in range(3)]
        cuts = ([min(nominal_breaks[0], lower[axis])]
                + [value for value in nominal_breaks[1:-1]
                   if lower[axis] < value < upper[axis]]
                + [max(nominal_breaks[-1], upper[axis])])
        boxes, counts = [], []
        for begin, end in zip(cuts, cuts[1:]):
            active = []
            for triangle in triangles:
                lo = [min(point[i] for point in triangle) for i in range(3)]
                hi = [max(point[i] for point in triangle) for i in range(3)]
                if hi[axis] < begin or lo[axis] > end:
                    continue
                lo[axis], hi[axis] = max(lo[axis], begin), min(hi[axis], end)
                active.append((lo, hi))
            if not active:
                raise ValueError('empty_source_slab:' + filename)
            lo = [math.floor(min(a[i] for a, _ in active) * 1000) for i in range(3)]
            hi = [math.ceil(max(b[i] for _, b in active) * 1000) for i in range(3)]
            # A triangle lies in its AABB. Closed slabs share their boundary,
            # so every triangle/slab intersection is contained by this union.
            if not all(all(lo[i] / 1000 <= a[i] <= b[i] <= hi[i] / 1000
                           for i in range(3)) for a, b in active):
                raise ValueError('triangle_cover_failed:' + filename)
            boxes.append(lo + hi)
            counts.append(len(active))
        report['meshes'].append(dict(
            filename=filename, sha256=expected_hash, triangles=len(triangles),
            actual_minimum=lower, actual_maximum=upper, slice_axis=axis,
            slab_edges_m=cuts, boxes_mm=boxes, contributing_triangles=counts,
        ))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh-dir', type=Path, required=True,
                        help='Directory containing the three pinned upstream STL files')
    parser.add_argument('--output', type=Path, default=DEFAULT_REPORT,
                        help='Report to write, or compare when --check is set')
    parser.add_argument('--check', action='store_true',
                        help='Compare the generated evidence with the report; do not write')
    args = parser.parse_args()
    report = build_report(args.mesh_dir)
    if args.check:
        if json.loads(args.output.read_text(encoding='utf-8')) != report:
            raise SystemExit('LEFT_COLLISION_COVER_MISMATCH')
        print('LEFT_COLLISION_COVER_OK meshes=3 mode=check')
    else:
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print('LEFT_COLLISION_COVER_WRITTEN', args.output)


if __name__ == '__main__':
    main()
