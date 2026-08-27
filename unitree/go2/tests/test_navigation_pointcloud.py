import struct
import sys
import unittest

# Some legacy Driver tests install a minimal numpy import stub before importing
# device.py. Restore the real dependency when the complete suite runs in one
# interpreter so this numeric conversion test remains order-independent.
if not hasattr(sys.modules.get("numpy"), "dtype"):
    sys.modules.pop("numpy", None)
import numpy as np

from navigation_pointcloud import (
    NAVIGATION_FIELDS,
    NAVIGATION_POINT_STEP,
    FLOAT32,
    UINT16,
    rotate_covariance9,
    rotate_orientation_xyzw,
    rotate_vector3,
    unitree_mid360_to_navigation_cloud,
    validated_rotation_matrix,
)


UNITREE_FIELDS = [
    ("x", 0, FLOAT32, 1),
    ("y", 4, FLOAT32, 1),
    ("z", 8, FLOAT32, 1),
    ("intensity", 16, FLOAT32, 1),
    ("ring", 20, UINT16, 1),
    ("time", 24, FLOAT32, 1),
]
UNITREE_POINT_STEP = 32


class Mid360ConversionTest(unittest.TestCase):
    def make_cloud(self):
        data = bytearray(2 * UNITREE_POINT_STEP)
        for index, values in enumerate(
            (
                (1.0, 2.0, 3.0, 42.0, 0, 5_000.0),
                (-1.0, -2.0, -3.0, 163.0, 3, 100_320_000.0),
            )
        ):
            base = index * UNITREE_POINT_STEP
            x, y, z, intensity, ring, point_time = values
            struct.pack_into("<fff", data, base, x, y, z)
            struct.pack_into("<f", data, base + 16, intensity)
            struct.pack_into("<H", data, base + 20, ring)
            struct.pack_into("<f", data, base + 24, point_time)
        return bytes(data)

    def decode(self, converted):
        dtype = np.dtype(
            {
                "names": [field.name for field in NAVIGATION_FIELDS],
                "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
                "offsets": [field.offset for field in NAVIGATION_FIELDS],
                "itemsize": NAVIGATION_POINT_STEP,
            }
        )
        return np.frombuffer(converted, dtype=dtype)

    def test_converts_relative_time_to_absolute_nanoseconds(self):
        header_ns = 1_785_811_000_000_000_000
        converted = unitree_mid360_to_navigation_cloud(
            data=self.make_cloud(),
            height=1,
            width=2,
            point_step=UNITREE_POINT_STEP,
            row_step=2 * UNITREE_POINT_STEP,
            fields=UNITREE_FIELDS,
            header_stamp_ns=header_ns,
        )
        points = self.decode(converted)

        self.assertEqual(len(converted), 2 * NAVIGATION_POINT_STEP)
        np.testing.assert_allclose(points["x"], [1.0, -1.0])
        np.testing.assert_allclose(points["intensity"], [42.0, 163.0])
        np.testing.assert_array_equal(points["tag"], [0x10, 0x10])
        np.testing.assert_array_equal(points["line"], [0, 3])
        np.testing.assert_allclose(
            points["timestamp"] - np.float64(header_ns),
            [5_000.0, 100_320_000.0],
            atol=256.0,
        )

    def test_rejects_missing_time_field(self):
        with self.assertRaisesRegex(ValueError, "missing MID360 field: time"):
            unitree_mid360_to_navigation_cloud(
                data=self.make_cloud(),
                height=1,
                width=2,
                point_step=UNITREE_POINT_STEP,
                row_step=2 * UNITREE_POINT_STEP,
                fields=[field for field in UNITREE_FIELDS if field[0] != "time"],
                header_stamp_ns=1_000_000_000,
            )

    def test_rejects_short_data(self):
        with self.assertRaisesRegex(ValueError, "size must equal"):
            unitree_mid360_to_navigation_cloud(
                data=b"short",
                height=1,
                width=2,
                point_step=UNITREE_POINT_STEP,
                row_step=2 * UNITREE_POINT_STEP,
                fields=UNITREE_FIELDS,
                header_stamp_ns=1_000_000_000,
            )

    def test_rejects_organized_cloud_row_padding(self):
        with self.assertRaisesRegex(ValueError, "tightly packed"):
            unitree_mid360_to_navigation_cloud(
                data=self.make_cloud() + b"\0" * 4,
                height=2,
                width=1,
                point_step=UNITREE_POINT_STEP,
                row_step=UNITREE_POINT_STEP + 2,
                fields=UNITREE_FIELDS,
                header_stamp_ns=1_000_000_000,
            )

    def test_rejects_row_step_smaller_than_one_row(self):
        with self.assertRaisesRegex(ValueError, "tightly packed"):
            unitree_mid360_to_navigation_cloud(
                data=self.make_cloud()[:60],
                height=1,
                width=2,
                point_step=UNITREE_POINT_STEP,
                row_step=60,
                fields=UNITREE_FIELDS,
                header_stamp_ns=1_000_000_000,
            )

    def test_applies_fixed_rotation_without_changing_auxiliary_fields(self):
        rotation = validated_rotation_matrix(
            [
                1.0,
                0.0,
                0.0,
                0.0,
                -1.0,
                0.0,
                0.0,
                0.0,
                -1.0,
            ]
        )
        header_ns = 1_785_811_000_000_000_000
        converted = unitree_mid360_to_navigation_cloud(
            data=self.make_cloud(),
            height=1,
            width=2,
            point_step=UNITREE_POINT_STEP,
            row_step=2 * UNITREE_POINT_STEP,
            fields=UNITREE_FIELDS,
            header_stamp_ns=header_ns,
            rotation_matrix=rotation,
        )
        points = self.decode(converted)

        np.testing.assert_allclose(points["x"], [1.0, -1.0])
        np.testing.assert_allclose(points["y"], [-2.0, 2.0])
        np.testing.assert_allclose(points["z"], [-3.0, 3.0])
        np.testing.assert_allclose(points["intensity"], [42.0, 163.0])
        np.testing.assert_array_equal(points["tag"], [0x10, 0x10])
        np.testing.assert_array_equal(points["line"], [0, 3])
        np.testing.assert_allclose(
            points["timestamp"] - np.float64(header_ns),
            [5_000.0, 100_320_000.0],
            atol=256.0,
        )

    def test_rotation_helpers_use_the_same_corrected_frame(self):
        rotation = validated_rotation_matrix(
            [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
        )

        np.testing.assert_allclose(
            rotate_vector3((1.0, 2.0, 3.0), rotation),
            (1.0, -2.0, -3.0),
        )
        np.testing.assert_allclose(
            rotate_covariance9(
                [1.0, 2.0, 3.0, 2.0, 4.0, 5.0, 3.0, 5.0, 6.0],
                rotation,
            ),
            [1.0, -2.0, -3.0, -2.0, 4.0, 5.0, -3.0, 5.0, 6.0],
        )
        qx, qy, qz, qw = rotate_orientation_xyzw(
            (0.0, 0.0, 0.0, 1.0), rotation
        )
        self.assertAlmostEqual(abs(qx), 1.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, 0.0)
        self.assertAlmostEqual(qw, 0.0)

    def test_rejects_non_rotation_matrix(self):
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            validated_rotation_matrix([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0])
        with self.assertRaisesRegex(ValueError, "proper rotation"):
            validated_rotation_matrix([-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
