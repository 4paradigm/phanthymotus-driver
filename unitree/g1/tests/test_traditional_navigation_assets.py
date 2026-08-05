import hashlib
import re
from pathlib import Path
import unittest

G1_DIR = Path(__file__).resolve().parents[1]
NAV_DIR = G1_DIR / "traditional_navigation"


def load_source_lock() -> dict[str, str]:
    values = {}
    for raw_line in (NAV_DIR / "source-lock.env").read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


class TraditionalNavigationAssetsTest(unittest.TestCase):
    def test_all_source_revisions_are_full_git_shas(self):
        source_lock = load_source_lock()
        commits = {
            key: value for key, value in source_lock.items() if key.endswith("_COMMIT")
        }
        self.assertEqual(len(commits), 5)
        for key, value in commits.items():
            self.assertRegex(value, r"^[0-9a-f]{40}$", key)

    def test_fast_livo_config_is_lio_only_and_non_persistent(self):
        config = (NAV_DIR / "g1_lio.yaml").read_text()
        for required_line in (
            "      img_en: 0",
            "      lidar_en: 1",
            "      model: Pinhole",
            "      imu_en: true",
            "      filter_size_surf: 0.1",
            "      lidar_type: 7",
            "      scan_line: 4",
            "      pcd_save_en: false",
            "      colmap_output_en: false",
            "      dense_map_en: false",
        ):
            self.assertIn(required_line, config)

    def test_shadow_compose_has_no_robot_write_surface(self):
        compose = (NAV_DIR / "compose.shadow.yml").read_text()
        self.assertIn('profiles: ["lio-shadow"]', compose)
        self.assertIn("read_only: true", compose)
        self.assertIn('restart: "no"', compose)
        self.assertNotIn("privileged:", compose)
        self.assertIn("cap_drop:\n      - ALL", compose)
        self.assertIn("navigation/lidar_fast_livo", compose)
        self.assertIn("navigation/odom", compose)
        self.assertIn("navigation/tf_lio_raw", compose)
        self.assertIn('PCD_SAVE_EN: "false"', compose)
        self.assertIn("pcd_save.pcd_save_en:=$${PCD_SAVE_EN}", compose)
        self.assertIn(
            "exec /opt/fast_livo_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping",
            compose,
        )
        self.assertNotIn("exec ros2 run fast_livo", compose)
        self.assertNotRegex(compose, re.compile(r"cmd_vel|SmartMotion|/action/"))

    def test_mapping_override_is_explicit_and_only_writes_the_map_directory(self):
        mapping = (NAV_DIR / "compose.mapping.yml").read_text()

        self.assertIn('PCD_SAVE_EN: "true"', mapping)
        self.assertIn(
            'PCD_SAVE_INTERVAL: "${G1_PCD_SAVE_INTERVAL:-600}"', mapping
        )
        self.assertIn(
            'com.phanthy.navigation.pcd_save_interval: '
            '"${G1_PCD_SAVE_INTERVAL:-600}"',
            mapping,
        )
        self.assertIn('group_add:\n      - "${G1_MAP_GID:-1000}"', mapping)
        self.assertIn(
            "${G1_MAP_DIR:?set G1_MAP_DIR to a dedicated empty directory}",
            mapping,
        )
        self.assertIn(
            "com.phanthy.navigation.map_name: "
            "${G1_MAP_NAME:?set G1_MAP_NAME to the controlled_spatial map_name}",
            mapping,
        )
        self.assertIn("com.phanthy.navigation.mode: mapping", mapping)
        self.assertIn("/opt/fast_livo_ws/src/fast_livo/Log/pcd:rw", mapping)
        self.assertIn("stop_signal: SIGINT", mapping)
        self.assertIn("stop_grace_period: 120s", mapping)
        self.assertNotRegex(mapping, re.compile(r"cmd_vel|SmartMotion|/action/|/dev:/dev"))

    def test_sensor_bridge_is_an_isolated_non_privileged_sidecar(self):
        compose = (NAV_DIR / "compose.shadow.yml").read_text()
        driver_config = (NAV_DIR / "driver.shadow.yaml").read_text()
        driver_dockerfile = (G1_DIR / "Dockerfile").read_text()
        bridge_source = (G1_DIR / "navigation_sensor_bridge.py").read_text()

        self.assertIn("sensor-bridge:", compose)
        self.assertIn("navigation_sensor_bridge_main.py", compose)
        self.assertIn("condition: service_healthy", compose)
        self.assertNotIn("/dev:/dev", compose)
        self.assertNotIn("pid: host", compose)
        self.assertNotIn("privileged: true", compose)
        self.assertIn("navigation_sensor_bridge_main.py", driver_dockerfile)

        self.assertIn("navigation_sensors:\n    enabled: true", driver_config)
        self.assertIn("publish_raw_cloud: false", driver_config)
        self.assertIn("publish_fast_livo_cloud: true", driver_config)
        self.assertIn("raw_lidar_frame: livox_raw_frame", driver_config)
        self.assertIn(
            "sensor_rotation_matrix: [1.0, 0.0, 0.0,",
            driver_config,
        )
        self.assertIn("rotation_matrix=self._sensor_rotation", bridge_source)
        self.assertIn("rotate_orientation_xyzw(", bridge_source)
        self.assertGreaterEqual(bridge_source.count("rotate_vector3("), 2)
        self.assertGreaterEqual(bridge_source.count("rotate_covariance9("), 3)
        imu_qos = bridge_source.split("_IMU_QOS =", 1)[1].split(")", 1)[0]
        self.assertIn("ReliabilityPolicy.RELIABLE", imu_qos)
        self.assertNotRegex(
            driver_config,
            re.compile(r"(?:loco|arm|motion_switcher):\s*\n\s+enabled:\s*true"),
        )

    def test_dockerfile_requires_each_build_source_lock(self):
        dockerfile = (NAV_DIR / "Dockerfile.fast-livo2").read_text()
        source_lock = load_source_lock()
        for key in (
            "FAST_LIVO2_COMMIT",
            "RPG_VIKIT_COMMIT",
            "LIVOX_ROS_DRIVER2_COMMIT",
            "LIVOX_SDK2_COMMIT",
            "SOPHUS_COMMIT",
        ):
            self.assertIn(f"ARG {key}", dockerfile)
            self.assertIn(source_lock[key], (NAV_DIR / "source-lock.env").read_text())
        self.assertNotIn("checkout main", dockerfile)
        self.assertNotIn("checkout ros2", dockerfile)

    def test_fast_livo_runtime_patch_is_content_locked(self):
        dockerfile = (NAV_DIR / "Dockerfile.fast-livo2").read_text()
        compose = (NAV_DIR / "compose.shadow.yml").read_text()
        source_lock = load_source_lock()
        patch = (NAV_DIR / "fast-livo2-runtime.patch").read_bytes()
        expected_hash = source_lock["FAST_LIVO2_RUNTIME_PATCH_SHA256"]

        self.assertEqual(hashlib.sha256(patch).hexdigest(), expected_hash)
        self.assertRegex(expected_hash, r"^[0-9a-f]{64}$")
        self.assertIn("ARG FAST_LIVO2_RUNTIME_PATCH_SHA256", dockerfile)
        self.assertIn("sha256sum -c -", dockerfile)
        self.assertIn("git apply --check", dockerfile)
        self.assertIn("git apply /tmp/fast-livo2-runtime.patch", dockerfile)
        self.assertIn("FAST_LIVO2_RUNTIME_PATCH_SHA256:", compose)

    def test_fast_livo_runtime_patch_recovers_after_forward_imu_gap(self):
        patch = (NAV_DIR / "fast-livo2-runtime.patch").read_text()
        gap_condition = (
            "if (last_timestamp_imu > 0.0 && "
            "timestamp > last_timestamp_imu + 0.2)"
        )
        self.assertIn(gap_condition, patch)

        gap_block = patch.split(gap_condition, 1)[1].split(
            "last_timestamp_imu = timestamp;", 1
        )[0]
        patched_gap_block = "\n".join(
            line[1:]
            for line in gap_block.splitlines()
            if not line.startswith("-")
        )
        self.assertNotIn("return;", patched_gap_block)
        self.assertIn("accepting current sample to resynchronize", patched_gap_block)

    def test_hotfix_build_reuses_a_content_locked_base_without_network_fetches(self):
        dockerfile = (NAV_DIR / "Dockerfile.fast-livo2-hotfix").read_text()
        source_lock = load_source_lock()

        self.assertRegex(source_lock["FAST_LIVO2_BASE_IMAGE_TAG"], r"^[0-9a-z.-]+$")
        self.assertRegex(
            source_lock["FAST_LIVO2_BASE_RUNTIME_PATCH_SHA256"],
            r"^[0-9a-f]{64}$",
        )
        self.assertIn("ARG FAST_LIVO2_BASE_IMAGE", dockerfile)
        self.assertIn("FROM ${FAST_LIVO2_BASE_IMAGE}", dockerfile)
        self.assertIn('git reset --hard "${FAST_LIVO2_COMMIT}"', dockerfile)
        self.assertIn("git apply --check", dockerfile)
        self.assertIn("cd /opt/fast_livo_ws &&", dockerfile)
        self.assertIn("--packages-select fast_livo", dockerfile)
        self.assertIn("--cmake-clean-cache", dockerfile)
        self.assertIn(
            "/opt/fast_livo_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping",
            dockerfile,
        )
        self.assertIn("accepting current sample to resynchronize", dockerfile)
        self.assertNotIn("apt-get", dockerfile)
        self.assertNotIn("git fetch", dockerfile)


if __name__ == "__main__":
    unittest.main()
