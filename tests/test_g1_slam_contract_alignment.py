"""The checked-in Driver and Perception copies must remain identical."""
import os
from pathlib import Path

import pytest

from common.spatial_slam import (
    SpatialSlamError, validate_input_port,
    validate_frame_identity, validate_slam_capability,
)


def test_optional_platform_contract_mirror_is_exactly_identical():
    platform = os.environ.get('PERCEPTION_REFERENCE_ROOT')
    if not platform:
        pytest.skip('set PERCEPTION_REFERENCE_ROOT for cross-repo contract mirror check')
    platform_contract = Path(platform) / 'perception/plugins/pointcloud_slam_contract.py'
    driver_contract = Path(__file__).resolve().parents[1] / 'common/spatial_slam.py'
    assert platform_contract.exists(), 'Perception contract mirror is required'
    assert driver_contract.read_bytes() == platform_contract.read_bytes()


def test_real_g1_declarations_remain_inadmissible_without_evidence():
    from common.slam_input import input_declaration
    port = input_declaration('/g1/lidar/cloud_native', 'sensor/pointcloud2',
                             'cloud', '/g1/lidar/cloud_native',
                             frame_metadata_topic='/g1/lidar/cloud_native/metadata')
    with pytest.raises(SpatialSlamError):
        validate_input_port(port)
    with pytest.raises(SpatialSlamError):
        validate_frame_identity({
            'schema': 'motus.slam_frame/1', 'stamp_ns': 27000000000,
            'frame_id': 'livox_native', 'clock_id': None,
            'source_generation': 'adapter-only',
        }, clock_id='verified_clock')


def test_false_slam_capability_is_rejected_even_if_profile_matches():
    from unitree.g1.sport_odometry import SportOdometry
    iface = SportOdometry().interface()
    assert iface['slam_pose']['profile'] == 'motus_slam/1'
    assert iface['slam_pose']['available'] is False
    with pytest.raises(SpatialSlamError, match='availability'):
        validate_slam_capability(iface)
