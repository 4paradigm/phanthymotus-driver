"""Raw IMU calibration must name the same physical camera as registered RGB-D."""
from types import SimpleNamespace as NS

import pytest

from common.realsense_imu import MotionSource


@pytest.mark.parametrize('reference,expected_rotation,expected_translation', [
    ('color', [[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]], [.01,.02,.03]),
    (None, [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], [0.,0.,0.]),
])
def test_motion_calibration_targets_selected_camera_and_retains_depth_default(
        reference, expected_rotation, expected_translation):
    target_color, target_depth = object(), object()
    def extrinsics(target):
        if target is target_color:
            return NS(rotation=[0.,1.,0.,-1.,0.,0.,0.,0.,1.], translation=[.01,.02,.03])
        assert target is target_depth
        return NS(rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.], translation=[0.,0.,0.])
    profiles = {kind:NS(get_extrinsics_to=extrinsics, stream_index=lambda:0,
                        unique_id=lambda kind=kind:kind) for kind in ('gyro','accel')}
    profiles.update(color=target_color, depth=target_depth)
    active = NS(get_stream=profiles.__getitem__, get_device=lambda:NS(get_info=lambda _: 'test-camera'))
    sdk = NS(stream=NS(gyro='gyro',accel='accel',depth='depth',color='color'),
             camera_info=NS(serial_number='serial'))
    source = MotionSource('/imu','rgb_optical' if reference else 'depth_optical',sdk,
        lambda ms,domain:1_000_000_000, lambda packet:None, clock_id=lambda:'capture')
    if reference is None:source.configure(active)
    else:source.configure(active,camera_stream=reference)
    def motion(kind):
        return NS(as_motion_frame=lambda:NS(profile=NS(stream_type=lambda:kind),
            get_frame_timestamp_domain=lambda:'hardware_clock', get_timestamp=lambda:1000.,
            get_motion_data=lambda:NS(x=1.,y=2.,z=3.)))
    assert source.accept(motion('accel')) is None
    packet = source.accept(motion('gyro'))
    assert packet['camera_from_gyro'] == packet['camera_from_accel'] == expected_rotation
    assert packet['camera_from_gyro_translation'] == packet['camera_from_accel_translation'] == expected_translation
    assert packet['camera_frame_id'] == ('rgb_optical' if reference else 'depth_optical')
    assert packet['angular_velocity'] == packet['linear_acceleration'] == [1.,2.,3.]
