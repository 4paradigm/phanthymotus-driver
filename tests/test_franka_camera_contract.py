from franka.fr3.camera_contract import optical_frame_id, stream_semantics


def test_physical_camera_identity_is_stable_and_distinct():
    assert optical_frame_id('franka', '263022075182', 'rgb') == 'franka_serial_263022075182_rgb_optical'
    assert len({optical_frame_id('franka', serial, 'rgb') for serial in ('a-b','a_b','a.b','hex_612d62','263022075182','263022071451')}) == 6
    assert len({optical_frame_id('franka','263022075182',ch) for ch in ('rgb','depth','infrared')}) == 3
    assert optical_frame_id('franka','','rgb') is None


def test_semantics_do_not_claim_alignment_or_hardware_time():
    depth=stream_semantics('franka','123','depth')
    assert depth['depth_unit']=='mm' and depth['invalid_depth_value']==0
    assert depth['depth_aligned_to_rgb'] is False
    assert depth['timestamp_source']=='publication_clock'
    assert depth['hardware_capture_timestamp_available'] is False
    ir=stream_semantics('franka','123','infrared')
    assert ir['image_semantics']=='left_near_infrared_intensity_not_temperature'
    assert 'depth_unit' not in ir
