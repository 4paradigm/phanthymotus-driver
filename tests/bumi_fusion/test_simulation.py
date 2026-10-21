import pytest
from noetix.bumi_odometry.simulate_control import trial


def test_nominal_model_stops_and_reports_truth_separately():
    data=trial()
    assert data['result']['state']=='done' and data['final_command']==0
    assert abs(data['control_error_m'])<.05
    assert data['result']['accuracy_validated'] is False
    assert data['fusion_algorithm_executed'] is False
    assert data['rows'][10]['truth_m']!=data['rows'][10]['estimated_m']


def test_done_does_not_certify_accuracy_when_odometry_scale_is_wrong():
    data=trial(scale=1.15)
    assert data['result']['state']=='done'
    assert abs(data['control_error_m'])>.1
    assert abs(data['estimation_error_m'])>.1


@pytest.mark.parametrize('fault',['lost','stale','reset','connection','cancel'])
def test_each_fault_stops_without_switching_to_imu(fault):
    data=trial(fault=fault)
    assert data['final_command']==0
    assert data['result']['state']==('stopped' if fault=='cancel' else 'error')
    assert data['result']['feedback_source']=='rgbd_imu_ekf'
    assert data['truth_m']<.6


def test_fault_result_reports_invalid_quality_not_only_a_source_name():
    result=trial(fault='lost')['result']
    assert result['estimated_quality']['status']=='lost'
    assert result['estimated_quality']['reason']
