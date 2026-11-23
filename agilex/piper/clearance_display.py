"""Canvas-readable labels while preserving machine-readable measurement fields."""
FIELDS = {
    'state':'数据状态：estimated=模型估算，ok=已输入验证标定，unavailable=不可用，idle=已停止',
    'reason':'当前状态原因代码', 'clearance_mm':'最低模型点到桌面的有符号间隙，毫米；负数表示模型穿过桌面',
    'conservative_clearance_mm':'间隙减去实测误差余量，毫米；未标定时为空',
    'lowest_point_base_mm':'最低模型点在所列基坐标系的 [x,y,z]，毫米',
    'estimated':'是否为模型估算', 'calibration_verified':'是否声明完成实测标定',
    'closest_part':'离桌最近的部件标识',
    'part_heights_mm':'各部件离桌高度（mm）：gripper_link1=夹指1，gripper_link2=夹指2，gripper_base=夹爪本体，flange_link=连接法兰',
    'opening_mm':'实时夹爪开度，毫米', 'feedback_time':'最早一项有效输入的 Unix 秒时间戳',
    'timestamp':'本条数据发布时的 Unix 秒时间戳', 'unit':'长度单位',
    'base_frame':'最低点坐标所属的基坐标系', 'model_commit':'官方模型来源提交版本',
    'model_sha256':'实际加载模型文件的 SHA-256', 'model_source':'模型上游仓库',
    'table_z_mm':'模型坐标系中的水平桌面高度，毫米', 'assumptions':'本次估算采用的安装与桌面假设',
    'collision_protection_active':'是否执行碰撞拦截；本卡始终不执行',
    'depth_used':'是否使用相机深度；本卡没有使用', 'advisory_only':'仅供观察，不作为安全联锁',
    'calibration_id':'实测标定记录编号', 'uncertainty_mm':'输入的实测总误差余量，毫米',
    'calibration_can_interface':'这份实测标定绑定的 CAN 接口，必须与测距来源一致',
    'model':'计算方法标识', 'quantity':'测量量：有符号桌面法向间隙',
    'can_interface':'当前测距来源 CAN 接口', 'arm_label':'用户配置的机械臂名称（不是自动识别）',
    'measurement_mode':'所选的模型估算或实测标定模式',
}
PARTS = {'gripper_link1':'夹指1','gripper_link2':'夹指2','gripper_base':'夹爪本体','flange_link':'连接法兰'}


def annotate(result, cfg, now):
    result = dict(result, can_interface=cfg['can_interface'],arm_label=cfg['arm_label'],
                  measurement_mode=cfg['mode'])
    distance = result.get('clearance_mm')
    display = {'机械臂': cfg['arm_label'] or cfg['can_interface'], 'CAN接口':cfg['can_interface'],
        '模式':'不标定／模型估算' if cfg['mode']=='model_estimate' else '实测标定',
        '数据状态':{'estimated':'估算中（未实测标定）','ok':'标定参数计算有效','idle':'已停止',
                    'unavailable':'不可用'}.get(result.get('state'),result.get('state')),
        '离桌间隙':f'{distance:.1f} mm' if distance is not None else '暂无有效距离',
        '夹爪开度':f"{result['opening_mm']:.1f} mm" if result.get('opening_mm') is not None else '暂无有效开度',
        '最近部件':PARTS.get(result.get('closest_part'),result.get('closest_part','未提供')),
        '反馈龄':f"{max(0,now-result['feedback_time'])*1000:.0f} ms" if result.get('feedback_time') is not None else '未知',
        '自动防碰桌':'未启用（仅监测）'}
    reasons={'measurement_stopped':'测距已暂停，请点击开始测距',
        'calibration_unverified':'尚未完成实测标定确认','waiting_for_feedback':'等待有效反馈',
        'feedback_expired':'反馈已过期','model_estimate_not_calibrated':'使用模型估算，尚未实测标定',
        'missing_or_stale_feedback':'反馈缺失、过期或时间戳无效','joint_reference_range':'关节角度超出模型参考范围',
        'opening_out_of_model_range':'开度超出模型范围','invalid_feedback':'反馈读取失败或格式无效',
        'estimate_source_unavailable':'所选机械臂来源不可用','disconnected':'机械臂反馈未连接',
        'calibration_arm_mismatch':'标定接口与反馈接口不匹配','nonfinite_geometry':'几何计算超出有限数值范围',
        'calibration_interface_mismatch':'实测标定绑定的接口与测距来源不匹配',
        'publish_failed':'数据发布失败','pose_gripper_time_skew':'位姿与开度反馈时间不一致',
        'joint_gripper_time_skew':'关节与开度反馈时间不一致','opening_outside_calibrated_range':'开度超出标定范围'}
    display['状态说明']=reasons.get(result.get('reason'),result.get('reason') or '有效')
    return {'读数':display,**result,'字段说明':{k:FIELDS.get(k,'诊断字段') for k in result}}
