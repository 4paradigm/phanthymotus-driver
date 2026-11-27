"""Per-card Canvas form for measurement source and calibration selection."""
from copy import deepcopy


def schema(cfg):
    props = {
        'can_interface':dict(type='string',title='机械臂来源 CAN 接口',oneOf=[
            dict(const='can0',title='can0（本工位左侧从臂）'),dict(const='can1',title='can1（本工位右侧从臂）')],
            description='其他工位机请按实际接线核对左右；只切换测距来源，不更改运动控制卡。'),
        'arm_label':dict(type='string',title='机械臂显示名称',description='例如左侧从臂；仅用于标注，不自动识别左右。'),
        'mode':dict(type='string',title='是否使用实测标定',oneOf=[
            dict(const='model_estimate',title='不标定：使用内置模型估算'),
            dict(const='calibrated',title='使用实测标定参数')]),
        'table_z_mm':dict(type='number',title='桌面相对底座安装面的高度（mm）',
            description='底座安装面与桌面齐平填0；桌面比安装面高时为正。',
            **{'x-show-when':{'mode':'model_estimate'}}),
    }
    fields = {
        'verified':('boolean','我已完成该臂、桌面及夹爪几何的实测核验'),
        'calibration_id':('string','标定记录编号'), 'base_frame':('string','SDK基坐标系名称'),
        'calibration_can_interface':('string','该份实测标定对应的 CAN 接口（必须与来源一致）'),
        'plane_base_mm':('array','实测桌面平面 [nx,ny,nz,d_mm]（朝上单位法线）'),
        'body_vertices_mm':('array','固定夹爪本体凸包顶点 [x,y,z]（mm）'),
        'left_finger_vertices_mm':('array','零开度左指凸包顶点 [x,y,z]（mm）'),
        'right_finger_vertices_mm':('array','零开度右指凸包顶点 [x,y,z]（mm）'),
        'opening_axis':('array','SDK末端坐标系中的单位开合轴 [x,y,z]'),
        'opening_range_mm':('array','已核验开度范围 [最小,最大]（mm）'),
        'uncertainty_mm':('number','实测总误差余量（mm）'),
    }
    for key,(kind,title) in fields.items():
        props[key]=dict(type=kind,title='仅实测标定：'+title,advanced=True,
            description='仅在实测标定模式生效。估算模式不会使用这些核验参数。',
            **{'x-show-when':{'mode':'calibrated'}})
        if kind=='array':props[key]['items']={}
    props['calibration_can_interface']['oneOf']=[dict(const='can0',title='can0 的实测标定'),
                                                dict(const='can1',title='can1 的实测标定')]
    for key,prop in props.items():
        prop['scope']='instance'
        if cfg.get(key) is not None:prop['default']=deepcopy(cfg[key])
    props['table_z_mm'].setdefault('default',0)
    return dict(type='object',properties=props,additionalProperties=False)
