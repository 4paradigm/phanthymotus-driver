"""RealMan-compatible transfer inputs and preflight paths for the FR3 wrist."""
import math
import numpy as np

from .geometry import locate, transform
from .kinematics import Kinematics
from .planning import rotation
from .trajectory import samples


def number(args, name, limit=None, default=None):
    value = args.get(name, default)
    if (type(value) not in (int,float) or not math.isfinite(value)
            or (limit is not None and abs(value)>limit)):
        raise ValueError(f'{name} must be a finite number'+(f' in [-{limit}, {limit}]' if limit is not None else ''))
    return float(value)


def parameters(action, args):
    start = [number(args,k,1) for k in ('start_point_x','start_point_y')]
    angle = number(args,'rotation_deg',180,0)
    if action == 'grab_by':
        delta = [number(args,k) for k in ('delta_x','delta_y')]
        if not any(delta) and not angle:
            raise ValueError('At least one of delta_x, delta_y and rotation_deg must be nonzero')
        return dict(position=start,rotation_deg=angle,displacement_mm=delta)
    return dict(position=start,rotation_deg=angle,
                target_position=[number(args,k,1) for k in ('target_point_x','target_point_y')])


def image_table_axes(camera, normal):
    right = camera[:3,0]-normal*(camera[:3,0]@normal)
    if np.linalg.norm(right)<1e-6:
        raise ValueError('Observation image right is perpendicular to the table')
    right /= np.linalg.norm(right)
    down = np.cross(normal,right)
    if down@camera[:3,1]<0: down=-down
    return np.column_stack((right, down))


def destination(observation, scene, handeye, options):
    normal = np.asarray(scene['table_normal_base'],dtype=float)
    table = np.asarray(scene['table_point_base_mm'])/1000
    source = np.asarray(scene['centre_base_mm'])/1000
    actual = np.asarray(observation['metadata']['O_T_EE']).reshape(4,4,order='F')
    camera = actual @ transform(handeye['EE_T_camera'],'EE_T_camera')
    if 'displacement_mm' in options:
        delta = image_table_axes(camera, normal) @ np.asarray(options['displacement_mm']) / 1000
    else:
        surface = locate(observation,options['target_position'],handeye)
        ray = np.asarray(surface['base_xyz_mm'])/1000-camera[:3,3]
        denominator = ray@normal
        if denominator>=-1e-6:
            raise ValueError('Placement ray does not point toward the table')
        target = camera[:3,3]+ray*((table-camera[:3,3])@normal/denominator)
        delta = target-(source-normal*((source-table)@normal))
    return delta


def plan_transfer(observation, pick, handeye, options, check=None, profile=None):
    """Check the entire transfer and return before the first pick command."""
    check = check or (lambda: None)
    scene = pick['scene']; normal=np.asarray(scene['table_normal_base'],dtype=float)
    table=np.asarray(scene['table_point_base_mm'])/1000
    qs=np.asarray(pick['joint_path_rad']); actual=np.asarray(observation['metadata']['O_T_EE']).reshape(4,4,order='F')
    model=Kinematics(qs[0],actual)
    delta=destination(observation,scene,handeye,options)
    q=qs[-1].copy(); start=model.pose(q)
    pad=pick['sdk_to_pad_offset_mm']/1000
    base_grasp=np.asarray(pick['sdk_grasp_base_mm'])/1000
    # Rotate around the physical grasp centre, after raising, before translating.
    centre=start[:3,3]+pad*start[:3,2]+normal*.05
    final_r=rotation(-normal,math.radians(options['rotation_deg']))@start[:3,:3]
    landing=base_grasp+pad*start[:3,2]+delta-pad*final_r[:,2]
    paths=[]; stages=[]
    def cart(stage, goals):
        nonlocal q
        controls=[q.copy()]
        for goal,count in goals:
            origin=model.pose(q)
            relative=goal[:3,:3]@origin[:3,:3].T
            angle=math.acos(np.clip((np.trace(relative)-1)/2,-1,1))
            # All transfer rotations are about the downward tool axis.
            signed=math.atan2((-normal)@np.array([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/2,
                              (np.trace(relative)-1)/2)
            for t in np.linspace(0,1,count+1)[1:]:
                check(); g=goal.copy();g[:3,3]=origin[:3,3]*(1-t)+goal[:3,3]*t
                g[:3,:3]=rotation(-normal,signed*t)@origin[:3,:3] if angle>1e-6 else origin[:3,:3]
                q=model.solve(q,g);controls.append(q.copy())
        paths.append(np.asarray(controls));stages.append(stage)
    def pose(point,r):
        t=np.eye(4);t[:3,:3]=r;t[:3,3]=point;return t
    goals=[(pose(centre-pad*start[:3,2],start[:3,:3]),6)]
    if options['rotation_deg']:
        goals.append((pose(centre-pad*final_r[:,2],final_r),4))
    goals.extend([(pose(centre+delta-pad*final_r[:,2],final_r),4 if options['rotation_deg'] else 6),
                  (pose(landing,final_r),6 if options['rotation_deg'] else 8)])
    cart('move_to_place',goals)
    cart('place_lift',[(pose(landing+normal*.15,final_r),20)])
    from .hand_collision import observation_path
    return_profile = dict(profile or {})
    return_profile.setdefault('table_plane', dict(
        point_base_mm=(table*1000).tolist(), normal_base=normal.tolist()))
    paths.append(observation_path(dict(q=q.tolist(), O_T_EE=model.pose(q).flatten(order='F').tolist(),
                                      F_T_EE=observation['metadata'].get('F_T_EE')),
                                  np.degrees(qs[0]), return_profile, lifts=(0., .05, .10)))
    stages.append('return_to_observation')
    from .hand_collision import frame, vertices, check_path_table
    hand_points = vertices(frame(profile or {}, observation['metadata'].get('F_T_EE')))
    gaps=[]
    for stage,path in zip(stages,paths):
        check();curve=samples(path,40)
        if not np.all((curve>=model.low)&(curve<=model.high)):
            raise ValueError('Transfer spline exceeds joint limits')
        for value in curve:
            check();gaps.append(model.clearance(value));ee=model.pose(value)
            check_path_table(model, value, hand_points, table, normal, stage, profile or {})
            if stage!='return_to_observation' and ee[:3,2]@(-normal)<math.cos(math.radians(3)):
                raise ValueError('Transfer spline exceeds the vertical grasp tolerance')
    if min(gaps)<=.02:
        raise ValueError('Transfer spline fails arm self-collision clearance')
    meta=observation['metadata'];width,height=meta['width'],meta['height']
    def pixel(pos):return [min(round((v+1)*size/2),size-1) for v,size in zip(pos,(width,height))]
    result=dict(pick_pixel=pixel(options['position']),
                pick_base_xy_mm=scene['centre_base_mm'][:2],
                place_base_xy_mm=(np.asarray(scene['centre_base_mm'])+delta*1000)[:2].tolist(),
                rotation_deg=options['rotation_deg'],direction_reference='observation_image')
    if 'displacement_mm' in options:
        result.update(zip(('delta_x','delta_y'),options['displacement_mm']))
    else:result['place_pixel']=pixel(options['target_position'])
    return dict(paths=[p.tolist() for p in paths],stages=stages,result=result,
                arm_capsule_clearance_mm=min(gaps)*1000,checked_samples=len(gaps))
