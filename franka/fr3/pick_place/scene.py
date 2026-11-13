"""Estimate a table and a spherical target from one saved aligned depth image."""
from collections import deque

import numpy as np

from .alignment import decode_depth
from .geometry import locate, transform


def estimate(observation, position, handeye):
    surface = locate(observation, position, handeye)
    metadata = observation['metadata']
    intr = metadata['calibration']['rgb_intrinsics']
    depth = decode_depth(observation['depth_zlib'], metadata['width'], metadata['height'])
    rows, columns = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
    valid = (depth[rows, columns] > 100) & (depth[rows, columns] < 2000)
    columns, rows = columns[valid], rows[valid]
    z = depth[rows, columns].astype(float)/1000
    if any(intr['coeffs']):
        import pyrealsense2 as rs
        optics = rs.intrinsics()
        for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy', 'coeffs'):
            setattr(optics, key, intr[key])
        optics.model = getattr(rs.distortion, intr['model'].rsplit('.', 1)[-1])
        points = np.array([rs.rs2_deproject_pixel_to_point(optics, [float(x), float(y)], float(d))
                           for x, y, d in zip(columns, rows, z)])
    else:
        points = np.column_stack(((columns-intr['ppx'])*z/intr['fx'],
                                  (rows-intr['ppy'])*z/intr['fy'], z))
    ee = transform(np.asarray(metadata['O_T_EE']).reshape(4, 4, order='F'), 'O_T_EE')
    camera = ee @ transform(handeye['EE_T_camera'], 'EE_T_camera')
    points = points @ camera[:3, :3].T + camera[:3, 3]
    target = np.asarray(surface['base_xyz_mm'])/1000
    outline = None
    if observation.get('jpeg'):
        import cv2
        image = cv2.imdecode(np.frombuffer(observation['jpeg'],np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != depth.shape:
            raise ValueError('Observation RGB/depth dimensions differ')
        x, y = surface['pixel']
        chroma = cv2.cvtColor(image,cv2.COLOR_BGR2LAB)[:,:,1:].astype(float)
        seed = np.median(chroma[max(0,y-5):y+6,max(0,x-5):x+6].reshape(-1,2),axis=0)
        yy, xx = np.mgrid[:depth.shape[0],:depth.shape[1]]
        radius_pixels = intr['fx']*.075/(surface['depth_mm']/1000)
        mask = ((np.linalg.norm(chroma-seed,axis=2)<20)
                & ((xx-x)**2+(yy-y)**2 < radius_pixels**2)).astype(np.uint8)
        outline = dict(colour_mask=mask[rows,columns].astype(bool),
                       pixels=np.column_stack((columns//2,rows//2)),
                       shape=(depth.shape[0]//2,depth.shape[1]//2),
                       seed=(x//2,y//2), selection_radius=radius_pixels/2,
                       camera=camera, intrinsics=intr, closing_axis=ee[:3,1])
    return fit_points(points, target, camera[:3, 3], np.column_stack((columns//2, rows//2)), outline)


def fit_points(points, target, camera_origin, pixels=None, outline=None):
    """Fit geometry from already deprojected base-frame points."""
    points = np.asarray(points, dtype=float)
    target = np.asarray(target, dtype=float)
    camera_origin = np.asarray(camera_origin, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or target.shape != (3,) or camera_origin.shape != (3,)
            or not np.isfinite(target).all() or not np.isfinite(camera_origin).all()):
        raise ValueError('Invalid base-frame point cloud')
    distance = np.linalg.norm(points-target, axis=1)
    background = points[(distance > .075) & (distance < .4)]
    if len(background) < 100:
        raise ValueError('Insufficient surrounding depth to estimate the table')
    rng = np.random.default_rng(42)
    if len(background) > 3000:
        background = background[rng.choice(len(background), 3000, replace=False)]
    best = np.zeros(len(background), dtype=bool)
    for _ in range(200):
        a, b, c = background[rng.choice(len(background), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        inliers = np.abs((background-a) @ normal) < .003
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < max(100, .3*len(background)):
        raise ValueError('No supported table plane in the observation')
    table_points = background[best]
    table = table_points.mean(axis=0)
    _, _, axes = np.linalg.svd(table_points-table, full_matrices=False)
    normal = axes[-1]
    if (camera_origin-table) @ normal < 0:
        normal = -normal
    if (target-table) @ normal < .015:
        raise ValueError('Selected target is not sufficiently above the table')
    # Use the object's upper surface to avoid table grooves and contact-edge noise.
    target_height = float((target-table) @ normal)
    foreground_height = max(.008, target_height*.3)
    selected_mask = (distance < .075) & ((points-table) @ normal > foreground_height)
    if outline is not None:
        return fit_outline(points, target, table, normal, outline)

    object_points = points[selected_mask]
    if pixels is not None and len(object_points):
        # Keep the connected target surface, excluding nearby raised table details.
        coordinates = np.asarray(pixels)[selected_mask]
        lookup = {tuple(pixel): i for i, pixel in enumerate(coordinates)}
        seed = int(np.argmin(np.linalg.norm(object_points-target, axis=1)))
        reached, queue = {seed}, deque([seed])
        while queue:
            current = queue.popleft()
            x, y = coordinates[current]
            for dx, dy in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                neighbour = lookup.get((x+dx,y+dy))
                if (neighbour is not None and neighbour not in reached
                        and np.linalg.norm(object_points[current]-object_points[neighbour]) < .008):
                    reached.add(neighbour)
                    queue.append(neighbour)
        object_points = object_points[sorted(reached)]
    if len(object_points) < 60:
        raise ValueError('Insufficient target depth for spherical fitting')
    best = np.zeros(len(object_points), dtype=bool)
    for _ in range(400):
        sample = object_points[rng.choice(len(object_points), 4, replace=False)]
        matrix = 2*(sample[1:]-sample[0])
        if np.linalg.cond(matrix) > 1e4:
            continue
        centre = np.linalg.solve(matrix, (sample[1:]**2).sum(axis=1)-(sample[0]**2).sum())
        radius = np.linalg.norm(sample[0]-centre)
        if not .01 < radius < .04:
            continue
        inliers = np.abs(np.linalg.norm(object_points-centre, axis=1)-radius) < .0025
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < max(60, .5*len(object_points)):
        raise ValueError('Selected depth does not support a spherical object')
    selected = object_points[best]
    matrix = np.column_stack((2*selected, np.ones(len(selected))))
    coefficients = np.linalg.lstsq(matrix, (selected**2).sum(axis=1), rcond=None)[0]
    centre = coefficients[:3]
    radius_squared = coefficients[3]+centre @ centre
    if radius_squared <= 0:
        raise ValueError('Invalid fitted sphere')
    radius = np.sqrt(radius_squared)
    if not .01 < radius < .0397:
        raise ValueError('Estimated target is outside the gripper opening')
    height_radius = radius
    residual = np.median(abs(np.linalg.norm(selected-centre, axis=1)-radius))
    model = 'sphere'
    if best.mean() < .7 or abs((centre-table) @ normal-radius) > .008:
        # Fit an upright spheroid whose bottom rests on the measured table.
        horizontal = np.cross(normal, [0., 0., 1.])
        if np.linalg.norm(horizontal) < .1:
            horizontal = np.cross(normal, [1., 0., 0.])
        horizontal /= np.linalg.norm(horizontal)
        basis = np.column_stack((horizontal, np.cross(normal, horizontal), normal))
        local = (object_points-table) @ basis
        anchor = (centre-table) @ basis
        parameters = np.array([anchor[0], anchor[1], radius, max(radius, anchor[2])])

        def errors(values):
            x, y, width, height = values
            delta = local-[x, y, height]
            return (np.sqrt((delta[:,0]**2+delta[:,1]**2)/width**2
                            +delta[:,2]**2/height**2)-1)*width

        for _ in range(100):
            error = errors(parameters)
            jacobian = np.column_stack([(errors(parameters+d)-error)/1e-5
                                        for d in np.eye(4)*1e-5])
            weights = np.minimum(1, .002/np.maximum(abs(error), 1e-9))
            step = np.linalg.solve(jacobian.T @ (weights[:,None]*jacobian)+1e-5*np.eye(4),
                                   -jacobian.T @ (weights*error))
            parameters += np.clip(step, -.002, .002)
            parameters[2:] = np.clip(parameters[2:], [.01,.01], [.05,.06])
            if np.linalg.norm(step) < 1e-7:
                break
        residual = float(np.median(abs(errors(parameters))))
        if residual > .002 or np.quantile(abs(errors(parameters)), .9) > .005:
            raise ValueError('Target shape cannot be fitted consistently with the table')
        radius, height_radius = parameters[2:]
        if radius >= .0397:
            raise ValueError('Estimated target is outside the gripper opening')
        centre = table+basis @ [parameters[0], parameters[1], height_radius]
        model = 'table_supported_spheroid'
    return dict(centre_base_mm=(centre*1000).tolist(), diameter_mm=float(2*radius*1000),
                table_normal_base=normal.tolist(), table_point_base_mm=(table*1000).tolist(),
                fit_residual_mm=float(residual*1000), height_mm=float(2*height_radius*1000),
                model=model, object_inlier_fraction=float(best.mean()))


def fit_outline(points, target, table, normal, outline):
    """Estimate a round object's enclosing width and height without fitting its surface."""
    import cv2
    grid=np.zeros(outline['shape'],np.uint8)
    coordinates=outline['pixels']
    above=(points-table) @ normal > max(.01,float((target-table) @ normal)*.2)
    selected_mask=outline['colour_mask'] & above
    grid[coordinates[selected_mask,1],coordinates[selected_mask,0]]=1
    count,labels,stats,_=cv2.connectedComponentsWithStats(grid)
    x,y=outline['seed']
    label=labels[y,x]
    if label==0 or stats[label,cv2.CC_STAT_AREA]<25:
        raise ValueError('No connected target outline above the table')
    region=np.uint8(labels==label)
    contours,_=cv2.findContours(region,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    hull=cv2.convexHull(np.concatenate(contours))
    contour=hull[:,0,:]
    if (contour[:,0].min()<=0 or contour[:,1].min()<=0
            or contour[:,0].max()>=grid.shape[1]-1 or contour[:,1].max()>=grid.shape[0]-1
            or np.max(np.linalg.norm(contour-[x,y],axis=1))>outline['selection_radius']*.95):
        raise ValueError('Target outline reaches the selection boundary')
    selected=points[labels[coordinates[:,1],coordinates[:,0]]==label]
    outline['contour']=contour*2
    if len(selected) < 100:
        raise ValueError('Insufficient depth inside the target outline')
    heights = (selected-table) @ normal
    height = float(np.quantile(heights,.99))
    if not np.isfinite(height) or height <= 0:
        raise ValueError('Target height is inconsistent with the table')
    intr = outline['intrinsics']
    if any(intr['coeffs']):
        raise ValueError('Outline projection requires rectified RGB intrinsics')
    pixels = outline['contour']
    rays = np.column_stack(((pixels[:,0]-intr['ppx'])/intr['fx'],
                            (pixels[:,1]-intr['ppy'])/intr['fy'],np.ones(len(pixels))))
    camera = outline['camera']
    rays = rays @ camera[:3,:3].T
    denominator = rays @ normal
    if np.any(denominator>=-1e-6):
        raise ValueError('Target outline does not project toward the table')
    distance = (height/2+(table-camera[:3,3]) @ normal)/denominator
    footprint = camera[:3,3]+distance[:,None]*rays
    axis = np.cross(normal,[0.,0.,1.])
    if np.linalg.norm(axis)<.1:
        axis=np.cross(normal,[1.,0.,0.])
    axis/=np.linalg.norm(axis)
    basis=np.column_stack((axis,np.cross(normal,axis)))
    coordinates=(footprint-table) @ basis
    low,high=coordinates.min(axis=0),coordinates.max(axis=0)
    closing_axis=outline['closing_axis']
    tangent=closing_axis-normal*(closing_axis @ normal)
    length=np.linalg.norm(tangent)
    if length<.5:
        raise ValueError('Closing direction is too steep for this table grasp')
    tangent/=length
    widths=[np.ptp(footprint @ (basis @ [np.cos(a),np.sin(a)]))
            for a in np.radians(np.arange(0,180,5))]
    diameter=float(min(widths))
    if not .02<diameter<.0794:
        raise ValueError(f'Outline width {diameter*1000:.1f} mm is outside the calibrated opening')
    centre=table+basis @ ((low+high)/2)+normal*height/2
    if np.linalg.norm(centre-target)>.1:
        raise ValueError('Target outline and selected depth disagree')
    return dict(centre_base_mm=(centre*1000).tolist(),diameter_mm=diameter*1000,
                height_mm=height*1000,table_normal_base=normal.tolist(),
                table_point_base_mm=(table*1000).tolist(),model='convex_image_envelope',
                outline_widths_mm=((high-low)*1000).tolist(),
                footprint_base_mm=(footprint*1000).tolist(),
                visible_surface_base_mm=(selected*1000).tolist())
