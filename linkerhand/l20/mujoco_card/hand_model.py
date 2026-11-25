"""Side-specific G20 geometry shared by the canvas stream and the 3D workbench."""
import base64
import hashlib
import io
import json
import math
import xml.etree.ElementTree as ET
import zipfile
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw
from joints_model import NEGATED_TABLES

ARCHIVE_SHA = 'f5558bc0aa0d70503cd21611efc9e446592aac535dd68c9e61f71754004b2327'


# Ordered from thumb to little finger, shared by both renderers and both hands.
TACTILE_PADS = {
    'thumb_distal': ('拇指', (45, 205, 178)),
    'index_distal': ('食指', (255, 163, 66)),
    'middle_distal': ('中指', (91, 157, 250)),
    'ring_distal': ('无名指', (193, 126, 240)),
    'pinky_distal': ('小指', (244, 212, 70)),
}
SHELL_COLOR = (181, 193, 190)


def pad_surface(model, geom, name, vertices, faces):
    """Mark the ridged palmar cap in the manufacturer's distal-link frame.

    The bundled STL has no taxel material. The oblique lower edge and half
    width follow the cap profile in the thumb and common four-finger meshes.
    MuJoCo recentres each mesh, so convert back before classifying faces.
    """
    rotation = np.empty(9)
    mj.mju_quat2Mat(rotation, model.geom_quat[geom])
    local_mm = (vertices @ rotation.reshape(3, 3).T + model.geom_pos[geom]) * 1000
    triangles = local_mm[faces]
    x, y, z = triangles.mean(axis=1).T
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    slope, edge, half_width = (.85, 19.5, 8.3) if name == 'thumb_distal' else (.8, 6.8, 7.0)
    outward = normals[:, 0] * slope + normals[:, 2]
    return (z + slope * x > edge) & (np.abs(y) < half_width) & (outward > 0.05 * np.linalg.norm(normals, axis=1))


def packed(array, dtype):
    return base64.b64encode(np.ascontiguousarray(array, dtype=dtype).tobytes()).decode()


def simplify(model, geom, grid):
    idx = model.geom_dataid[geom]
    addr = model.mesh_vertadr[idx]
    verts = model.mesh_vert[addr:addr + model.mesh_vertnum[idx]]
    addr = model.mesh_faceadr[idx]
    faces = model.mesh_face[addr:addr + model.mesh_facenum[idx]]
    _, inv = np.unique(np.rint(verts / grid).astype(int), axis=0, return_inverse=True)
    count = np.bincount(inv)
    verts = np.column_stack([np.bincount(inv, weights=x) / count for x in verts.T])
    faces = inv[faces]
    faces = faces[(faces[:,0]!=faces[:,1]) & (faces[:,0]!=faces[:,2]) & (faces[:,1]!=faces[:,2])]
    _, keep = np.unique(np.sort(faces,axis=1),axis=0,return_index=True)
    return verts, faces[keep]


class HandModel:
    def __init__(self, archive, side):
        if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA:
            raise ValueError('G20 asset checksum mismatch')
        prefix = 'L20/'+side.upper()+'/'
        with zipfile.ZipFile(archive) as z:
            urdf = ET.fromstring(z.read(prefix+f'linkerhand_g20_{side}.urdf'))
            cal = json.loads(z.read(prefix+f'g20_{side}_G20.json'))['joints']
            assets = {n.removeprefix(prefix):z.read(n) for n in z.namelist() if n.startswith(prefix+'meshes/') and n.lower().endswith('.stl')}
        for link in urdf.findall('link'):
            for tag in ('visual','collision'):
                for i,g in enumerate(link.findall(tag)):
                    g.set('name', f'{link.get("name")}_{tag}_{i}')
        ET.SubElement(ET.SubElement(urdf,'mujoco'),'compiler',balanceinertia='true',strippath='false',fusestatic='false')
        self.model = m = mj.MjModel.from_xml_string(ET.tostring(urdf,encoding='unicode'),assets)
        names = [m.joint(i).name for i in range(m.njnt)]
        self.indices = np.array([cal[n]['motor_index'] for n in names])
        self.angles = np.array([np.asarray(cal[n]['angle_rad'])*(-1 if n in NEGATED_TABLES[side] else 1) for n in names])
        self.geoms = [i for i in range(m.ngeom) if m.geom_type[i] == mj.mjtGeom.mjGEOM_MESH]
        self.meshes, self.draw_meshes = [], []
        for g in self.geoms:
            name = m.body(m.geom_bodyid[g]).name
            pad = TACTILE_PADS.get(name)
            v, f = simplify(m, g, .0019 if pad else .0032)
            mesh = {'name': name, 'vertices': packed(np.rint(v*100000), '<i2'), 'faces': packed(f, '<u2')}
            if pad:
                mesh['pad_faces'] = packed(pad_surface(m, g, name, v, f), 'u1')
                mesh['pad_color'] = pad[1]
                mesh['pad_label'] = pad[0]
            self.meshes.append(mesh)

            v, f = simplify(m, g, .0025 if pad else .005)
            colors = np.tile(SHELL_COLOR, (len(f), 1)).astype(np.uint8)
            if pad:
                colors[pad_surface(m, g, name, v, f)] = pad[1]
            self.draw_meshes.append((v, f, colors))
        self.side=side

    def pose(self,data,raw):
        m=self.model
        a=self.angles[np.arange(m.njnt),np.asarray(raw)[self.indices]]
        data.qpos[m.jnt_qposadr]=np.clip(a,m.jnt_range[:,0],m.jnt_range[:,1])
        mj.mj_forward(m,data)

    def transforms(self,raw):
        d=mj.MjData(self.model)
        self.pose(d,raw)
        quats=np.empty((len(self.geoms),4))
        for i,g in enumerate(self.geoms):
            mj.mju_mat2Quat(quats[i],d.geom_xmat[g])
        return np.column_stack((d.geom_xpos[self.geoms],quats)).round(6).tolist()

    def jpeg(self,raw):
        d=mj.MjData(self.model); self.pose(d,raw)
        az=-.52 if self.side=='right' else .52
        ca,sa,ce,se=math.cos(az),math.sin(az),math.cos(.25),math.sin(.25)
        camera=np.array([[-sa,ca,0],[-se*ca,-se*sa,ce],[ce*ca,ce*sa,se]])
        polys, colors = [], []
        for g,(v,f,base) in zip(self.geoms,self.draw_meshes):
            pts=(v @ d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]-[.035,.015 if self.side=='right' else -.015,.138]) @ camera.T
            pts *= [980,-980,980];pts += [202,145,0]
            tris=pts[f];normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
            visible=normals[:,2]<-.025;tris=tris[visible];normals=normals[visible]
            light=.48+.52*np.abs(normals @ [.29,.47,-.83])/np.maximum(np.linalg.norm(normals,axis=1),1.e-8)
            polys.append(tris);colors.append(base[visible]*light[:,None])
        tris=np.concatenate(polys);cols=np.clip(np.concatenate(colors),0,255).astype(np.uint8)
        order=np.argsort(tris[:,:,2].mean(axis=1));xy=tris[order,:,:2].reshape(-1,6).round().astype(int).tolist()
        image=Image.new('RGB',(400,286),(21,38,45));draw=ImageDraw.Draw(image)
        for poly,col in zip(xy,cols[order].tolist()):
            draw.polygon(poly,fill=tuple(col))
        buf=io.BytesIO();image.save(buf,'JPEG',quality=78);return buf.getvalue()
