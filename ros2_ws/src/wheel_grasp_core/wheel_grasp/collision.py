"""Conservative URDF collision OBBs; no claim of mesh-exact collision checking.

Meshes are bounded by local boxes, spheres/cylinders by enclosing boxes. This
can reject safe paths but does not shrink collision geometry. Only same-link
and directly joint-connected pairs are excluded by default.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .kinematics import pose


@dataclass
class Box:
    name: str
    transform: np.ndarray
    half: np.ndarray

    def __post_init__(self):
        self.transform=pose(self.transform)
        self.half=np.asarray(self.half,float)
        if self.half.shape!=(3,) or not np.isfinite(self.half).all() or np.any(self.half<=0):
            raise ValueError('Invalid collision box')


def overlaps(a,b,margin=0.):
    if margin<0: raise ValueError('Negative collision margin')
    aa=a.transform[:3,:3];bb=b.transform[:3,:3]
    delta=b.transform[:3,3]-a.transform[:3,3]
    axes=[*aa.T,*bb.T]+[np.cross(x,y) for x in aa.T for y in bb.T]
    for axis in axes:
        norm=np.linalg.norm(axis)
        if norm<1e-10: continue
        axis=axis/norm
        ra=np.sum((a.half+margin)*np.abs(aa.T@axis))
        rb=np.sum((b.half+margin)*np.abs(bb.T@axis))
        if abs(delta@axis)>ra+rb: return False
    return True


def origin(node):
    t=np.eye(4)
    if node is not None:
        t[:3,3]=np.fromstring(node.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(node.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t


class URDFCollision:
    def __init__(self,urdf,robot,world_boxes,*,base='base_footprint',allowed_pairs=(),margin=.005):
        self.robot=robot;self.base=base;self.world=list(world_boxes);self.margin=margin;self.parts=[]
        self.ignored={frozenset(p) for p in allowed_pairs}
        self.ignored|={frozenset((j['parent'],j['child'])) for j in robot.joints.values()}
        self.last_collision=None
        path=Path(urdf);root=ET.parse(path).getroot()
        for link in root.findall('link'):
            name=link.get('name')
            for idx,c in enumerate(link.findall('collision')):
                g=c.find('geometry');t=origin(c.find('origin'))
                box=g.find('box');sphere=g.find('sphere');cylinder=g.find('cylinder');mesh=g.find('mesh')
                if box is not None: half=np.fromstring(box.get('size'),sep=' ')/2
                elif sphere is not None: half=np.repeat(float(sphere.get('radius')),3)
                elif cylinder is not None: half=np.array([float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2])
                elif mesh is not None:
                    import trimesh
                    filename=mesh.get('filename')
                    if '://' in filename: raise ValueError('Resolve package mesh URI explicitly: '+filename)
                    obj=trimesh.load(path.parent/filename,force='mesh')
                    scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
                    vertices=np.asarray(obj.vertices)*scale
                    lower,upper=vertices.min(axis=0),vertices.max(axis=0)
                    shift=np.eye(4);shift[:3,3]=(lower+upper)/2;t=t@shift;half=(upper-lower)/2
                else: raise ValueError('Unsupported collision geometry on '+name)
                self.parts.append((name,Box(name+':'+str(idx),t,half)))

    def __call__(self,joints):
        boxes=[]
        for name,box in self.parts:
            tf=self.robot.transform(name,joints,self.base)
            boxes.append((name,Box(box.name,tf@box.transform,box.half)))
        self.last_collision=None
        for idx,(name,a) in enumerate(boxes):
            for other,b in boxes[idx+1:]:
                if name==other or frozenset((name,other)) in self.ignored: continue
                if overlaps(a,b,self.margin):
                    self.last_collision=(a.name,b.name);return False
            for b in self.world:
                if overlaps(a,b,self.margin):
                    self.last_collision=(a.name,b.name);return False
        return True
