"""URDF-only installation transforms, strict live joint requirements and pair IK."""
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


from wheel_task_core.transforms import pose


class RobotModel:
    def __init__(self,path):
        root=ET.parse(path).getroot()
        self.links={x.get('name') for x in root.findall('link')}
        self.parents={}
        self.joints={}
        for j in root.findall('joint'):
            name,kind=j.get('name'),j.get('type')
            parent,child=j.find('parent').get('link'),j.find('child').get('link')
            if child in self.parents or name in self.joints or parent not in self.links or child not in self.links:
                raise ValueError('Malformed URDF tree')
            origin=j.find('origin');t=np.eye(4)
            if origin is not None:
                t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            ax=j.find('axis')
            axis=np.fromstring(ax.get('xyz') if ax is not None else '1 0 0',sep=' ')
            if kind!='fixed':
                if np.linalg.norm(axis)==0: raise ValueError('Zero joint axis')
                axis=axis/np.linalg.norm(axis)
            lim=j.find('limit');bounds=None
            if lim is not None and kind!='continuous':
                bounds=(float(lim.get('lower')),float(lim.get('upper')))
            mimic=j.find('mimic')
            spec=dict(name=name,kind=kind,parent=parent,child=child,t=t,axis=axis,bounds=bounds,
                      velocity=float(lim.get('velocity','inf')) if lim is not None else np.inf,
                      mimic=None if mimic is None else (mimic.get('joint'),float(mimic.get('multiplier',1)),float(mimic.get('offset',0))))
            self.parents[child]=spec;self.joints[name]=spec

    def chain(self,frame,base='base_footprint'):
        if frame not in self.links or base not in self.links: raise ValueError('Unknown URDF frame')
        chain=[];seen=set()
        while frame!=base:
            if frame in seen or frame not in self.parents: raise ValueError('Frame not below requested base')
            seen.add(frame);j=self.parents[frame];chain.append(j);frame=j['parent']
        return list(reversed(chain))

    def transform(self,frame,values,base='base_footprint'):
        def value(j,seen=()):
            name=j['name']
            if name in seen: raise ValueError('Cyclic mimic joint')
            if j['mimic']:
                src,mult,offset=j['mimic'];return value(self.joints[src],seen+(name,))*mult+offset
            if name not in values: raise ValueError('Missing live joint: '+name)
            v=float(values[name])
            if not np.isfinite(v): raise ValueError('Nonfinite joint: '+name)
            if j['bounds'] and not j['bounds'][0]-1e-7<=v<=j['bounds'][1]+1e-7:
                raise ValueError('Joint outside URDF limits: '+name)
            return v
        t=np.eye(4)
        for j in self.chain(frame,base):
            motion=np.eye(4)
            if j['kind']=='prismatic': motion[:3,3]=j['axis']*value(j)
            elif j['kind'] in ('revolute','continuous'):
                motion[:3,:3]=Rotation.from_rotvec(j['axis']*value(j)).as_matrix()
            elif j['kind']!='fixed': raise ValueError('Unsupported joint type: '+j['kind'])
            t=t@j['t']@motion
        return t


@dataclass
class PairSolution:
    joints: dict
    position_errors: dict
    rotation_errors: dict
    success: bool


class PairIK:
    def __init__(self,model,tcp,*,base='base_footprint',position_tolerance=.004,angle_tolerance=.035):
        self.model=model;self.base=base
        self.tcp={s:pose(tcp[s]) for s in ('left','right')}
        self.names=['lift_joint']+[f'openarm_{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
        self.lower=np.array([model.joints[n]['bounds'][0] for n in self.names])
        self.upper=np.array([model.joints[n]['bounds'][1] for n in self.names])
        self.ptol=position_tolerance;self.atol=angle_tolerance

    def fk(self,values):
        return {s:self.model.transform(f'openarm_{s}_ee_base_link',values,self.base)@self.tcp[s] for s in self.tcp}

    def solve(self,targets,measured,*,lift_bounds=None,fixed_lift=None,max_nfev=200):
        if set(targets)!=set(self.tcp): raise ValueError('Both hand targets required')
        targets={s:pose(t) for s,t in targets.items()}
        self.fk(measured)  # Reject invalid live feedback instead of clipping it.
        seed=np.array([measured[n] for n in self.names],float)
        low,high=self.lower.copy(),self.upper.copy()
        if lift_bounds is not None:
            low[0]=max(low[0],lift_bounds[0]);high[0]=min(high[0],lift_bounds[1])
        if fixed_lift is not None:
            if not low[0]<=fixed_lift<=high[0]: raise ValueError('Lift outside feasible range')
            low[0]=high[0]=fixed_lift
        if np.any(low>high): raise ValueError('Empty joint limit intersection')
        free=np.flatnonzero(high-low>1e-10)
        initial=np.clip(seed,low,high)
        def unpack(x):
            q=initial.copy();q[free]=x;return dict(zip(self.names,q))
        def residual(x):
            values=unpack(x);actual=self.fk(values);r=[]
            for side in targets:
                r.extend(actual[side][:3,3]-targets[side][:3,3])
                r.extend(.20*Rotation.from_matrix(targets[side][:3,:3]@actual[side][:3,:3].T).as_rotvec())
            r.extend(.001*(np.array([values[n] for n in self.names])-seed))
            return r
        if len(free):
            result=least_squares(residual,initial[free],bounds=(low[free],high[free]),max_nfev=max_nfev,
                                 ftol=1e-9,xtol=1e-9,gtol=1e-9)
            values=unpack(result.x)
        else: values=unpack([])
        actual=self.fk(values)
        pe={s:float(np.linalg.norm(actual[s][:3,3]-targets[s][:3,3])) for s in targets}
        ae={s:float(Rotation.from_matrix(targets[s][:3,:3]@actual[s][:3,:3].T).magnitude()) for s in targets}
        return PairSolution(values,pe,ae,max(pe.values())<=self.ptol and max(ae.values())<=self.atol)
