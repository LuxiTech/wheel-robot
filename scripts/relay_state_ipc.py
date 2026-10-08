"""Latest-state IPC on shared tmpfs. Neither side ever waits for the other."""
import fcntl
import json
import mmap
import os
import struct
import time
from pathlib import Path

CAPACITY=2*1024*1024
HEADER=struct.Struct('<8sQQ')
MAGIC=b'RELAY001'
DEFAULT=Path(os.environ.get('RELAY_STATE_PATH','/workspace/relay_ipc/state.bin'))

class StateChannel:
    def __init__(self,path=DEFAULT,writer=False):
        self.path=Path(path)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.file=self.path.open('a+b' if writer else 'rb')
        if writer:
            os.ftruncate(self.file.fileno(),CAPACITY)
        self.mapping=mmap.mmap(self.file.fileno(),CAPACITY,access=mmap.ACCESS_WRITE if writer else mmap.ACCESS_READ)
        self.sequence=0
        self.writer=writer
        self.skipped=0

    def publish(self,state):
        if not self.writer:raise RuntimeError('Read-only state channel')
        raw=json.dumps(state,separators=(',',':'),allow_nan=False).encode()
        if len(raw)>CAPACITY-HEADER.size:raise ValueError('State exceeds shared memory capacity')
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.skipped+=1
            return False
        try:
            self.sequence+=1
            self.mapping[HEADER.size:HEADER.size+len(raw)]=raw
            self.mapping[:HEADER.size]=HEADER.pack(MAGIC,self.sequence,len(raw))
        finally:fcntl.flock(self.file,fcntl.LOCK_UN)
        return True

    def read(self):
        try:fcntl.flock(self.file,fcntl.LOCK_SH|fcntl.LOCK_NB)
        except BlockingIOError:return None
        try:
            magic,seq,size=HEADER.unpack(self.mapping[:HEADER.size])
            if magic!=MAGIC or not 0<size<=CAPACITY-HEADER.size:return None
            raw=self.mapping[HEADER.size:HEADER.size+size]
        finally:fcntl.flock(self.file,fcntl.LOCK_UN)
        value=json.loads(raw)
        value['transport_sequence']=seq
        return value

    def close(self):
        self.mapping.close()
        self.file.close()

class PhysicsPoses:
    """Read every rigid body in one tensor call, independent of USD writeback."""
    def __init__(self, stage):
        from pxr import UsdPhysics
        self.stage = stage
        self.paths = [str(p.GetPath()) for p in stage.Traverse()
                      if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.frame_offsets = {}
        self.reset()

    def reset(self):
        import omni.physics.tensors as tensors
        from pxr import UsdUtils
        stage_id=UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simulation = tensors.create_simulation_view('numpy',stage_id=stage_id)
        self.simulation.set_subspace_roots('/')
        self.bodies = self.simulation.create_rigid_body_view(self.paths)
        self.body_paths = list(self.bodies.prim_paths)
        if set(self.body_paths) != set(self.paths) or len(self.body_paths) != len(self.paths):
            raise RuntimeError('Incomplete physical pose view; refusing stale USD fallback')
        self.indices = {path: i for i, path in enumerate(self.body_paths)}
        self.frame_offsets.clear()

    def read(self):
        import numpy as np
        poses = self.bodies.get_transforms()
        if not np.isfinite(poses).all():
            raise RuntimeError('Nonfinite physical pose snapshot')
        # Tensor API is xyzw; the viewer protocol is wxyz.
        return poses[:, [0, 1, 2, 6, 3, 4, 5]].copy()

    def frame_position(self, path, poses):
        from pxr import Gf, UsdGeom
        if path not in self.frame_offsets:
            prim = self.stage.GetPrimAtPath(path)
            ancestor = prim
            while ancestor and str(ancestor.GetPath()) not in self.indices:
                ancestor = ancestor.GetParent()
            if not ancestor:
                raise RuntimeError('Frame has no physical body: ' + path)
            relative, _ = UsdGeom.XformCache().ComputeRelativeTransform(prim, ancestor)
            self.frame_offsets[path] = (self.indices[str(ancestor.GetPath())], relative.ExtractTranslation())
        index, offset = self.frame_offsets[path]
        p = poses[index].tolist()
        matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(p[3], Gf.Vec3d(*p[4:])))
        matrix.SetTranslateOnly(Gf.Vec3d(*p[:3]))
        return list(matrix.Transform(offset))


class PhysicsPublisher:
    def __init__(self,stage,robots,session,path=DEFAULT):
        self.stage=stage
        self.robots=robots
        self.session=session
        self.channel=StateChannel(path,writer=True)
        self.physics_poses=PhysicsPoses(stage)
        self.last=0.
        self.frames=0
        self.seconds=0.

    def publish(self,simulation_seconds,operator):
        now=time.monotonic()
        if now-self.last<.05:return
        self.last=now
        started=time.perf_counter()
        poses=list(zip(self.physics_poses.body_paths,self.physics_poses.read().tolist()))
        snapshot=dict(session=self.session,monotonic=now,updated_at=time.time(),simulation_seconds=simulation_seconds,
            poses=poses,joints={name:robot.get_joint_positions().tolist() for name,robot in self.robots.items()},operator=operator)
        if self.channel.publish(snapshot):self.frames+=1
        self.seconds+=time.perf_counter()-started
