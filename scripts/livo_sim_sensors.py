"""Synchronized simulation sensor boundary for the real FAST-LIVO2 process.

Only this adapter reads physics poses to generate raw measurements. Estimated
navigation pose is exclusively read from FAST-LIVO2 odometry, never from truth.
IPC uses atomically committed files so Isaac can keep network_mode=none.
"""
import json
import math
import os
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
from common import OUTPUTS
from relay_task import RelayFault


class LivoSimSensors:
    def __init__(self,runtime,stage,root):
        import omni.physx
        import omni.replicator.core as rep
        from pxr import UsdGeom,Gf
        self.rt=runtime
        self.robot=runtime.robots['Robot_A']
        self.query=omni.physx.get_physx_scene_query_interface()
        self.offset=np.array([0.,0.,1.25])
        import omni.kit.app
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.physics',True)
        from isaacsim.sensors.experimental.physics import IMU,IMUSensor
        self.imu=IMUSensor(IMU.create(root+'/NavigationIMU',translations=self.offset.reshape(1,3)))
        self.folder=OUTPUTS/'fast_livo'
        self.folder.mkdir(exist_ok=True)
        self.anchor=np.array(runtime.c['robots'][0]['position'][:2]+[runtime.c['robots'][0]['yaw']])
        self.reset_session()
        camera=UsdGeom.Camera.Define(stage,root+'/NavigationCamera')
        camera.CreateFocalLengthAttr(24.)
        camera.CreateHorizontalApertureAttr(36.)
        camera.CreateVerticalApertureAttr(27.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05,20.))
        xf=UsdGeom.Xformable(camera)
        xf.AddTranslateOp().Set(Gf.Vec3d(*self.offset))
        # USD camera looks along -Z, +Y up; robot sensor frame is FLU.
        q=Rotation.from_matrix([[0,0,-1],[-1,0,0],[0,1,0]]).as_quat()
        xf.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
        self.product=rep.create.render_product(str(camera.GetPath()),(480,360))
        self.rgb=rep.AnnotatorRegistry.get_annotator('rgb')
        self.rep=rep
        self.rgb.attach([self.product])
        self.camera_params=rep.AnnotatorRegistry.get_annotator('camera_params')
        self.camera_params.attach([self.product])
        az=np.linspace(-math.pi,math.pi,180,endpoint=False)
        el=np.radians(np.linspace(-65,25,16))
        self.directions=np.array([[math.cos(e)*math.cos(a),math.cos(e)*math.sin(a),math.sin(e)] for e in el for a in az])

    def reset_session(self):
        self.run_id=uuid.uuid4().hex
        self.stream=self.folder/self.run_id
        self.stream.mkdir()
        self.sequence=0;self.samples=[];self.prev_velocity=None
        self.rng=np.random.default_rng(47)
        self.tick_count=0;self.sensor_time=0.
        self.closest_obstacle=float('inf')
        self.atomic_json(self.folder/'session.json',dict(run_id=self.run_id,anchor=self.anchor.tolist(),sensor_height=1.25))

    @staticmethod
    def atomic_json(path,value):
        tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(value));os.replace(tmp,path)

    def tick(self,dt):
        self.sensor_time+=dt
        p,q=self.robot.get_world_pose()
        rot=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
        if not self.imu.get_sensor_reading(read_gravity=True).is_valid:
            raise RelayFault('Native IMU sensor reading is invalid')
        reading=self.imu.get_data(read_gravity=True)
        accel=np.asarray(reading['linear_acceleration'])
        gyro=np.asarray(reading['angular_velocity'])
        self.samples.append([self.sensor_time,* (accel+self.rng.normal(0,.002,3)),*(gyro+self.rng.normal(0,.0002,3))])
        self.tick_count+=1
        if self.tick_count%12:
            return
        # Renderer annotators otherwise expose the preceding rendered frame.
        # Flush at zero simulation delta so RGB and rays share a physical pose.
        self.rep.orchestrator.step(delta_time=0.,rt_subframes=1,pause_timeline=False)
        origin=np.asarray(p)+rot @ self.offset
        points=[]
        for direction in self.directions:
            closest=[12.]
            def hit(h):
                if not str(h.collision).startswith('/World/Robots/Robot_A/') and .15<h.distance<closest[0]:
                    closest[0]=float(h.distance)
                return True
            self.query.raycast_all(tuple(map(float,origin)),tuple(map(float,rot @ direction)),12.,hit)
            if closest[0]<11.99:
                points.append(direction*(closest[0]+self.rng.normal(0,.002)))
        rgb=np.asarray(self.rgb.get_data())
        if rgb.ndim!=3 or rgb.shape[0]!=360:
            raise RelayFault('FAST-LIVO2 RGB sensor has no rendered frame')
        xyz=np.asarray(points,dtype=np.float32)
        body_height=xyz[:,2]+self.offset[2]
        relevant=xyz[(body_height>.08)&(body_height<1.2)]
        self.closest_obstacle=float(np.min(np.linalg.norm(relevant[:,:2],axis=1))) if len(relevant) else float('inf')
        destination=self.stream/f'{self.sequence:08d}.npz'
        tmp=destination.with_suffix('.tmp')
        with tmp.open('wb') as f:
            np.savez(f,stamp=self.sensor_time,imu=np.array(self.samples),points=np.asarray(points,dtype=np.float32),rgb=rgb[:,:,:3])
        os.replace(tmp,destination)
        self.sequence+=1;self.samples=[]
        if self.sequence%10==0:
            from PIL import Image
            Image.fromarray(rgb[:,:,:3]).save(self.folder/'camera_latest.png')
            camera_data=self.camera_params.get_data()
            view=np.asarray(camera_data['cameraViewTransform']).reshape(4,4)
            camera_world=np.linalg.inv(view).T
            camera_rotation=rot @ np.array([[0,0,-1],[-1,0,0],[0,1,0]])
            self.atomic_json(self.folder/'sensor_status.json',dict(run_id=self.run_id,frames=self.sequence,stamp=self.sensor_time,points=len(points),
                ray_origin=origin.tolist(),render_origin=camera_world[:3,3].tolist(),
                render_position_error=float(np.linalg.norm(camera_world[:3,3]-origin)),
                render_angle_error_rad=float(Rotation.from_matrix(camera_world[:3,:3] @ camera_rotation.T).magnitude()),
                camera_projection=np.asarray(camera_data['cameraProjection']).tolist(),
                closest_obstacle_m=self.closest_obstacle,imu_source='Isaac native IMUSensor'))

    def estimated_pose(self,stamp):
        try:
            data=json.loads((self.folder/'odometry.json').read_text())
        except (OSError,ValueError) as exc:
            raise RelayFault('Waiting for FAST-LIVO2 odometry') from exc
        age=stamp-data['stamp']
        if data.get('source')!='FAST-LIVO2' or data.get('run_id')!=self.run_id or not -1e-6<=age<=.6 or data.get('samples',0)<10:
            raise RelayFault('FAST-LIVO2 odometry is stale or uninitialized')
        pos=np.asarray(data['position'])
        rot=Rotation.from_quat(data['quaternion_xyzw']).as_matrix()
        local_base=pos-rot @ self.offset+self.offset
        c,s=np.cos(self.anchor[2]),np.sin(self.anchor[2])
        xy=self.anchor[:2]+np.array([[c,-s],[s,c]]) @ local_base[:2]
        yaw=self.anchor[2]+Rotation.from_matrix(rot).as_euler('xyz')[2]
        if not np.isfinite([*xy,yaw]).all():
            raise RelayFault('FAST-LIVO2 returned non-finite pose')
        return np.r_[xy,yaw]
