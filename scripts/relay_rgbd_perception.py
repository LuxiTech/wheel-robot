"""URDF-mounted frozen RGB-D snapshots -> isolated model worker -> world box pose."""
import json,os,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from common import ROOT
from relay_task import RelayFault


class ModelRGBDPerception:
    def __init__(self,runtime):
        from pxr import Gf,UsdGeom
        import omni.replicator.core as rep
        from wheel_grasp.kinematics import RobotModel
        self.rt=runtime;self.root=Path(os.environ['RELAY_MODEL_RUN']);self.rep=rep
        self.cfg=json.loads((self.root/'runtime.json').read_text());self.c=self.cfg['simulation']['camera']
        self.model=RobotModel(ROOT/'assets/robots/relay/relay_robot.urdf')
        self.priors={};self.cache={};self.streams={};self.sequence=0
        self.last_frames={};self.target_sequence=0
        self.stage=next(iter(runtime.controls.values())).drives['lift_joint'].GetPrim().GetStage()
        for i,s in enumerate(runtime.initial):
            p=np.eye(4);p[:3,3]=s['position'];p[:3,:3]=Rotation.from_euler('z',s.get('yaw',0.)).as_matrix();self.priors[i]=p
        for who in ('A','B'):
            ctl=runtime.controls['Robot_'+who];frame='head_d455_color_optical_frame'
            path='/World/Robots/Robot_'+who+ctl.meta['frames'][frame]
            camera=UsdGeom.Camera.Define(self.stage,path+'/ModelRGBD')
            camera.AddRotateXOp().Set(180.)
            camera.CreateFocalLengthAttr(24.)
            camera.CreateHorizontalApertureAttr(24*self.c['width']/self.c['fx'])
            camera.CreateVerticalApertureAttr(24*self.c['height']/self.c['fy'])
            camera.CreateClippingRangeAttr(Gf.Vec2f(.03,10.))
            product=rep.create.render_product(str(camera.GetPath()),(self.c['width'],self.c['height']))
            rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            rgb.attach([product]);depth.attach([product])
            self.streams[who]=(path,rgb,depth,product)

    def reset(self):
        self.cache.clear()
        self.last_frames.clear()
        for i,s in enumerate(self.rt.initial):
            p=np.eye(4);p[:3,3]=s['position'];p[:3,:3]=Rotation.from_euler('z',s.get('yaw',0.)).as_matrix();self.priors[i]=p

    def set_prior(self,index,pose):
        self.priors[index]=np.asarray(pose).copy()

    def record_targets(self,who,targets,*,kind,box_pose=None,spread=None):
        """Keep every target computation and link it to its source observation.

        A motion request can be a future pose, so it is never labeled a measured
        contact point. The image remains at its original capture time.
        """
        import cv2
        from runlog import write_json
        from wheel_grasp.visualization import draw_targets
        frame=self.last_frames.get(who)
        if frame is None:return
        folder,camera,k,index,pair,captured=frame
        self.target_sequence+=1
        record=dict(sequence=self.target_sequence,kind=kind,robot=who,target_id=str(index),
            pair_id=pair,frame=folder.name,simulation_time=self.rt.time,capture_time=captured,
            observation_age_sim_s=self.rt.time-captured,stage=self.rt.states[who],
            coordinate_frame='world',units='metre',targets={s:np.asarray(t).tolist() for s,t in targets.items()},
            box_pose_used=None if box_pose is None else np.asarray(box_pose).tolist(),spread_m=spread,
            camera_world=camera.tolist(),K=k,
            semantics='desired TCP targets; may be future or candidate goals, not measured contacts')
        source=folder/'pose_overlay.png'
        if not source.exists():source=folder/'rgb.png'
        image=cv2.imread(str(source))
        if image is None:raise RelayFault('Missing source image for target trace: '+str(source))
        overlay,projections=draw_targets(image,targets,camera,k)
        record['projections']=projections
        # Numerical history is complete; previews show the latest record per kind.
        with (folder/'grasp_targets.jsonl').open('a') as out:
            out.write(json.dumps(record,allow_nan=False)+'\n')
        with (self.root/'grasp_targets.jsonl').open('a') as out:
            out.write(json.dumps(record,allow_nan=False)+'\n')
        write_json(folder/(kind+'_latest.json'),record)
        if not cv2.imwrite(str(folder/(kind+'_overlay.png')),overlay):
            raise RelayFault('Failed to save target overlay: '+str(folder))

    def observe(self,index):
        from pxr import UsdGeom
        from isaacsim.core.api import World
        from runlog import write_json
        who='B' if self.rt.inventory.owners.get(index) in ('R','B','TARGET') else 'A'
        # Reuse only a recent actual measurement, never an initial layout pose.
        cached=self.cache.get((who,index))
        if cached and self.rt.time-cached[0]<.2:
            self.last_frames[who]=cached[2]
            return cached[1].copy()
        world=World.instance();world.pause()
        try:
            self.rep.orchestrator.step(rt_subframes=4,delta_time=0.,pause_timeline=True,wait_for_render=True)
            path,rgb_reader,depth_reader,_=self.streams[who]
            rgb=np.asarray(rgb_reader.get_data())[:,:,:3].copy()
            depth=np.asarray(depth_reader.get_data(),dtype='<f4').reshape(self.c['height'],self.c['width']).copy()
            depth[(depth<=.03)|(depth>=10)|~np.isfinite(depth)]=np.nan
            ctl=self.rt.controls['Robot_'+who]
            basepath='/World/Robots/Robot_'+who+ctl.meta['root_path']
            cache=UsdGeom.XformCache();base=np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(basepath))).T
            usd=np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))).T
            camera=base@self.model.transform('head_d455_color_optical_frame',self.rt.joints(who))
            if not np.allclose(camera,usd,atol=1e-4):raise RelayFault('Relay camera USD/URDF mismatch')
            self.sequence+=1;folder=self.root/'frames'/('%06d'%self.sequence);folder.mkdir(parents=True)
            pair=self.root.name+':'+str(self.sequence)
            k=[self.c['fx'],0,self.c['cx'],0,self.c['fy'],self.c['cy'],0,0,1]
            metadata=dict(schema='luxi_rgbd_v1',source='isaac_sim',mock=True,camera_id='head_rgb',pair_id=pair,
                frame_id='head_rgb_color_optical',width=self.c['width'],height=self.c['height'],age_ms=0.,
                aligned_to_color=True,calibration_available=True,
                intrinsics=dict(K=k,D=[0]*5,distortion_model='plumb_bob'),
                color=dict(encoding='jpeg',bytes=1),depth=dict(encoding='32FC1',unit='metre',byte_order='little-endian',
                step=self.c['width']*4,bytes=depth.nbytes,value_semantics='color_optical_z'))
            with (folder/'observation.npz').open('wb') as out:np.savez_compressed(out,metadata=json.dumps(metadata),rgb=rgb,depth=depth)
            write_json(folder/'request.json',dict(robot=who,target_id=str(index),pair_id=pair,
                camera_world=camera.tolist(),prior_world=self.priors[index].tolist(),size=self.rt.c['crate']['size'],
                simulation_time=self.rt.time,prior_source='task layout or previous observation/encoder propagation'))
            print('RELAY_MODEL_REQUEST',folder,flush=True)
            deadline=time.monotonic()+90
            import omni.kit.app
            while not (folder/'result.json').exists():
                if time.monotonic()>deadline:raise RelayFault('Model timeout; inspect '+str(folder))
                omni.kit.app.get_app().update();time.sleep(.01)
            result=json.loads((folder/'result.json').read_text())
            if result['status']!='completed':raise RelayFault('Model perception failed: '+result['error']+'; '+str(folder))
            if result['pair_id']!=pair or result['target_id']!=str(index):raise RelayFault('Model frame/target mismatch')
            pose=np.asarray(result['pose']);self.priors[index]=pose.copy()
            self.last_frames[who]=(folder,camera.copy(),k,index,pair,self.rt.time)
            self.cache[(who,index)]=(self.rt.time,pose.copy(),self.last_frames[who])
            self.rt.events.append(dict(t=self.rt.time,robot=who,state='MODEL_OBSERVATION',box=index,
                                      pair_id=pair,quality=result['quality'],folder=str(folder)))
            return pose
        finally:
            world.play()
