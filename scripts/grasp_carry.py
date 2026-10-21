"""Contact-driven pinch, lift and wheel transport; no object attachment or teleport."""
import json
import math
import numpy as np
from runlog import write_json


def execute_carry(*,cfg,directory,world,stage,robot,meta,control,names,contacts,bad_contacts,
                  contact_mode,log,render_tick,status):
    from pxr import UsdGeom
    from isaacsim.core.utils.types import ArticulationAction
    from robot_control import OmniKinematics
    settings=cfg['simulation']['carry'];dt=1/120
    phase='close';tick_count=0
    wheels=[names.index(w['joint']) for w in meta['wheels']]
    omni=OmniKinematics(meta['wheels'],max_linear=.08,max_angular=.1)
    def box_pose():
        return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Target'))).T
    initial=box_pose().copy();base_initial=robot.get_world_pose()[0].copy()
    last_contact={s:-100. for s in ('left','right')}
    contact_mode[0]='pinch'
    samples=(directory/'carry_steps.jsonl').open('w',buffering=1)
    def step(twist=(0.,0.,0.),require_grip=False):
        nonlocal tick_count
        contacts.clear()
        robot.apply_action(ArticulationAction(joint_positions=control))
        robot.apply_action(ArticulationAction(joint_velocities=np.asarray(omni.inverse(twist)),joint_indices=np.asarray(wheels)))
        world.step(render=render_tick(tick_count))
        t=tick_count*dt;tick_count+=1
        for side in last_contact:
            if any('openarm_'+side+'_ee_link' in x for x in contacts):last_contact[side]=t
        pose=box_pose();base=robot.get_world_pose()[0]
        tilt=math.acos(float(np.clip(pose[2,2],-1,1)))
        sample=dict(step=tick_count,time=t,phase=phase,box_position=pose[:3,3].tolist(),
                    base_position=base.tolist(),box_rise_m=float(pose[2,3]-initial[2,3]),
                    tilt_rad=tilt,contacts=sorted(contacts),last_contact_age_s={s:t-v for s,v in last_contact.items()},
                    lift=float(robot.get_joint_positions()[names.index('lift_joint')]))
        samples.write(json.dumps(sample)+'\n')
        if bad_contacts:raise ValueError('Non-finger collision during '+phase+': '+str(bad_contacts[:2]))
        if not np.isfinite(pose).all() or tilt>.25:raise ValueError('Box unstable during '+phase)
        if require_grip and any(t-v>.4 for v in last_contact.values()):raise ValueError('Lost bilateral grip during '+phase)
        return pose,base
    try:
        with log.stage('close_grippers'):
            status('CLOSE: establishing finger contact')
            start=control.copy()
            for k in range(360):
                u=min(1.,(k+1)/240);u=u*u*(3-2*u)
                for i,n in enumerate(names):
                    if 'finger_joint' in n:control[i]=start[i]*(1-u)
                step()
            if any(tick_count*dt-v>.2 for v in last_contact.values()):
                raise ValueError('Closure failed: both hands must contact the box')
            closed=box_pose().copy()
            write_json(directory/'closure.result.json',dict(status='completed',box_position=closed[:3,3].tolist(),
                actual_fingers={n:float(robot.get_joint_positions()[i]) for i,n in enumerate(names) if 'finger_joint' in n}))
        phase='lift'
        with log.stage('lift_box'):
            status('LIFT: 10 cm trial lift')
            index=names.index('lift_joint');start=float(control[index]);height=settings['lift_m']
            if start+height>.4:raise ValueError('Insufficient lift travel for trial lift')
            duration=1.875*height/.025
            for k in range(math.ceil((duration+2)/dt)):
                u=min(1.,k*dt/duration);control[index]=start+height*(10*u**3-15*u**4+6*u**5)
                pose,base=step(require_grip=True)
                if pose[2,3]<closed[2,3]-.015:raise ValueError('Box slipped downward during lift')
            rise=float(pose[2,3]-closed[2,3])
            if rise<height-.02:raise ValueError('Box did not follow lift: rise='+str(rise))
            write_json(directory/'lift.result.json',dict(status='completed',rise_m=rise))
        phase='carry'
        with log.stage('carry_box'):
            status('CARRY: wheel-driven reverse transport')
            start_base=robot.get_world_pose()[0].copy();offset=box_pose()[:3,3]-start_base
            distance=settings['distance_m'];speed=.04;duration=distance/speed+1.
            for k in range(math.ceil((duration+2)/dt)):
                t=k*dt
                v=-speed*min(1.,t/.5,max(0.,(duration-t)/.5)) if t<duration else 0.
                pose,base=step((v,0.,0.),require_grip=True)
                if np.linalg.norm((pose[:3,3]-base)-offset)>.03:raise ValueError('Box slipped relative to robot during transport')
            moved=float(start_base[0]-base[0])
            if moved<distance*.8:raise ValueError('Wheel transport distance insufficient: '+str(moved))
            write_json(directory/'carry.result.json',dict(status='completed',distance_m=moved,
                box_rise_m=float(pose[2,3]-initial[2,3]),dynamic_box=True,attachment_used=False))
            status('DONE: physical pinch, lift and transport verified')
        return dict(grasp_and_carry_validated=True,transport_distance_m=moved,box_lift_m=rise)
    finally:
        robot.apply_action(ArticulationAction(joint_velocities=np.zeros(len(wheels)),joint_indices=np.asarray(wheels)))
        samples.close()
