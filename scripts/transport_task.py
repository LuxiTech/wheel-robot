"""One reusable SLAM/IK transport cycle, interrupted cooperatively by Reset."""
import numpy as np
import json

def execute_transport(rt, c, cfg, source, target, initial, args, mode):
    def dock(table, clearance):
        return np.array([table["xy"][0], table["xy"][1]+table["size"][1]/2+clearance])
    def range_dock():
        # Final docking uses measured table range, independent of map drift.
        for _ in range(150):
            forward = np.abs(rt.slam.angles) < .10
            distance = float(np.median(rt.ranges[forward]*np.cos(rt.slam.angles[forward])))
            error = distance-cfg['manipulation'].get('place_dock_distance',.245)
            if _ % 20 == 0:
                print('RANGE_DOCK',json.dumps(dict(time=rt.time,distance=distance,error=error)),flush=True)
            if abs(error) < .012: break
            if not cfg['manipulation'].get('place_dock_distance',.245)-.04 < distance < 1.0:
                raise RuntimeError('Target docking range inconsistent')
            # A minimum crawl speed overcomes wheel/roller stiction; the
            # range tolerance above still terminates the final approach.
            speed = np.sign(error)*max(.03,min(.05,.8*abs(error)))
            rt.ctl.set_twist(float(speed),0.,0.)
            rt.step(.2)
        else:
            raise RuntimeError('Range-based target docking timed out')
        rt.ctl.stop()
        rt.step(.4)
    source_dock = dock(source,cfg["manipulation"]["dock_clearance"])
    aisle_y = -(c["room"]["aisle_width"]/2+.3)
    rt.event('BUILD_MAP')
    rt.step(1.)
    rt.verify_initial_state(initial)
    if not args.manipulation_test:
        rt.event('NAVIGATE_SOURCE')
        rt.navigate(dock(source,.65))
    if args.navigation_test:
        rt.event('NAVIGATE_TARGET')
        rt.navigate([source['xy'][0],aisle_y])
        rt.navigate([target['xy'][0],aisle_y])
        rt.navigate(dock(target,.65))
        rt.event('NAVIGATION_TEST_COMPLETE')
        rt.report(mode=mode)
    else:
        if 'lift_joint' in rt.ctl.specs:
            rt.event('RAISE_LIFT')
            rt.move_joints({'lift_joint':cfg['manipulation']['lift']},timeout=12,tolerance=.005)
        angle = cfg['manipulation'].get('grasp_finger_angle',0.0)
        rt.move_joints({'openarm_left_finger_joint1':angle,'openarm_right_finger_joint1':-angle})
        rt.event('UNFOLD_OUTSIDE_TABLE')
        rt.move_joints(rt.solve_pair(cfg['manipulation'].get('unfold_center',[.32,0,1.0]),cfg['manipulation'].get('unfold_half_width',.23),seed=cfg['manipulation'].get('unfold_seed')),timeout=16)
        rt.event('DOCK_SOURCE')
        rt.navigate(source_dock,docking=True)
        rt.event('OBSERVE_BOX')
        observation = rt.sensors.box_depth(c['box']['size'])
        if observation is None:
            raise RuntimeError('Box not visible in head depth camera')
        center,points = observation
        rt.event('VISUAL_ALIGN')
        for _ in range(30):
            error = center[:2]-np.array([.42,0.])
            if np.linalg.norm(error) < .010 and abs(rt.sensors.last_box_yaw)<.025:
                break
            command = np.clip(1.2*error,-.045,.045)
            rt.ctl.set_twist(float(command[0]),float(command[1]),float(np.clip(rt.sensors.last_box_yaw,-.15,.15)))
            rt.step(.3)
            observation = rt.sensors.box_depth(c['box']['size'])
            if observation is None:
                raise RuntimeError('Lost depth observation during visual docking')
            center,points = observation
        rt.ctl.stop()
        rt.step(.4)
        rt.event('VISUAL_ALIGNED',center_base=center.tolist(),yaw_error=rt.sensors.last_box_yaw)
        if np.linalg.norm(center[:2]-np.array([.42,0.])) > .018:
            raise RuntimeError('Visual docking did not converge')
        rt.event('PREGRASP',center_base=center.tolist(),depth_points=points)
        half = c['box']['size'][0]/2
        # Solve raised/outboard pregrasp first, then descend outside the side faces.
        rt.pair_move(center,half+cfg['manipulation'].get('pregrasp_clearance',.055),seconds=3.)
        rt.snapshot('pregrasp')
        rt.event('SQUEEZE')
        rt.pair_move(center,half-cfg['manipulation']['squeeze_depth'],seconds=3.)
        rt.step(.5)
        rt.snapshot('squeeze')
        if not (rt.contacting('openarm_left') and rt.contacting('openarm_right')):
            raise RuntimeError('Bilateral grasp contact not established')
        rt.event('LIFT_BOX')
        carry_center = center+[-.02,0,cfg['manipulation']['lift_height']]
        rt.pair_move(carry_center,half-cfg['manipulation']['squeeze_depth'],seconds=4.)
        if rt.box.get_world_pose()[0][2] < source['size'][2]+c['box']['size'][2]/2+.045:
            raise RuntimeError('Grasp verification failed: box did not clear source table')
        rt.snapshot('lifted')
        if args.manipulation_test:
            rt.event('PLACE_TEST_APPROACH')
            range_dock()
            rt.event('LOWER_TEST_BOX')
            rt.pair_move(center,half-cfg['manipulation']['squeeze_depth'],seconds=4.)
            rt.pair_move(center,half+cfg['manipulation'].get('pregrasp_clearance',.055),seconds=3.)
            rt.step(1.)
            if not rt.source_placement_success() or rt.contacting('/World/Robot'):
                raise RuntimeError('Manipulation test box not released onto source support')
            rt.event('MANIPULATION_TEST_COMPLETE')
            rt.report(mode=mode)
        else:
            rt.event('UNDOCK')
            rt.navigate(dock(source,.75),carry=True,docking=True)
            rt.event('CARRY')
            rt.navigate([source['xy'][0],aisle_y],carry=True)
            rt.navigate([target['xy'][0],aisle_y],carry=True)
            rt.navigate(dock(target,.75),carry=True)
            rt.event('PLACE_APPROACH')
            range_dock()
            rt.snapshot('target')
            rt.event('LOWER_BOX')
            rt.pair_move(center,half-cfg['manipulation']['squeeze_depth'],seconds=4.)
            rt.step(.5)
            if not rt.contacting('/'+c['task']['target']+'_Top'):
                raise RuntimeError('No target table support: refuse release')
            rt.event('RELEASE')
            rt.pair_move(center,half+cfg['manipulation'].get('pregrasp_clearance',.055),seconds=3.)
            if rt.contacting('openarm_left') or rt.contacting('openarm_right'):
                raise RuntimeError('Gripper contact remains after release command')
            rt.released = True
            rt.event('VERIFY_PLACEMENT')
            rt.step(2.)
            if not rt.placement.success:
                raise RuntimeError('Released box did not settle inside target bounds')
            rt.retreat()
            if rt.contacting('/World/Robot') or not rt.placement.success:
                raise RuntimeError('Placement verification failed after retreat')
            rt.event('COMPLETE')
            rt.snapshot('complete')
            rt.report(mode=mode)
