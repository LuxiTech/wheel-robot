"""Three physical floor pickups with independent placement evaluation."""
import json
import numpy as np
from common import ROOT


def load_demo():
    demo = json.loads((ROOT / 'config/demo_config.json').read_text())
    if len(demo['floor_positions']) != 3 or len(demo['slot_offsets']) != 3:
        raise ValueError('Demo requires three floor boxes and three target slots')
    if not np.isfinite(np.asarray(demo['floor_positions'], dtype=float)).all():
        raise ValueError('Floor coordinates must be finite')
    if not 0 < demo['lift_travel'] <= .4:
        raise ValueError('Lift travel must be within the supplied 0.4 m limit')
    return demo


def execute_floor_demo(rt, c, cfg, source, target, initial, args, mode):
    demo = cfg['demo']
    if 'lift_joint' not in rt.ctl.specs:
        raise RuntimeError('Floor demo requires the lift robot; rebuild the robot asset')
    rt.event('BUILD_MAP')
    rt.step(1.)
    rt.verify_initial_state(initial)
    half = c['box']['size'][0] / 2
    squeeze = half - demo['squeeze_depth']
    opened = half + .035
    top = target['size'][2]
    edge_y = target['xy'][1] + target['size'][1] / 2
    for index, position in enumerate(demo['floor_positions']):
        rt.select_box(index)
        rt.event('NAVIGATE_FLOOR_BOX')
        # Return via the north side of the floor row; cross it only in the
        # current pickup lane. The uncollected boxes are 0.95 m apart.
        if index:
            # The released arms remain above the floor boxes even with the
            # column down. Lower during the empty return, then verify arrival.
            rt.ctl.set_joint('lift_joint', 0.0)
            rt.navigate([demo['floor_positions'][index-1][0], -2.05], carry=False)
            rt.navigate([demo['floor_positions'][index-1][0], -.35])
        rt.navigate([position[0], -.35])
        rt.move_joints({'lift_joint': 0.0}, timeout=12, tolerance=.005)
        rt.event('OBSERVE_FLOOR_BOX')
        rt.navigate([position[0], position[1]+demo['observe_forward']], docking=True)
        observation = rt.sensors.box_depth(c['box']['size'], floor=True)
        if observation is None:
            raise RuntimeError('Floor box not visible in head depth camera')
        center, points = observation
        rt.event('FLOOR_OBSERVED', center_base=center.tolist(), depth_points=points)
        # The head camera loses the box close to the base. Preserve the
        # measured world position through encoder/laser localization.
        from lidar_slam import rotation2
        measured_world = rt.slam.pose[:2] + rotation2(rt.slam.pose[2]) @ center[:2]
        dock = measured_world + [0, demo['grasp_forward']]
        rt.navigate(dock, docking=True)
        center[:2] = rotation2(-rt.slam.pose[2]) @ (measured_world-rt.slam.pose[:2])
        center[2] = demo['grasp_height']
        rt.event('PREGRASP', center_base=center.tolist())
        angle = demo['finger_angle']
        rt.move_joints({'openarm_left_finger_joint1': angle, 'openarm_right_finger_joint1': -angle})
        # Unfold directly above the floor pickup, then descend outside it.
        seed = demo.get('ground_seed')
        rt.move_joints(rt.solve_pair([center[0], center[1], .30], opened, seed=seed))
        rt.pair_move(center, opened, seconds=1.8)
        rt.snapshot('floor_pregrasp')
        rt.event('SQUEEZE')
        rt.pair_move(center, squeeze, seconds=1.2)
        rt.step(.3)
        if not (rt.contacting('openarm_left') and rt.contacting('openarm_right')):
            raise RuntimeError('Bilateral floor grasp contact not established')
        rt.event('LIFT_BOX')
        # Raise the arms first, then the lift column; the load is held by
        # friction throughout, never attached or repositioned in flight.
        rt.pair_move([center[0], center[1], .46], squeeze, seconds=3.5)
        rt.move_joints({'lift_joint': demo['lift_travel']}, timeout=12, tolerance=.005)
        carry_center = [demo['carry_center'][0], center[1], demo['carry_center'][2]]
        rt.pair_move(carry_center, squeeze, seconds=2.5)
        if rt.box.get_world_pose()[0][2] < top+c['box']['size'][2]/2+.04:
            raise RuntimeError('Box did not reach safe workbench clearance')
        rt.snapshot('floor_lifted')
        rt.event('CARRY')
        rt.navigate([position[0], -2.05], carry=True)
        slot_x = target['xy'][0] + demo['slot_offsets'][index]
        rt.navigate([slot_x, -2.05], carry=True)
        rt.navigate([slot_x, edge_y+.55], carry=True, docking=True)
        rt.event('PLACE_APPROACH')
        for _ in range(150):
            mask = np.abs(rt.slam.angles) < .10
            distance = float(np.median(rt.ranges[mask]*np.cos(rt.slam.angles[mask])))
            error = distance - demo['place_dock_distance']
            if abs(error) < .012:
                break
            if not .17 < distance < 1.0:
                raise RuntimeError('Workbench docking range inconsistent')
            rt.ctl.set_twist(float(np.sign(error)*max(.03, min(.07, .8*abs(error)))), 0., 0.)
            rt.step(.2)
        else:
            raise RuntimeError('Workbench docking timed out')
        rt.ctl.stop()
        rt.step(.3)
        rt.event('LOWER_BOX')
        # Grasp height is offset from the box center at the floor.
        place_center = [demo['place_forward'], 0, top+demo['grasp_height']-.005]
        rt.pair_move(place_center, squeeze, seconds=2.)
        rt.step(.4)
        if not rt.contacting('/'+c['task']['target']+'_Top'):
            raise RuntimeError('No workbench support; refuse release')
        rt.event('RELEASE')
        release_center = [.48, 0, place_center[2]]
        rt.pair_move(release_center, opened, seconds=1.2)
        rt.released = True
        rt.box_released[index] = True
        rt.event('VERIFY_PLACEMENT')
        rt.step(1.3)
        if not rt.placement.success or rt.contacting('/World/Robot'):
            raise RuntimeError('Box not stable and released in its assigned slot')
        rt.event('CLEAR_WORKBENCH')
        rt.pair_move([.42, 0, .94], .22, seconds=1.8)
        rt.retreat()
        if not rt.placement.success or rt.contacting('/World/Robot'):
            raise RuntimeError('Placement invalid after retreat')
        rt.completed_boxes.append(index)
        rt.event('BOX_COMPLETE')
        rt.snapshot('box_complete')
    rt.step(1.2)
    robot_contact = any('/World/Robot' in path for contacts in rt.all_contacts.values() for path in contacts)
    if not all(p.success for p in rt.placements) or robot_contact:
        raise RuntimeError('A previously placed box is no longer stable')
    rt.event('COMPLETE')
    rt.snapshot('complete')
    rt.report(mode=mode)
