"""Prepare the supplied relay URDF with its original OpenArm pinch grippers."""
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ROOT / 'assets/robots'
OUT = ROBOTS / 'relay'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((ROOT / 'config/profiles/robot_relay.json').read_text())
    source = ROBOTS / cfg['source_urdf']
    cfg['provenance']['relay_source_urdf'] = cfg['source_urdf']
    robot = ET.parse(source).getroot()
    robot.set('name', 'r680_lift_openarm_relay')
    for mesh in robot.iter('mesh'):
        mesh.set('filename', os.path.relpath(source.parent / mesh.get('filename'), OUT))
    ET.indent(robot)
    ET.ElementTree(robot).write(OUT / 'relay_robot.urdf', encoding='utf-8', xml_declaration=True)
    cfg.update(source_urdf='relay/relay_robot.urdf', model=cfg['model'] + ' / original pinch grippers')
    # Startup/reset is a relaxed hanging pose at the top of the lift travel.
    # Keep the folded working posture separate for automatic transport.
    cfg['stow'] = dict(cfg['home'], lift_joint=0.25)
    cfg['home'] = {f'openarm_{side}_joint{i}': 0.0
                   for side in ('left', 'right') for i in range(1, 8)}
    cfg['home'].update(lift_joint=0.4, openarm_left_joint2=-0.15,
                       openarm_right_joint2=0.15,
                       openarm_left_joint4=0.08, openarm_right_joint4=0.08)
    cfg['home'].update(openarm_left_finger_joint1=0.7854, openarm_right_finger_joint1=-0.7854)
    cfg['gripper'] = {'stiffness': 20.0, 'damping': 1.5, 'command_velocity_limit': 0.5}
    cfg['provenance']['relay_gripper'] = 'Original supplied OpenArm pinch grippers; geometry, inertia, limits and mimic joints preserved.'
    (ROOT / 'config/relay_robot_config.json').write_text(json.dumps(cfg, indent=2))
    print('RELAY_URDF_READY', OUT / 'relay_robot.urdf')


if __name__ == '__main__':
    main()
