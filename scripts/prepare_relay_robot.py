"""Generate a separate AG95-equipped OpenArm URDF; keep original robot intact.

Requires xacro only when regenerating the vendored standalone gripper URDF.
"""
import copy
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ROOT / 'assets/robots'
VENDOR = ROBOTS / 'vendor/dh_ag95'
OUT = ROBOTS / 'relay'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    standalone = VENDOR / 'ag95.urdf'
    if not standalone.exists():
        import xacro
        wrapper = VENDOR / 'standalone.xacro'
        wrapper.write_text('''<robot name="ag95" xmlns:xacro="http://ros.org/wiki/xacro">
<xacro:include filename="urdf/dh_ag95_macro.xacro"/>
<link name="mount"/>
<xacro:dh_ag95_gripper prefix="" parent="mount"><origin xyz="0 0 0" rpy="0 0 0"/></xacro:dh_ag95_gripper>
</robot>''')
        expanded = ET.fromstring(xacro.process_file(str(wrapper)).toxml())
        for el in list(expanded):
            if el.tag not in ('link', 'joint', 'material'):
                expanded.remove(el)
        for mesh in expanded.iter('mesh'):
            mesh.set('filename', mesh.get('filename').replace('package://dh_ag95_description/', ''))
        for i, material in enumerate(expanded.iter('material')):
            if not material.get('name'):
                material.set('name', f'ag95_material_{i}')
        ET.indent(expanded)
        ET.ElementTree(expanded).write(standalone, encoding='utf-8', xml_declaration=True)
    cfg = json.loads((ROOT / 'config/robot_config.json').read_text())
    source = ROBOTS / cfg['source_urdf']
    robot = ET.parse(source).getroot()
    robot.set('name', 'r680_lift_openarm_ag95_relay')
    for mesh in robot.iter('mesh'):
        mesh.set('filename', os.path.relpath(source.parent / mesh.get('filename'), OUT))
    for side in ('left', 'right'):
        for name in (f'openarm_{side}_ee_link1', f'openarm_{side}_ee_link2'):
            robot.remove(robot.find(f"link[@name='{name}']"))
        for name in (f'openarm_{side}_finger_joint1', f'openarm_{side}_finger_joint2'):
            robot.remove(robot.find(f"joint[@name='{name}']"))
        base = robot.find(f"link[@name='openarm_{side}_ee_base_link']")
        base.clear()
        base.set('name', f'openarm_{side}_ee_base_link')
        # An explicitly approximate adapter flange, not a hardware drilling drawing.
        inertial = ET.SubElement(base, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='0 0 -0.006', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='0.08')
        ET.SubElement(inertial, 'inertia', ixx='0.000018', iyy='0.000018', izz='0.000034', ixy='0', ixz='0', iyz='0')
        for tag in ('visual', 'collision'):
            el = ET.SubElement(base, tag)
            ET.SubElement(el, 'origin', xyz='0 0 -0.006', rpy='0 0 0')
            ET.SubElement(ET.SubElement(el, 'geometry'), 'cylinder', radius='0.029', length='0.012')
        gripper = copy.deepcopy(ET.parse(standalone).getroot())
        prefix = f'openarm_{side}_gripper_'
        for el in gripper:
            if el.tag == 'link' and el.get('name') == 'mount':
                continue
            el.set('name', prefix + el.get('name'))
            if el.tag == 'joint':
                for tag in ('parent', 'child'):
                    ref = el.find(tag)
                    ref.set('link', prefix + ref.get('link'))
                if el.find('mimic') is not None:
                    el.find('mimic').set('joint', prefix + el.find('mimic').get('joint'))
                if el.get('name') == prefix + 'gripper_base_joint':
                    el.find('parent').set('link', base.get('name'))
                    el.find('origin').set('xyz', '0 0 -0.012')
                    el.find('origin').set('rpy', f'{math.pi} 0 0')
            for mesh in el.iter('mesh'):
                mesh.set('filename', os.path.relpath(VENDOR / mesh.get('filename'), OUT))
            for mat in el.iter('material'):
                mat.set('name', prefix + mat.get('name'))
            robot.append(el)
    ET.indent(robot)
    ET.ElementTree(robot).write(OUT / 'relay_robot.urdf', encoding='utf-8', xml_declaration=True)
    cfg.update(source_urdf='relay/relay_robot.urdf', model='R680 + lift + OpenArm / AG95 substitute')
    # Startup/reset is a relaxed hanging pose at the top of the lift travel.
    # Keep the folded working posture separate for automatic transport.
    cfg['stow'] = dict(cfg['home'], lift_joint=0.25)
    cfg['home'] = {f'openarm_{side}_joint{i}': 0.0
                   for side in ('left', 'right') for i in range(1, 8)}
    cfg['home'].update(lift_joint=0.4, openarm_left_joint2=-0.15,
                       openarm_right_joint2=0.15,
                       openarm_left_joint4=0.08, openarm_right_joint4=0.08)
    cfg['gripper'] = {'stiffness': 20.0, 'damping': 1.5, 'command_velocity_limit': 0.5}
    cfg['provenance']['relay_gripper'] = 'AG95 public substitute; not identified as the video gripper. Adapter approximate. Mimic tree replaces Gazebo closed loops.'
    (ROOT / 'config/relay_robot_config.json').write_text(json.dumps(cfg, indent=2))
    print('RELAY_URDF_READY', OUT / 'relay_robot.urdf')


if __name__ == '__main__':
    main()
