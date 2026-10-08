"""Keep original fixed OpenArm body; add supplied cameras in original EE frames."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / 'assets/robots'


def build():
    robot = ET.parse(ROOT / 'r680_openarm/r680_openarm.urdf').getroot()
    cameras = ET.parse(ROOT / 'r680_lift_openarm/r680_lift_openarm_sim.urdf').getroot()
    robot.set('name', 'r680_fixed_openarm_cameras')
    for source in cameras:
        if not source.get('name', '').startswith(('head_d455', 'left_wrist_eb435i', 'right_wrist_eb435i')):
            continue
        item = deepcopy(source)
        for mesh in item.findall('.//mesh'):
            mesh.set('filename', '../r680_lift_openarm/' + mesh.get('filename'))
        if item.tag == 'joint' and item.get('name') in ('left_wrist_eb435i_mount_joint', 'right_wrist_eb435i_mount_joint'):
            origin = item.find('origin')
            xyz = list(map(float, origin.get('xyz').split()))
            xyz[0] += .0205  # Normalized lift-model EE frame -> original EE frame.
            origin.set('xyz', ' '.join(map(str, xyz)))
        robot.append(item)
    ET.indent(robot)
    ET.ElementTree(robot).write(ROOT / 'r680_openarm/r680_fixed_openarm_sim.urdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    build()
