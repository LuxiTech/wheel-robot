import math
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import numpy as np
from scipy.spatial.transform import Rotation
from box_perception import fit_box


class PerceptionTests(unittest.TestCase):
    def observe(self,center,yaw):
        camera=np.array([.183562318,.047725407,.946356443])
        camrot=Rotation.from_euler('xyz',[0,.745022486,.002466441]).as_matrix()
        objrot=Rotation.from_euler('z',yaw).as_matrix()
        points,table,rows,cols=[],[],[],[]
        half=np.array([.12,.15,.11])
        for i,pitch in enumerate(np.linspace(-math.radians(29),math.radians(29),35)):
            for j,a in enumerate(np.linspace(-math.radians(43.5),math.radians(43.5),61)):
                direction=camrot @ [math.cos(pitch)*math.cos(a),math.cos(pitch)*math.sin(a),math.sin(pitch)]
                o=objrot.T @ (camera-center)
                d=objrot.T @ direction
                t1=(-half-o)/d
                t2=(half-o)/d
                near=max(np.minimum(t1,t2));far=min(np.maximum(t1,t2))
                table_dist=(.75-camera[2])/direction[2] if direction[2]<0 else np.inf
                if near>0 and far>=near and near<table_dist:
                    points.append(camera+direction*near);rows.append(i);cols.append(j)
                elif 0<table_dist<2.5:
                    table.append(camera+direction*table_dist)
        return fit_box(points,table,rows,cols,[.3,.24,.22])

    def test_near_field_partial_box(self):
        for x in (.42,.48):
            for y in (-.035,0,.035):
                for yaw in (-.06,0,.06):
                    center=np.array([x,y,.86])
                    result=self.observe(center,yaw)
                    self.assertIsNotNone(result,(center,yaw))
                    estimate,angle=result
                    self.assertLess(np.linalg.norm(estimate-center),.018,(center,yaw,estimate))
                    self.assertLess(abs(angle-yaw),.04,(angle,yaw))

    def test_missing_depth_fails_closed(self):
        self.assertIsNone(fit_box([],[],[],[],[.3,.24,.22]))
