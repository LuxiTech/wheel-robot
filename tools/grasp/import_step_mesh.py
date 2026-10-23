#!/usr/bin/env python3
"""Convert a millimetre STEP to a metre OBJ, centered in XY with bottom Z=0."""
import argparse
import hashlib
from importlib.metadata import version
import json
import numpy as np
import trimesh
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source');p.add_argument('--output',required=True)
    args=p.parse_args();source=Path(args.source);out=Path(args.output)
    if out.exists() or out.with_suffix('.cad.json').exists():raise FileExistsError(out)
    reader=STEPControl_Reader()
    if reader.ReadFile(str(source))!=IFSelect_RetDone:raise ValueError('Cannot read STEP')
    # OCCT import uses millimetres; this importer explicitly fixes that system unit.
    reader.SetSystemLengthUnit(1.0)
    if not reader.TransferRoots():raise ValueError('STEP contains no transferable roots')
    shape=reader.OneShape();bounds=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,bounds,False,False)
    low,high=bounds.CornerMin(),bounds.CornerMax()
    x0,y0,z0=low.X(),low.Y(),low.Z();x1,y1,z1=high.X(),high.Y(),high.Z()
    offset=[(x0+x1)/2,(y0+y1)/2,z0]
    mesher=BRepMesh_IncrementalMesh(shape,.25,False,.2,True)
    if not mesher.IsDone():raise RuntimeError('STEP meshing failed')
    vertices=[];triangles=[];faces=0
    explorer=TopExp_Explorer(shape,TopAbs_FACE)
    while explorer.More():
        cast=getattr(TopoDS,"Face_s",None) or TopoDS.Face
        face=cast(explorer.Current());location=TopLoc_Location()
        mesh=BRep_Tool.Triangulation_s(face,location)
        if mesh is None:raise ValueError('Face has no triangulation')
        start=len(vertices);trsf=location.Transformation()
        for i in range(1,mesh.NbNodes()+1):
            point=mesh.Node(i).Transformed(trsf)
            vertices.append(tuple((v-c)*.001 for v,c in zip((point.X(),point.Y(),point.Z()),offset)))
        for i in range(1,mesh.NbTriangles()+1):
            triangle=mesh.Triangle(i)
            a,b,c=(triangle.Value(j) for j in (1,2,3))
            if face.Orientation()==TopAbs_REVERSED:b,c=c,b
            triangles.append((start+a,start+b,start+c))
        faces+=1;explorer.Next()
    if not triangles:raise ValueError('Empty tessellation')
    solids=0;explorer=TopExp_Explorer(shape,TopAbs_SOLID)
    while explorer.More():solids+=1;explorer.Next()
    out.parent.mkdir(parents=True,exist_ok=True)
    mesh=trimesh.Trimesh(vertices=np.asarray(vertices),faces=np.asarray(triangles)-1,process=True)
    before=len(mesh.faces)
    mesh.update_faces(mesh.nondegenerate_faces() & mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    if not np.isfinite(mesh.vertices).all():raise ValueError('Non-finite mesh')
    mesh.export(out)
    record=dict(source=str(source.resolve()),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        output_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),ocp_version=version('cadquery-ocp'),
        source_bounds_mm=[x0,y0,z0,x1,y1,z1],dimensions_m=[(x1-x0)*.001,(y1-y0)*.001,(z1-z0)*.001],
        source_offset_mm=offset,scale_to_metres=.001,output_origin='XY center, bottom Z=0',
        original_axis_directions_preserved=True,linear_deflection_mm=.25,angular_deflection_rad=.2,
        solids=solids,faces=faces,vertices=len(mesh.vertices),triangles=len(mesh.faces),
        removed_degenerate_or_duplicate_triangles=before-len(mesh.faces),watertight=bool(mesh.is_watertight),
        hardware_dimensions_verified=False,grasp_rules_validated=False)
    out.with_suffix('.cad.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
