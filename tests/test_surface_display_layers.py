import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from refine_surface_display import depth_sheet,srgb_to_linear,standardize_glb
from surface_model_utils import write_glb


def test_srgb_midtones_are_linearized_not_geometry():
    values=srgb_to_linear(np.array([0.,.5,1.]))
    np.testing.assert_allclose(values,[0.,.21404114048223255,1.])


def frame(depth):
    return {'depth':np.asarray(depth,float),'k_depth':np.eye(3),'k_rgb':np.eye(3),
        'world_optical':np.eye(4),'rgb':np.zeros((10,10,3),np.uint8)}


def test_depth_sheet_does_not_join_depth_discontinuity():
    _,triangles,_,_=depth_sheet(frame([[1.,1.],[2.,2.]]),stride=1)
    assert len(triangles)==0
    _,triangles,_,_=depth_sheet(frame([[1.,1.],[1.,1.]]),stride=1)
    assert len(triangles)==2


def test_depth_sheet_does_not_fill_invalid_depth():
    _,triangles,_,_=depth_sheet(frame([[1.,0.],[1.,1.]]),stride=1)
    assert len(triangles)==0


def test_glb_colour_conversion_preserves_geometry(tmp_path):
    import json,struct
    v=np.eye(3);t=np.array([[0,1,2]]);source=tmp_path/'source.glb';target=tmp_path/'linear.glb'
    write_glb(source,v,t,v,np.full((3,3),.5))
    report=standardize_glb(source,target)
    assert report['colour_accessors_converted']==1
    assert report['geometry_and_atlas_pixels_changed'] is False
    raw=target.read_bytes();size=struct.unpack('<I',raw[12:16])[0];doc=json.loads(raw[20:20+size]);binary=raw[28+size:]
    attrs=doc['meshes'][0]['primitives'][0]['attributes']
    for key,expected in [('POSITION',v),('COLOR_0',np.full((3,3),.21404114048223255))]:
        acc=doc['accessors'][attrs[key]];view=doc['bufferViews'][acc['bufferView']]
        actual=np.frombuffer(binary,dtype='<f4',count=acc['count']*3,offset=view['byteOffset']).reshape(-1,3)
        np.testing.assert_allclose(actual,expected,atol=1e-7)
