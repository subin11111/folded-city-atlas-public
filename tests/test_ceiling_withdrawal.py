import json,sys,struct
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from withdraw_inferred_ceilings import retained_without_modeled_ceilings,merge_glbs
from surface_model_utils import write_glb
from audit_structural_delivery import glb_triangle_positions


def test_withdrawal_removes_only_added_ceiling_triangles():
    planes=[{'accepted':True,'kind':'floor','triangles':2},{'accepted':True,'kind':'ceiling','triangles':4},{'accepted':True,'kind':'floor','triangles':3}]
    keep=retained_without_modeled_ceilings(14,5,planes)
    assert keep.tolist()==[True]*7+[False]*4+[True]*3


def test_unknown_triangle_layout_is_not_silently_filtered():
    import pytest
    with pytest.raises(ValueError):retained_without_modeled_ceilings(14,5,[])


def test_joined_observed_geometry_keeps_exact_vertices_and_material_offsets(tmp_path):
    points=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
    for name,offset in [('first',0.),('second',3.)]:
        write_glb(tmp_path/(name+'.glb'),points+[0,0,offset],np.array([[0,1,2]]),np.tile([0.,0.,1.],(3,1)),np.full((3,3),.5))
    merge_glbs(tmp_path/'first.glb',tmp_path/'second.glb',tmp_path/'joined.glb')
    np.testing.assert_array_equal(glb_triangle_positions(tmp_path/'joined.glb'),np.array([points,points+[0,0,3.]],np.float32))
    raw=(tmp_path/'joined.glb').read_bytes();size=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+size])
    assert [v['material'] for v in doc['meshes'][0]['primitives']]==[0,1]


def test_user_ceiling_policy_disallows_invention_and_wall_height_changes():
    policy=json.loads((Path(__file__).resolve().parents[1]/'config/ceiling_completion_policy.json').read_text())
    assert not policy['allow_inferred_ceiling_completion']
    assert not policy['allow_height_from_histogram_or_floor_footprint']
    assert not policy['allow_wall_height_changes']
