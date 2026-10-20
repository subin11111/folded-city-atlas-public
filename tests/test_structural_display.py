import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from rebuild_structural_display import bounded_mask,world_grid,free_space_votes
from clean_structural_fragments import filter_glb
from surface_model_utils import write_glb
from refine_structural_heightfields import smooth_height,height_bands
from structural_prior_profiles import strong_wall_mask,regularize_height_planes


def test_small_hole_filled_large_door_preserved():
    mask=np.ones((60,60),bool);mask[10:12,10:12]=False;mask[25:45,25:45]=False
    fixed=bounded_mask(mask)
    assert fixed[10:12,10:12].all()
    assert not fixed[30:40,30:40].any()
    assert np.all(fixed[mask])


def test_open_doorway_is_not_closed_by_local_morphology():
    mask=np.ones((60,60),bool);mask[30:,20:40]=False
    fixed=bounded_mask(mask)
    assert not fixed[40:,25:35].any()


def test_world_grid_stays_on_local_plane_and_keeps_height():
    centre=np.array([1.,2.,5.]);basis=np.eye(3)
    p=world_grid(centre,basis,np.array([0.,0.]),(3,4))
    np.testing.assert_allclose(p[:,2],5.)
    np.testing.assert_allclose(p[-1],[1.18,2.12,5.])


def test_depth_observation_distinguishes_free_space_and_support():
    def camera(depth):
        return {'k_depth':np.eye(3),'world_optical':np.eye(4),'depth':np.full((3,3),depth)}
    points=np.array([[0.,0.,1.]])
    support,free=free_space_votes(points,{1:camera(1.02),2:camera(2.),3:camera(2.)})
    assert support[0]==1 and free[0]==2


def test_glb_cleanup_preserves_positions_and_only_changes_triangle_indices(tmp_path):
    import json,struct
    v=np.array([[0,0,0],[1,0,0],[0,1,0],[2,0,0],[3,0,0],[2,1,0]],float)
    t=np.array([[0,1,2],[3,4,5]]);source=tmp_path/'source.glb';target=tmp_path/'filtered.glb'
    write_glb(source,v,t,np.ones_like(v),np.ones_like(v)*.5)
    assert filter_glb(source,target,np.array([False,True]))==1
    raw=target.read_bytes();size=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+size]);payload=raw[28+size:]
    prim=doc['meshes'][0]['primitives'][0];acc=doc['accessors'][prim['indices']];view=doc['bufferViews'][acc['bufferView']]
    indices=np.frombuffer(payload,dtype='<u4',count=acc['count'],offset=view['byteOffset'])
    np.testing.assert_array_equal(indices,[3,4,5])
    attr=doc['accessors'][prim['attributes']['POSITION']];view=doc['bufferViews'][attr['bufferView']]
    xyz=np.frombuffer(payload,dtype='<f4',count=attr['count']*3,offset=view['byteOffset']).reshape(-1,3)
    np.testing.assert_array_equal(xyz[indices],v[t[1]])


def test_heightfield_does_not_flatten_or_connect_stair_step():
    height=np.zeros((30,30));height[:,15:]=.17;valid=np.ones_like(height,bool)
    result,guard=smooth_height(height,valid)
    assert guard[:,13:17].all()
    np.testing.assert_allclose(result[:,:8],0.,atol=.001)
    np.testing.assert_allclose(result[:,22:],.17,atol=.001)
    assert np.max(abs(result-height))<=.06+1e-12


def test_heightfield_preserves_slope_and_absolute_height():
    y,x=np.indices((40,40));height=2.55+x*.003+y*.002
    result,guard=smooth_height(height,np.ones_like(height,bool))
    np.testing.assert_allclose(result[10:-10,10:-10],height[10:-10,10:-10],atol=1e-5)
    assert not guard.any()


def test_adjacent_height_modes_are_merged_without_duplicate_floor_layers():
    bands=height_bands([-1.65,-1.17,2.55,5.43])
    assert len(bands)==3
    np.testing.assert_allclose(bands[0][1:],[-2.15,-.67])
    assert all(a[2]<b[1] for a,b in zip(bands,bands[1:]))


def test_isolated_height_noise_is_not_mistaken_for_a_stair_edge():
    heights=np.ones((30,30))*2.5;heights[15,15]+=.2
    result,guard=smooth_height(heights,np.ones_like(heights,bool))
    assert not guard[12:19,12:19].any()
    assert result[15,15]<heights[15,15]
    assert np.max(abs(result-heights))<=.06+1e-12


def test_second_cleanup_respects_existing_glb_indices(tmp_path):
    v=np.arange(27).reshape(9,3).astype(float);t=np.arange(9).reshape(3,3)
    a=tmp_path/'a.glb';b=tmp_path/'b.glb';c=tmp_path/'c.glb'
    write_glb(a,v,t,np.ones_like(v),np.ones_like(v)*.5)
    filter_glb(a,b,np.array([False,True,True]))
    assert filter_glb(b,c,np.array([True,False]))==1


def test_stronger_mask_fills_slit_but_protects_window_and_large_opening():
    mask=np.ones((100,100),bool);mask[10:40,10:16]=False
    mask[50:60,50:60]=False;mask[65:95,70:90]=False
    strong,protected=strong_wall_mask(mask)
    assert strong[15:35,11:15].all()
    assert protected[50:60,50:60].all() and not strong[50:60,50:60].any()
    assert not strong[70:90,75:85].any()
    assert not bounded_mask(mask)[20,13]


def test_strong_prior_flattens_noise_preserves_slope_and_boundary():
    rng=np.random.default_rng(12);y,x=np.indices((80,80));true=2.5+x*.002+y*.001
    height=true+rng.normal(0,.025,true.shape);mask=np.ones_like(height,bool)
    out,report=regularize_height_planes(height,mask)
    assert report[0]['accepted'] and report[0]['slope_deg']>0
    assert np.std(out[10:-10,10:-10]-true[10:-10,10:-10])<np.std(height[10:-10,10:-10]-true[10:-10,10:-10])*.2
    assert abs(out-height).max()<=.18+1e-12


def test_strong_prior_does_not_merge_disconnected_floor_levels():
    height=np.zeros((40,80));height[:,41:]=.18;mask=np.ones_like(height,bool);mask[:,39:42]=False
    out,report=regularize_height_planes(height,mask)
    np.testing.assert_allclose(out[:,10:30],0,atol=1e-9)
    np.testing.assert_allclose(out[:,50:70],.18,atol=1e-9)
    assert len(report)==2


def test_strong_prior_rejects_steep_ramp():
    y,x=np.indices((60,60));height=x*.04
    out,report=regularize_height_planes(height,np.ones_like(height,bool))
    assert not report[0]['accepted']
    np.testing.assert_array_equal(out,height)
