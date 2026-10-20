import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from revalidate_cross_sessions import clusters_across_pose_sources,edge_records,project_node_points,parse_pair,valid_depth_feature_mask,drift_hypothesis_records


def test_projection_respects_local_transform_and_rgb_calibration_scale():
    local=np.eye(4);local[:3,:3]=Rotation.from_euler('z',np.pi/2).as_matrix();local[:3,3]=[1,2,3]
    k=np.array([[100,0,50],[0,100,40],[0,0,1.]])
    session=SimpleNamespace(depth=lambda n:(None,None,100,80,k,local),
                            image=lambda n:np.zeros((160,200,3),np.uint8))
    optical=np.array([[.2,.3,2.]])
    node=optical@local[:3,:3].T+local[:3,3]
    np.testing.assert_allclose(project_node_points(session,1,node),[[120,110]])


def test_global_drift_does_not_erase_locally_valid_edge():
    result={'accepted':True,'outside_id':1,'lobby_id':5,'lobby_node_to_outside_node':np.eye(4).tolist()}
    poses_a={'raw':{1:np.eye(4)}};poses_b={'raw':{5:np.eye(4)}}
    records=edge_records([result],[],'floor2','bridge_12_sub',poses_a,poses_b)
    assert len(records)==1 and records[0]['local_geometry_verified']
    assert not records[0]['eligible_for_joint']
    assert records[0]['node_b']==5


def test_multiple_pose_hypotheses_preserve_independent_support():
    session=SimpleNamespace(stamps={1:0.,2:10.,3:20.})
    raw={i:np.eye(4) for i in (1,2,3)}
    for i in raw:raw[i][0,3]=i
    distorted={i:raw[i].copy() for i in raw}
    for i in distorted:distorted[i][1,3]=i*.7
    a={'raw':raw,'app':raw,'optimized':distorted}
    b={'raw':raw,'app':raw,'optimized':raw}
    rows=[{'accepted':True,'outside_id':i,'lobby_id':i,'lobby_node_to_outside_node':np.eye(4).tolist()} for i in raw]
    proposals=clusters_across_pose_sources(rows,session,session,a,b)
    assert {p['pose_source'] for p in proposals}=={'raw','app'}
    assert all(p['independent_support_count']==3 for p in proposals)
    edges=edge_records(rows,proposals,'a','b',a,b)
    assert len(edges)==3 and all(e['eligible_for_joint'] for e in edges)


def test_isolated_node_cannot_be_joint_edge():
    row={'accepted':True,'outside_id':9,'lobby_id':8,'lobby_node_to_outside_node':np.eye(4).tolist()}
    proposals=[{'independent_pairs':[[9,8]],'independent_support_count':3}]
    edges=edge_records([row],proposals,'a','b',{'raw':{}},{'raw':{8:np.eye(4)}})
    assert not edges[0]['nodes_in_main_components'] and not edges[0]['eligible_for_joint']


def test_pair_parser_rejects_same_or_unknown_sessions():
    assert parse_pair('outside:bridge_out1f')==('outside','bridge_out1f')
    with pytest.raises(Exception):parse_pair('outside:outside')
    with pytest.raises(Exception):parse_pair('../x:lobby')


def test_whole_link_holdout_keeps_three_training_and_initializes_from_training():
    rows=[{'accepted':True,'outside_id':i,'lobby_id':i,'lobby_node_to_outside_node':np.eye(4).tolist()} for i in range(1,5)]
    poses={'raw':{i:np.eye(4) for i in range(1,5)}}
    details=[{'outside_id':i,'lobby_id':i,'lobby_map_to_outside_map':(np.eye(4)*i).tolist()} for i in range(1,5)]
    proposals=[{'independent_pairs':[[i,i] for i in range(1,5)],'independent_support_count':4,'support_details':details}]
    edges=edge_records(rows,proposals,'a','b',poses,poses)
    assert [e['joint_role'] for e in edges]==['train','train','train','heldout']
    assert proposals[0]['initialization_pair']==[1,1]
    assert not edges[-1]['initialization_allowed']
    assert all(e['held_out']==e['joint_holdout'] for e in edges)


def test_valid_depth_detection_support_preserves_input_arrays():
    depth=np.ones((8,8));confidence=np.full((8,8),100,np.uint8)
    depth[3,3]=np.nan;confidence[6,6]=0
    mask=valid_depth_feature_mask(depth,confidence,(16,16,3))
    assert mask.shape==(16,16) and mask.dtype==np.uint8
    assert mask[6,6]==0 and mask[12,12]==0 and mask[0,0]==255
    assert np.isnan(depth[3,3]) and confidence[6,6]==0


def test_drift_hypothesis_separates_experiment_from_verified_rigid_edges():
    poses={i:np.eye(4) for i in range(1,5)}
    for i in poses:poses[i][0,3]=i
    session=SimpleNamespace(stamps={i:float(i*10) for i in poses})
    rows=[]
    for i in poses:
        t=np.eye(4);t[1,3]=.2*i
        rows.append({'node_a':i,'node_b':i,'T_a_b':t.tolist(),'local_geometry_verified':True})
    edges,report=drift_hypothesis_records(rows,session,session,poses,poses,True)
    assert len(edges)==4 and report['eligible']
    assert sum(e['held_out'] for e in edges)==1
    assert all(e['hypothesis_eligible'] and not e['eligible_for_joint'] for e in edges)
    assert not edges[-1]['initialization_allowed']
    assert not drift_hypothesis_records(rows,session,session,poses,poses,False)[0]
    rows[-1]['T_a_b'][1][3]=4.
    assert not drift_hypothesis_records(rows,session,session,poses,poses,True)[0]


def test_planar_variant_uses_disjoint_depth_holdout_and_bidirectional_icp(tmp_path,monkeypatch):
    import cv2
    import revalidate_cross_sessions as driver
    pixels=np.array([(u,v) for u in range(100,901,100) for v in range(100,901,100)],float)
    keypoints=[cv2.KeyPoint(float(u),float(v),1) for u,v in pixels]
    matches=[cv2.DMatch(i,i,0.) for i in range(len(pixels))]
    monkeypatch.setattr(driver.checker,'mutual_ratio_matches',lambda *args:matches)
    calls=[]
    def icp(target,source,initial):
        calls.append((target,source))
        return initial,{'symmetric_overlap':1.,'symmetric_rmse_m':0.,'refinement_m':0.,'refinement_deg':0.}
    monkeypatch.setattr(driver.checker,'refine_icp',icp)
    k=np.array([[500,0,500],[0,500,500],[0,0,1.]])
    session=SimpleNamespace(feature=lambda *args:(keypoints,np.zeros((len(pixels),128),np.float32),1.),
        image=lambda n:np.zeros((1000,1000,3),np.uint8),
        depth=lambda n:(np.full((256,256),2.),np.full((256,256),100,np.uint8),1000,1000,k,np.eye(4)),
        unproject=lambda n,p:np.column_stack(((p[:,0]-500)*2/500,(p[:,1]-500)*2/500,np.full(len(p),2.))))
    (tmp_path/'matches').mkdir()
    result=driver.verify_planar_disjoint(session,session,{'outside_id':1,'lobby_id':2},tmp_path)
    assert result['accepted'] and len(calls)==2
    assert result['outside_geometry']['std_m'][2]<1e-12
    assert not result['outside_geometry']['previous_volume_gate_passed']
    assert result['outside_geometry']['planar_aware_gate_passed']
    assert result['excluded_depth_pixels'][0]>0
    assert result['heldout_reprojection']['median_px']<1e-8


def test_secondary_edges_are_downweighted_and_cannot_replace_backbone_holdout():
    from revalidate_cross_sessions import expand_drift_hypothesis
    poses={i:np.eye(4) for i in range(1,6)}
    backbone=[{'node_a':i,'node_b':i,'T_a_b':np.eye(4).tolist(),'local_geometry_verified':True,
               'independent_support_count':4,'independent_support_member':True,'hypothesis_coherence':{},
               'held_out':i==4,'joint_holdout':i==4,'initialization_allowed':i!=4} for i in range(1,5)]
    extra={'node_a':5,'node_b':5,'T_a_b':np.eye(4).tolist(),'local_geometry_verified':True}
    rows=expand_drift_hypothesis(backbone,backbone+[extra],poses,poses)
    assert len(rows)==5 and rows[3]['held_out']
    assert not rows[-1]['initialization_allowed'] and not rows[-1]['independent_support_member']
    assert rows[-1]['sigma_translation_m']==.2 and rows[-1]['sigma_rotation_deg']==6
    assert not rows[-1]['eligible_for_joint'] and rows[-1]['hypothesis_eligible']
