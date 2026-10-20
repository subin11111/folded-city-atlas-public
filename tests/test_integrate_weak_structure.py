import copy
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from integrate_weak_structure import add_factor


def inputs():
    a,b='floor2:frame:00548','floor2:frame:00981'
    edge=dict(id='original',**{'from':a,'to':b},type='loop',role='train',
              measurement=np.eye(4).tolist(),covariance=np.eye(6).tolist())
    graph=dict(schema_version=1,submaps=[dict(id=n,session_id='floor2',pose=np.eye(4).tolist()) for n in (a,b)],
               edges=[edge],metadata={})
    measurement=dict(fit_executed=True,original_databases_unchanged=True,
        solver=dict(success=True,active_bounds=0,pose_data_rank_after_landmark_elimination=6),
        withheld_cloud_before=dict(clipped_rmse_m=.1,overlap=.5),
        withheld_cloud_after=dict(clipped_rmse_m=.09,overlap=.6),
        test_before=dict(pixel_observations=8,pixel_median_px=12,pixel_p90_px=27),
        test_after=dict(pixel_observations=8,pixel_median_px=4,pixel_p90_px=20),
        annotation=dict(session_a='floor2',session_b='floor2',node_a=548,node_b=981),
        click_sensitivity=[dict(translation_m=.31,rotation_deg=2.8)],T_a_b=np.eye(4).tolist())
    return graph,measurement


def test_weak_factor_can_use_improving_but_imperfect_evidence_without_double_counting():
    g,m=inputs();original=copy.deepcopy(g);result=add_factor(g,m)
    assert g==original
    assert len(result['edges'])==1
    added=result['edges'][0]
    assert added['provenance']['archived_factor_ids']==['original']
    assert added['provenance']['sigma_translation_m']==.31
    assert added['provenance']['sigma_rotation_deg']==5
    assert result['metadata']['accepted'] is False


def test_heldout_and_regressing_evidence_cannot_be_sneaked_into_training():
    g,m=inputs();g['edges'][0]['role']='heldout'
    with pytest.raises(ValueError,match='heldout'):add_factor(g,m)
    g,m=inputs();m['withheld_cloud_after']['clipped_rmse_m']=.15
    with pytest.raises(ValueError,match='regresses'):add_factor(g,m)
