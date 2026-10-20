from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from analyze_visual_shortlist import distribution, pose_difference


def test_missing_measurements_are_not_zero_accuracy():
    assert distribution([])=={'count':0,'median':None,'p90':None,'maximum':None}


def test_pose_comparison_keeps_translation_and_rotation_units_separate():
    reference={1:np.eye(4),2:np.eye(4)}
    candidate={k:v.copy() for k,v in reference.items()}
    candidate[2][0,3]=.01
    candidate[2][:3,:3]=Rotation.from_euler('z',2,degrees=True).as_matrix()
    d=pose_difference(reference,candidate)
    assert d['translation_m']['maximum']==pytest.approx(.01)
    assert d['rotation_deg']['maximum']==pytest.approx(2.)
    assert not d['identical_to_1mm_and_0p01deg']


def test_pose_comparison_refuses_different_observation_support():
    with pytest.raises(ValueError):pose_difference({1:np.eye(4)},{1:np.eye(4),2:np.eye(4)})


def test_exact_same_poses_are_reported_as_same_not_algorithm_improvement():
    poses={1:np.eye(4),2:np.eye(4)}
    poses[2][2,3]=3.5
    d=pose_difference(poses,poses)
    assert d['translation_m']['maximum']==0
    assert d['rotation_deg']['maximum']==0
    assert d['identical_to_1mm_and_0p01deg']
