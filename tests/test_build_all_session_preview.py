import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from build_all_session_preview import checked_rigid, compose_outside
from explore_lobby_bridge_attachment import yaw_transform


def test_chained_transform_maps_outside_to_existing_joint_frame():
    joint_bridge = yaw_transform([1.,2.,3.,.5])
    outside_bridge = yaw_transform([4.,5.,6.,1.2])
    p_bridge = np.array([.2,.3,.4,1.])
    p_outside = outside_bridge @ p_bridge
    np.testing.assert_allclose(compose_outside(joint_bridge,outside_bridge)@p_outside,joint_bridge@p_bridge)


def test_reject_scaling_and_reflection():
    for scale in (2., -1.):
        invalid = np.eye(4); invalid[0,0] = scale
        with pytest.raises(ValueError,match='forbidden'):
            checked_rigid(invalid)
