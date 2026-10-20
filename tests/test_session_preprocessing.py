import sys
from pathlib import Path
import zlib

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from session_preprocessing import BASE, decode_confidence, depth_mask
from build_session_portfolios import normalize


def test_confidence_trailer_is_not_invented_high_confidence():
    values=np.array([[0,50],[100,0]],np.uint8)
    for raw in (values.tobytes(),values.tobytes()+bytes(12)):
        np.testing.assert_array_equal(decode_confidence(zlib.compress(raw),(2,2)),values)
    with pytest.raises(ValueError):decode_confidence(None,(2,2))


def test_stair_profile_preserves_step_pixels_without_smoothing():
    depth=np.ones((7,7),np.float32);depth[:,4:]=1.4;original=depth.copy()
    profile={'confidence':50,'max_depth':4.,'median_gate':.22,'edge_gate':0.}
    mask=depth_mask(depth,np.full(depth.shape,100),profile)
    assert mask.all()
    np.testing.assert_array_equal(depth,original)


def test_depth_spike_removed_without_changing_other_points():
    depth=np.ones((7,7));depth[3,3]=2.
    profile={'confidence':50,'max_depth':4.,'median_gate':.15,'edge_gate':0.}
    mask=depth_mask(depth,np.full(depth.shape,100),profile)
    assert not mask[3,3]
    assert mask.sum()==48


def test_preprocessing_does_not_make_invalid_depth_valid():
    depth=np.array([[np.nan,0.,5.],[1.,2.,3.]])
    confidence=np.array([[100,100,100],[0,50,100]])
    mask=depth_mask(depth,confidence,BASE)
    np.testing.assert_array_equal(mask,[[False,False,False],[False,True,True]])


def test_display_gauge_preserves_internal_relative_poses():
    first=np.eye(4);second=np.eye(4);second[0,3]=2.
    reference={1:np.eye(4),2:second};reference[1][1,3]=3.
    output,gauge=normalize({1:first,2:second},reference,[1,2])
    np.testing.assert_allclose(np.linalg.inv(output[1])@output[2],second)
    np.testing.assert_allclose(output[1],reference[1])
