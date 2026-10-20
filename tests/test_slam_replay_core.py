from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import zlib

from slam_replay_core import eligible_edges, decode_rgbd, PrefixOptimizer


def test_no_future_loop_or_odometry_edges_enter_prefix():
    a=SimpleNamespace(a=1,b=3); b=SimpleNamespace(a=3,b=8)
    assert eligible_edges([a,b],{1,3}) == [a]
    assert eligible_edges([a,b],{1}) == []


def test_invalid_confidence_is_not_silently_promoted():
    depth=np.ones((12,12),'<f4')
    packed=depth.view('u1').reshape(12,12,4)
    db=cv2.imencode('.png',packed)[1].tobytes()
    image=cv2.imencode('.jpg',np.zeros((12,12,3),np.uint8))[1].tobytes()
    with pytest.raises(ValueError,match='confidence'):
        decode_rgbd((image,db,zlib.compress(b'bad'),b''))


def test_correction_does_not_cross_missing_odometry_connection():
    raw={n:np.eye(4) for n in [1,2,8]}
    session=SimpleNamespace(raw=raw,odom=[SimpleNamespace(a=1,b=2)])
    b=PrefixOptimizer(session); b.seen={1,2,8}; b.poses={1:np.eye(4)}
    b.poses[1][0,3]=2
    b.extend([2,8])
    assert b.poses[2][0,3]==2
    assert b.poses[8][0,3]==0
