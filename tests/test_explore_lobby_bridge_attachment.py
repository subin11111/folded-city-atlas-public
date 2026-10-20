import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from explore_lobby_bridge_attachment import yaw_transform, fit_placement


def test_rigid_placement_direction_and_height():
    expected = yaw_transform([2., -4., .8, .5])
    a = {1:yaw_transform([.5, .3, 1., .1]), 2:yaw_transform([2., 3., 4., -.2])}
    b = {4:yaw_transform([-.3, 1., 2., -.1]), 5:yaw_transform([4., 5., 6., .8])}
    rows = [{'outside_id':x,'lobby_id':y,'lobby_node_to_outside_node':
             (np.linalg.inv(a[x])@expected@b[y]).tolist()} for x,y in [(1,4),(2,5)]]
    actual, report = fit_placement(rows,a,b)
    np.testing.assert_allclose(actual,expected,atol=1e-7)
    assert report['source_pose_shape_preserved']
    np.testing.assert_allclose(actual[2,:], [0,0,1,.8], atol=1e-7)
