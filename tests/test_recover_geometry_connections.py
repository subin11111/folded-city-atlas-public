import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from recover_geometry_connections import fit_pnp, split_asymmetric, geometry_at_pose


def test_pnp_preserves_node_optical_extrinsic_and_direction():
    rng = np.random.default_rng(419)
    source = rng.uniform([-1., -1., 2.], [1., 1., 4.], (80, 3))
    optical = np.eye(4)
    optical[:3, :3] = Rotation.from_euler('xyz', [.06, -.1, .2]).as_matrix()
    optical[:3, 3] = [.2, -.13, .09]
    local = np.eye(4)
    local[:3, :3] = Rotation.from_euler('xyz', [.3, -.2, 1.]).as_matrix()
    local[:3, 3] = [.1, -.3, .2]
    k = np.array([[500., 0., 360.], [0., 500., 480.], [0., 0., 1.]])
    pixels, _ = cv2.projectPoints(source, Rotation.from_matrix(optical[:3,:3]).as_rotvec(),
                                 optical[:3,3], k, None)
    measured, report = fit_pnp(source, pixels.reshape(-1,2), k, local, 19)
    assert report['accepted']
    np.testing.assert_allclose(measured, local@optical, atol=1e-5)


def test_asymmetric_holdout_excludes_both_depth_neighborhoods():
    pixels = np.array([[x, y] for x in range(4, 75, 4) for y in range(4,75,4)])
    train, held, masks = split_asymmetric(pixels, pixels, (80,80), (80,80), 17)
    assert not set(train) & set(held)
    for mask in masks:
        assert not mask[pixels[train,1], pixels[train,0]].any()
        assert mask[pixels[held,1], pixels[held,0]].all()


def test_geometry_at_pose_uses_source_to_target_convention():
    rng = np.random.default_rng(70)
    source = rng.normal(size=(100,3))
    transform = np.eye(4)
    transform[:3,3] = [1.,2.,3.]
    target = source + transform[:3,3]
    report = geometry_at_pose(target, source, transform)
    assert report['symmetric_overlap'] == 1.
    assert report['symmetric_rmse_m'] < 1e-10
