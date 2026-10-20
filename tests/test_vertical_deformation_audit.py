import numpy as np
from scipy.spatial.transform import Rotation

from audit_selected_vertical_deformation import first_pose_align, relative_change, rigid_position_residual


def poses():
    values = {}
    for i, point in enumerate([[0, 0, 0], [2, 0, 0], [0, 3, 0], [1, 1, 4], [-2, 1, 3]]):
        pose = np.eye(4)
        pose[:3, 3] = point
        values[i] = pose
    return values


def test_global_tilt_and_offset_not_internal_deformation():
    ref = poses()
    gauge = np.eye(4)
    gauge[:3, :3] = Rotation.from_euler('xyz', [8, -4, 30], degrees=True).as_matrix()
    gauge[:3, 3] = [4, -2, 9]
    candidate = {n: gauge @ pose for n, pose in ref.items()}
    aligned, _, _ = first_pose_align(ref, candidate)
    assert all(np.allclose(aligned[n], ref[n]) for n in ref)
    assert abs(relative_change(ref, candidate, 1, 4)['delta_z_m']) < 1e-12
    _, _, residual = rigid_position_residual(ref, candidate)
    assert np.max(abs(residual)) < 1e-12


def test_local_z_warp_survives_best_rigid_fit():
    ref = poses()
    candidate = {n: pose.copy() for n, pose in ref.items()}
    candidate[4][2, 3] += .3
    assert abs(relative_change(ref, candidate, 1, 4)['delta_z_m']-.3) < 1e-12
    _, _, residual = rigid_position_residual(ref, candidate)
    assert np.max(abs(residual[:, 2])) > .03


def test_relative_change_ignores_candidate_world_gauge():
    ref = poses()
    candidate = {n: pose.copy() for n, pose in ref.items()}
    candidate[4][2, 3] += .2
    gauge = np.eye(4)
    gauge[:3, :3] = Rotation.from_euler('xyz', [17, 8, -45], degrees=True).as_matrix()
    gauge[:3, 3] = [-9, 2, 3]
    shifted = {n: gauge @ pose for n, pose in candidate.items()}
    assert np.allclose(relative_change(ref, candidate, 2, 4)['delta_xyz_m'],
                       relative_change(ref, shifted, 2, 4)['delta_xyz_m'])
