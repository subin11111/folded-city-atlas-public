import csv
import numpy as np
from scipy.spatial.transform import Rotation
from optimize_floor2_z_only import apply_z_offsets, z_basis, write_trajectory_exact, paired_height_target


def test_z_basis_has_exact_hard_fixed_and_outside_zeros():
    ids = list(range(1, 1219))
    _, _, basis = z_basis(ids, [216, 1055], [216, 553, 1055], 20)
    for node in [1, 215, 216, 553, 1055, 1218]:
        assert not basis[node-1].any()
    assert basis[727].any()


def test_only_z_changes_never_xy_or_any_rotation():
    p = np.eye(4); p[:3, :3] = Rotation.from_euler('xyz', [20, -10, 45], degrees=True).as_matrix()
    p[:3, 3] = [3, 4, 5]
    q = apply_z_offsets({7: p}, [7], [.21])[7]
    assert np.array_equal(q[:3, :3], p[:3, :3])
    assert np.array_equal(q[:2, 3], p[:2, 3])
    assert q[2, 3] == 5.21
    assert p[2, 3] == 5


def test_csv_non_z_fields_preserved_exactly(tmp_path):
    source = tmp_path/'old.csv'; dest = tmp_path/'new.csv'
    source.write_text('id,stamp,x,y,z,qx,qy,qz,qw\n1,123.000,1.20000,-2.100,3.0,0.00,0.0,0.000,1.000\n')
    write_trajectory_exact(source, dest, {1: .25})
    a = next(csv.DictReader(source.open())); b = next(csv.DictReader(dest.open()))
    assert all(a[k] == b[k] for k in a if k != 'z')
    assert float(b['z']) == 3.25


def test_paired_height_target_has_correct_sign_and_common_xy_support():
    x, y = np.meshgrid(np.linspace(-1, 1, 8), np.linspace(-1, 1, 8))
    points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    patch = {'training': points, 'normal': np.array([0., 0., 1.]), 'center': np.zeros(3)}
    first, second = np.eye(4), np.eye(4); second[2, 3] = .2
    target, evidence = paired_height_target(patch, patch, first, second)
    assert evidence['accepted']
    assert abs(target+.2) < 1e-12


def test_no_height_fit_without_actual_finite_xy_overlap():
    x, y = np.meshgrid(np.linspace(-1, 1, 8), np.linspace(-1, 1, 8))
    points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    patch = {'training': points, 'normal': np.array([0., 0., 1.]), 'center': np.zeros(3)}
    first, second = np.eye(4), np.eye(4); second[0, 3] = 5.
    target, evidence = paired_height_target(patch, patch, first, second)
    assert target is None
    assert not evidence['accepted']
