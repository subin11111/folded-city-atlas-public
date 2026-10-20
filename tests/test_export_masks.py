"""End-to-end checks for calibrated RGB mask propagation and read-only export."""
import hashlib
import sqlite3
import struct
import subprocess
import sys
import zlib
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from render_session_report import read_ply
from mask_dynamic_objects import polygons_to_mask


def test_rgb_exclusion_and_pose_override(tmp_path):
    db = tmp_path / 'tiny.db'
    connection = sqlite3.connect(db)
    connection.executescript('CREATE TABLE Node (id INTEGER, stamp REAL, pose BLOB);'
                             'CREATE TABLE Data (id INTEGER, image BLOB, depth BLOB, depth_confidence BLOB, calibration BLOB);')
    pose = np.eye(4, dtype=np.float32)
    pose[0, 3] = 10
    calibration = bytearray(164)
    struct.pack_into('<II', calibration, 16, 2, 2)
    calibration[44:116] = np.eye(3, dtype='<f8').tobytes()
    calibration[116:164] = np.eye(4, dtype='<f4')[:3].tobytes()
    depth = np.ones((2, 2), dtype='<f4')
    depth_blob = cv2.imencode('.png', depth.view(np.uint8).reshape(2, 2, 4))[1].tobytes()
    image_blob = cv2.imencode('.png', np.full((2, 2, 3), 127, dtype=np.uint8))[1].tobytes()
    connection.execute('INSERT INTO Node VALUES (?,?,?)', (1, 1.0, pose[:3].tobytes()))
    connection.execute('INSERT INTO Data VALUES (?,?,?,?,?)',
                       (1, image_blob, depth_blob, zlib.compress(bytes([100] * 4)), bytes(calibration)))
    connection.commit(); connection.close()
    before = hashlib.sha256(db.read_bytes()).hexdigest()
    masks = tmp_path / 'masks'; masks.mkdir()
    cv2.imwrite(str(masks / '1.png'), np.array([[255, 0], [0, 0]], dtype=np.uint8))
    output = tmp_path / 'cloud.ply'
    command = [sys.executable, str(Path(__file__).resolve().parents[1] / 'src/export_rtabmap_db.py'),
               str(db), '--cloud', str(output), '--trajectory', str(tmp_path / 'path.csv'),
               '--summary', str(tmp_path / 'summary.json'), '--mask-dir', str(masks), '--frame-stride', '1']
    subprocess.run(command, check=True, capture_output=True)
    cloud = read_ply(output)
    points = np.column_stack([cloud[c] for c in ('x', 'y', 'z')])
    assert len(points) == 3
    assert {tuple(p) for p in points} == {(11., 0., 1.), (10., 1., 1.), (11., 1., 1.)}
    assert hashlib.sha256(db.read_bytes()).hexdigest() == before
    (masks / '1.png').unlink()
    assert subprocess.run(command, capture_output=True).returncode != 0


def test_empty_polygon_mask_is_not_fabricated_detection():
    mask = polygons_to_mask([], (50, 60), 5)
    assert mask.shape == (50, 60) and not np.any(mask)


def test_instance_polygon_dilation():
    polygon = np.array([[10, 10], [20, 10], [20, 20], [10, 20]])
    plain = polygons_to_mask([polygon], (40, 40), 0)
    dilated = polygons_to_mask([polygon], (40, 40), 2)
    assert np.all(dilated[plain > 0] == 255)
    assert np.count_nonzero(dilated) > np.count_nonzero(plain)
    assert not dilated[0, 0]
