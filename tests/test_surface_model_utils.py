import json
from pathlib import Path
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from surface_model_utils import atlas_uv, project, select_frames, unproject, write_glb


def test_camera_projection_roundtrip():
    depth = np.array([[1., 2.], [3., 4.]])
    k = np.array([[2., 0., .5], [0., 3., .5], [0., 0., 1.]])
    pose = np.eye(4); pose[:3, 3] = [4., -2., 1.]
    points = unproject(depth, k).reshape(-1, 3)+pose[:3, 3]
    uv, z = project(points, k, pose)
    np.testing.assert_allclose(uv, [[0., 0.], [1., 0.], [0., 1.], [1., 1.]], atol=1e-12)
    np.testing.assert_allclose(z, depth.ravel())


def test_native_atlas_coordinates_no_photo_resizing():
    uv = np.array([[0., 0.], [719., 959.]])
    actual = atlas_uv(uv, 6, (960, 720, 3))
    np.testing.assert_allclose(actual*4096, uv+[720., 960.])
    with pytest.raises(ValueError, match='do not fit'):
        atlas_uv(uv, 0, (2000, 2000, 3))


def test_texture_views_do_not_reintroduce_rejected_frames():
    ids = [1, 3, 5, 8, 10, 12, 14]
    chosen = select_frames(ids, 4, [3, 7, 10])
    assert set(chosen)<=set(ids)
    assert {3, 10}<=set(chosen)
    assert len(chosen)<=4


def test_glb_header_geometry_and_vertex_rgb(tmp_path):
    path = tmp_path/'mesh.glb'
    v = np.eye(3); t = np.array([[0, 1, 2]])
    report = write_glb(path, v, t, np.tile([0, 0, 1.], (3, 1)), v)
    raw = path.read_bytes(); magic, version, length = struct.unpack('<4sII', raw[:12])
    assert magic==b'glTF' and version==2 and length==len(raw)==report['bytes']
    json_length, kind = struct.unpack('<I4s', raw[12:20]); assert kind==b'JSON'
    d = json.loads(raw[20:20+json_length]); primitive = d['meshes'][0]['primitives'][0]
    assert 'COLOR_0' in primitive['attributes']
    assert d['materials'][0]['doubleSided']
    with pytest.raises(FileExistsError):
        write_glb(path, v, t, v, v)


def test_texture_glb_keeps_unassigned_faces_as_original_rgb(tmp_path):
    from PIL import Image
    image = tmp_path/'atlas.png'; Image.new('RGB', (2, 2)).save(image)
    path = tmp_path/'mesh.glb'; v = np.eye(3); triangles = np.array([[0, 1, 2], [0, 2, 1]])
    result = write_glb(path, v, triangles, v, v, face_pages=np.array([-1, 0]),
        face_uv=np.zeros((2, 3, 2)), atlas_paths=[image])
    assert result['textured_faces']==1
    raw = path.read_bytes(); size = struct.unpack('<I', raw[12:16])[0]; d=json.loads(raw[20:20+size])
    assert len(d['meshes'][0]['primitives'])==2
    assert 'COLOR_0' in d['meshes'][0]['primitives'][0]['attributes']
    assert 'TEXCOORD_0' in d['meshes'][0]['primitives'][1]['attributes']
