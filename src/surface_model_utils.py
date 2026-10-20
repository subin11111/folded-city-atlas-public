"""Geometry/texture helpers independent of the optional Open3D runtime."""
import io
import json
from pathlib import Path
import struct

import cv2
import numpy as np
from PIL import Image

from optimize_session import apply, inverse
from session_preprocessing import decode_confidence, depth_mask
from export_rtabmap_db import calibration_from_blob


def decode_frame(connection, node, profile, pose):
    row = connection.execute('SELECT image,depth,depth_confidence,calibration FROM Data WHERE id=?', (int(node),)).fetchone()
    if row is None:
        raise ValueError('Missing RGB-D frame')
    bgr = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_COLOR)
    packed = cv2.imdecode(np.frombuffer(row[1], np.uint8), cv2.IMREAD_UNCHANGED)
    if bgr is None or packed is None or packed.ndim!=3 or packed.shape[2]!=4:
        raise ValueError('Invalid original RGB-D encoding')
    depth = np.ascontiguousarray(packed).view('<f4').reshape(packed.shape[:2])
    confidence = decode_confidence(row[2], depth.shape)
    valid = depth_mask(depth, confidence, profile)
    width, height, k, local = calibration_from_blob(row[3])
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    kd = k.copy(); kd[0] *= depth.shape[1]/width; kd[1] *= depth.shape[0]/height
    krgb = k.copy(); krgb[0] *= rgb.shape[1]/width; krgb[1] *= rgb.shape[0]/height
    return {'id': int(node), 'rgb': rgb, 'depth': np.where(valid, depth, 0.).astype(np.float32),
            'valid': valid, 'k_depth': kd, 'k_rgb': krgb, 'world_optical': pose@local}


def unproject(depth, intrinsic):
    yy, xx = np.indices(depth.shape)
    return np.stack(((xx-intrinsic[0, 2])*depth/intrinsic[0, 0],
                     (yy-intrinsic[1, 2])*depth/intrinsic[1, 1], depth), axis=-1)


def project(points, intrinsic, world_optical):
    optical = apply(points, inverse(world_optical))
    pixels = optical@intrinsic.T
    uv = pixels[:, :2]/np.maximum(pixels[:, 2:3], 1e-12)
    return uv, optical[:, 2]


def atlas_uv(rgb_uv, slot, image_shape, side=4096, columns=5, rows=4):
    height, width = image_shape[:2]
    if width*columns>side or height*rows>side:
        raise ValueError('Native photographs do not fit the declared atlas')
    x = (slot%columns)*width; y = (slot//columns)*height
    # glTF texture coordinates have their origin at the top-left of the image.
    return (rgb_uv+np.array([x, y]))/side


def select_frames(ids, maximum, preferred):
    ids = sorted(set(ids)); preferred = sorted(set(preferred)&set(ids))
    if len(preferred)>maximum:
        raise ValueError('Too many preferred camera views')
    chosen = set(preferred)
    for i in np.linspace(0, len(ids)-1, min(maximum, len(ids))).astype(int):
        chosen.add(ids[i])
    if len(chosen)>maximum:
        other = sorted(chosen-set(preferred))
        indices = np.linspace(0, len(other)-1, maximum-len(preferred)).astype(int)
        chosen = set(preferred)|{other[i] for i in indices}
    return sorted(chosen)


def detail_candidates(frame, cfg):
    rgb, depth = frame['rgb'], frame['depth']
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    gradient = np.hypot(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    gradient = cv2.resize(gradient, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_AREA)
    jump = np.zeros_like(depth)
    for axis in [0, 1]:
        shifted = np.roll(depth, 1, axis=axis)
        jump = np.maximum(jump, np.where((depth>0)&(shifted>0), abs(depth-shifted), 0))
    # These are visual detail candidates, NOT semantic signs or dynamic-object masks.
    chosen = (depth>0)&((gradient>cfg['gradient_threshold'])|(jump>cfg['depth_jump_m']))
    points = unproject(depth, frame['k_depth'])[chosen]
    colors = cv2.resize(rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_AREA)[chosen]
    return apply(points, frame['world_optical']), colors


def write_glb(path, vertices, triangles, normals, colors, *, face_pages=None, face_uv=None, atlas_paths=()):
    """Write a self-contained glTF 2.0 GLB, with double-sided unlit RGB material.

    Textured corners are deliberately duplicated to preserve image seams. The
    geometry itself is unchanged. Unassigned faces retain original vertex RGB.
    """
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    document = {'asset': {'version': '2.0', 'generator': 'Folded City surface comparison'},
        'extensionsUsed': ['KHR_materials_unlit'], 'scene': 0, 'scenes': [{'nodes': [0]}],
        'nodes': [{'mesh': 0}], 'meshes': [{'primitives': []}], 'materials': [],
        'buffers': [{'byteLength': 0}], 'bufferViews': [], 'accessors': []}
    payload = bytearray()
    def buffer_view(blob, target=None):
        payload.extend(b'\0'*(-len(payload)%4)); offset = len(payload); payload.extend(blob)
        row = {'buffer': 0, 'byteOffset': offset, 'byteLength': len(blob)}
        if target is not None:
            row['target'] = target
        document['bufferViews'].append(row); return len(document['bufferViews'])-1
    def accessor(array, kind):
        array = np.ascontiguousarray(array, dtype='<f4')
        row = {'bufferView': buffer_view(array.tobytes(), 34962), 'componentType': 5126,
               'count': len(array), 'type': kind}
        if kind=='VEC3':
            row.update(min=array.min(axis=0).astype(float).tolist(), max=array.max(axis=0).astype(float).tolist())
        document['accessors'].append(row); return len(document['accessors'])-1
    if atlas_paths:
        document.update(images=[], textures=[], samplers=[{'magFilter': 9729, 'minFilter': 9729,
                                                           'wrapS': 33071, 'wrapT': 33071}])
        for i, atlas in enumerate(atlas_paths):
            document['images'].append({'bufferView': buffer_view(Path(atlas).read_bytes()), 'mimeType': 'image/png'})
            document['textures'].append({'source': i, 'sampler': 0})
    pages = [-1] if face_pages is None else list(np.unique(face_pages))
    for page in pages:
        chosen = np.arange(len(triangles)) if face_pages is None else np.flatnonzero(face_pages==page)
        if not len(chosen):
            continue
        ids = triangles[chosen].reshape(-1)
        attributes = {'POSITION': accessor(vertices[ids], 'VEC3'), 'NORMAL': accessor(normals[ids], 'VEC3')}
        material = {'doubleSided': True, 'extensions': {'KHR_materials_unlit': {}},
                    'pbrMetallicRoughness': {'metallicFactor': 0., 'roughnessFactor': 1.}}
        if page>=0:
            attributes['TEXCOORD_0'] = accessor(face_uv[chosen].reshape(-1, 2), 'VEC2')
            material['pbrMetallicRoughness']['baseColorTexture'] = {'index': int(page)}
        else:
            attributes['COLOR_0'] = accessor(colors[ids], 'VEC3')
        document['materials'].append(material)
        document['meshes'][0]['primitives'].append({'attributes': attributes,
            'material': len(document['materials'])-1, 'mode': 4})
    document['buffers'][0]['byteLength'] = len(payload)
    encoded = json.dumps(document, separators=(',', ':')).encode()
    encoded += b' '*(-len(encoded)%4); payload.extend(b'\0'*(-len(payload)%4))
    length = 12+8+len(encoded)+8+len(payload)
    with path.open('wb') as stream:
        stream.write(struct.pack('<4sII', b'glTF', 2, length))
        stream.write(struct.pack('<I4s', len(encoded), b'JSON')); stream.write(encoded)
        stream.write(struct.pack('<I4s', len(payload), b'BIN\0')); stream.write(payload)
    return {'bytes': length, 'triangles': len(triangles), 'textured_faces': int(np.count_nonzero(face_pages>=0)) if face_pages is not None else 0,
            'unassigned_faces_keep_vertex_rgb': True}
