#!/usr/bin/env python3
"""Create reversible shadow-colour display candidates without changing geometry."""
import argparse
import copy
import gc
import io
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from optimize_session import sha256_file, utc_now
from refine_display_edges import accessor, unpack, write_glb
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]
LUMA = np.array([.2126, .7152, .0722], np.float32)


def shadow_lift(linear_rgb, cfg):
    """Lift luminance neutrally; preserve absolute chroma, not amplified noise."""
    if cfg.get('method') == 'colour_preserving_denoised':
        from refine_shadow_colour import colour_shadow_lift
        return colour_shadow_lift(linear_rgb, cfg)
    rgb = np.asarray(linear_rgb, np.float32)
    if not np.isfinite(rgb).all() or rgb.min(initial=0) < 0 or rgb.max(initial=0) > 1.00001:
        raise ValueError('Expected finite linear RGB in [0,1]')
    y = np.clip(rgb @ LUMA, 0, 1)
    gain = 1 + cfg['lift'] * (1-y)**cfg['highlight_power'] / (y + cfg['knee_linear_luminance'])
    gain = np.minimum(gain, cfg['maximum_linear_gain'])
    delta = np.minimum(y*(gain-1), 1-rgb.max(axis=-1))
    return np.clip(rgb + delta[..., None], 0, 1)


def append_blob(doc, payload, blob):
    payload.extend(b'\0'*(-len(payload) % 4))
    offset = len(payload)
    payload.extend(blob)
    doc['bufferViews'].append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(blob)})
    return len(doc['bufferViews'])-1


def lift_glb(source, target, cfg):
    doc, payload = unpack(source)
    primitives = [p for mesh in doc['meshes'] for p in mesh['primitives']]
    images, colours = set(), set()
    for primitive in primitives:
        material = doc['materials'][primitive['material']].get('pbrMetallicRoughness', {})
        if material.get('baseColorFactor', [1, 1, 1, 1]) != [1, 1, 1, 1]:
            raise ValueError('Non-unit colour factor needs final-colour baking')
        tex = material.get('baseColorTexture', {}).get('index')
        colour = primitive['attributes'].get('COLOR_0')
        if tex is not None and colour is not None:
            raise ValueError('Do not apply the tone curve twice to multiplied texture/vertex colour')
        if tex is not None:
            images.add(doc['textures'][tex]['source'])
        if colour is not None:
            colours.add(colour)
    reports = []
    # Existing colour bytes are retained; only active image descriptors point
    # to appended PNGs. Geometry and UV buffer streams are not rewritten.
    for image_id in sorted(images):
        image = doc['images'][image_id]
        view = doc['bufferViews'][image['bufferView']]
        start = view.get('byteOffset', 0)
        blob = bytes(payload[start:start+view['byteLength']])
        original = np.asarray(Image.open(io.BytesIO(blob)).convert('RGBA'))
        result = original.copy()
        rgb = original[:, :, :3]
        # Chunking avoids several 4K-atlas-sized floating-point temporaries.
        for first in range(0, len(rgb), 256):
            if cfg.get('method') == 'colour_preserving_denoised':
                from refine_shadow_colour import correct_shadow_texture
                # Halo makes the filter independent of row-chunk boundaries.
                lo, hi = max(0, first-8), min(len(rgb), first+264)
                corrected = correct_shadow_texture(rgb[lo:hi], original[lo:hi, :, 3], cfg)
                result[first:first+256, :, :3] = corrected[first-lo:min(first+256, len(rgb))-lo]
            else:
                sample = srgb_to_linear(rgb[first:first+256]/255.)
                mapped = shadow_lift(sample, cfg)
                result[first:first+256, :, :3] = np.uint8(linear_to_srgb(mapped)*255+.5)
        output = io.BytesIO()
        Image.fromarray(result, 'RGBA').save(output, format='PNG')
        image['bufferView'] = append_blob(doc, payload, output.getvalue())
        image['mimeType'] = 'image/png'
        reports.append({'image': image_id, 'size': [int(rgb.shape[1]), int(rgb.shape[0])],
                        'changed_pixels': int(np.any(result[:, :, :3] != rgb, axis=2).sum()),
                        'source_mean_srgb': float(rgb.mean()/255.), 'after_mean_srgb': float(result[:, :, :3].mean()/255.),
                        'alpha_exact': bool(np.array_equal(original[:, :, 3], result[:, :, 3]))})
    replacements = {}
    for index in sorted(colours):
        row = doc['accessors'][index]
        if row['componentType'] != 5126 or row['type'] != 'VEC3':
            raise ValueError('Unexpected baked colour format; preserve rather than misinterpret')
        old = accessor(doc, payload, index).copy()
        new = shadow_lift(old, cfg).astype('<f4')
        new_view = append_blob(doc, payload, new.tobytes())
        updated = copy.deepcopy(row)
        updated['bufferView'], updated['byteOffset'] = new_view, 0
        updated['min'], updated['max'] = new.min(0).tolist(), new.max(0).tolist()
        doc['accessors'].append(updated)
        replacements[index] = len(doc['accessors'])-1
    for primitive in primitives:
        index = primitive['attributes'].get('COLOR_0')
        if index in replacements:
            primitive['attributes']['COLOR_0'] = replacements[index]
    write_glb(target, doc, payload)
    return {'active_images': len(images), 'baked_colour_streams': len(colours), 'images': reports}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/display_shadow_lift.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    base = ROOT/cfg['source_bundle']
    run = json.loads((base/'manifest.json').read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    sessions, cleanup = {}, {}
    for name, spec in run['sessions'].items():
        folder = args.output/name
        folder.mkdir()
        report = lift_glb(base/name/'clean_textured.glb', folder/'clean_textured.glb', cfg)
        shutil.copyfile(base/name/'clean_structural.ply', folder/'clean_structural.ply')
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb']:
                shutil.copyfile(path, folder/path.name)
        sessions[name] = {**spec, 'shadow_lift_report': report}
        cleanup[name] = {'ply_sha256': sha256_file(folder/'clean_structural.ply'),
                         'glb_sha256': sha256_file(folder/'clean_textured.glb')}
        print(f'Shadow lift: {name}, {report["active_images"]} images / {report["baked_colour_streams"]} vertex streams', flush=True)
        gc.collect()
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, args.output/path.name)
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions,
              'shadow_lift': {'config': cfg}, 'scope': cfg['scope'],
              'previous_material_manifest': str((base/'manifest.json').resolve()),
              'shadow_source_manifest': str((base/'manifest.json').resolve()),
              'inputs_sha256': {**run['inputs_sha256'], **{str(p.resolve()): sha256_file(p) for p in
                                [base/'manifest.json', args.config, Path(__file__)]}}}
    (args.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(args.output/'manifest.json'),
                                                     'sessions': cleanup}, indent=2)+'\n')


if __name__ == '__main__':
    main()
