#!/usr/bin/env python3
"""Low-frequency feather only around known ceiling photo-label boundaries."""
import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from level_floor_materials import replace_floor_references
from optimize_session import sha256_file, utc_now
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]


def feather_photo_boundaries(rgb, labels, surface, pixel_m, cfg):
    edges = np.zeros(labels.shape, bool)
    edges[1:] |= labels[1:] != labels[:-1]
    edges[:-1] |= labels[1:] != labels[:-1]
    edges[:, 1:] |= labels[:, 1:] != labels[:, :-1]
    edges[:, :-1] |= labels[:, 1:] != labels[:, :-1]
    edges &= surface
    if not edges.any():
        return rgb.copy(), np.zeros(surface.shape, np.float32)
    distance = ndimage.distance_transform_edt(~edges)*pixel_m
    weight = np.clip(1-distance/cfg['band_width_m'], 0, 1)
    weight = weight*weight*(3-2*weight)*surface
    linear = srgb_to_linear(rgb/255.).astype(np.float32)
    sigma = max(1., cfg['low_frequency_sigma_m']/pixel_m)
    valid = surface.astype(np.float32)
    smooth = cv2.GaussianBlur(linear*valid[:, :, None], (0, 0), sigma)
    smooth /= np.maximum(cv2.GaussianBlur(valid, (0, 0), sigma), 1e-6)[:, :, None]
    detail = linear-cv2.GaussianBlur(linear, (0, 0), cfg['detail_sigma_px'])
    fixed = linear*(1-weight[:, :, None])+(smooth+detail)*weight[:, :, None]
    result = np.uint8(linear_to_srgb(np.clip(fixed, 0, 1))*255+.5)
    limit = cfg['maximum_srgb_change']
    result = np.clip(result.astype(np.int16), rgb.astype(np.int16)-limit,
                     rgb.astype(np.int16)+limit).clip(0, 255).astype(np.uint8)
    result[weight == 0] = rgb[weight == 0]
    return result, weight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/photo_boundary_finish.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    original_hash = sha256_file(ROOT/cfg['source_roof_texture'])
    report = next(r for r in run['sessions']['floor2']['texture_edge_reports'] if r['source_sha256'] == original_hash)
    original = source/'floor2/edge_textures'/f'image_{report["image"]:04d}.png'
    labels = np.asarray(Image.open(ROOT/cfg['photo_labels']))
    rgb = np.asarray(Image.open(original).convert('RGB'))
    labels = cv2.resize(labels, rgb.shape[1::-1], interpolation=cv2.INTER_NEAREST)
    pack = np.load(ROOT/cfg['surface_mask'])
    mask = cv2.resize(pack['mask'].astype(np.uint8), rgb.shape[1::-1], interpolation=cv2.INTER_NEAREST) > 0
    pixel_m = max(pack['mask'].shape[1]*pack['cell']/rgb.shape[1], pack['mask'].shape[0]*pack['cell']/rgb.shape[0])
    result, band = feather_photo_boundaries(rgb, labels, mask, float(pixel_m), cfg)
    args.output.mkdir(parents=True, exist_ok=False)
    target = args.output/'ceiling_photo_edge.png'
    Image.fromarray(result).save(target)
    Image.fromarray(np.uint8(band*255+.5)).save(args.output/'ceiling_photo_edge_band.png')
    inputs = {str(p.resolve()): sha256_file(p) for p in [Path(__file__), args.config, source/'manifest.json', original, ROOT/cfg['photo_labels'], ROOT/cfg['surface_mask'], ROOT/cfg['source_roof_texture'], ROOT/'src/level_floor_materials.py']}
    replacements = {sha256_file(original): target}
    sessions, cleanup = {}, {}
    for name, spec in run['sessions'].items():
        folder = args.output/name
        folder.mkdir()
        for p in (source/name).iterdir():
            if p.is_file() and p.name != 'clean_textured.glb':
                shutil.copyfile(p, folder/p.name)
        hits = replace_floor_references(source/name/'clean_textured.glb', folder/'clean_textured.glb', replacements)
        sessions[name] = {**spec, 'photo_boundary_texture_references_changed': hits}
        cleanup[name] = {**json.loads((source/'cleanup.json').read_text())['sessions'][name],
                         'glb_sha256': sha256_file(folder/'clean_textured.glb')}
        for p in [source/name/'clean_textured.glb', source/name/'clean_structural.ply']:
            inputs[str(p.resolve())] = sha256_file(p)
    for p in source.iterdir():
        if p.is_file() and p.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(p, args.output/p.name)
    manifest = {**run, 'created_utc': utc_now(), 'sessions': sessions,
                'inputs_sha256': {**run['inputs_sha256'], **inputs},
                'photo_boundary_finish': {'config': cfg, 'source_roof_sha256': original_hash,
                                         'original_image_index': report['image'], 'new_image_sha256': sha256_file(target),
                                         'edge_source_manifest': str((source/'manifest.json').resolve()),
                                         'scope': cfg['scope']}}
    assert sessions['floor2']['photo_boundary_texture_references_changed'], 'Roof image was not referenced'
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (args.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(args.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')
    for path, digest in manifest['inputs_sha256'].items():
        assert sha256_file(Path(path)) == digest, 'Input changed: '+path
    print('Ceiling source-photo edge band softened; all v17 geometry and other textures retained.', flush=True)


if __name__ == '__main__':
    main()
