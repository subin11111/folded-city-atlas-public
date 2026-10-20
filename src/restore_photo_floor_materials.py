#!/usr/bin/env python3
"""Restore photographic floor atlases while retaining completed surface geometry."""
import argparse
import json
from pathlib import Path
import shutil

from level_floor_materials import replace_floor_references
from optimize_session import sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/floor_photo_preservation.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    assert cfg['strategy'] == 'original_floor_atlas_exact_restore'
    base = ROOT/cfg['source_bundle']
    run = json.loads((base/'manifest.json').read_text())
    previous = ROOT/cfg['previous_material_bundle']/'manifest.json'
    args.output.mkdir(parents=True, exist_ok=False)
    chart_dir = args.output/'floor_charts'
    chart_dir.mkdir()
    inputs = {
        str(path.resolve()): sha256_file(path) for path in [
            Path(__file__), args.config, base/'manifest.json', previous,
            ROOT/'src/level_floor_materials.py',
        ]
    }
    replacements, reports = {}, []
    for chart in cfg['charts']:
        source = ROOT/'outputs/student_union_topology_fixed_v2'/chart['session']/(chart['id']+'_texture.png')
        target = chart_dir/(chart['id']+'.png')
        shutil.copyfile(source, target)
        digest = sha256_file(source)
        assert sha256_file(target) == digest
        replacements[digest] = target
        inputs[str(source.resolve())] = digest
        reports.append({**chart, 'source_texture_sha256': digest, 'new_texture_sha256': digest,
                        'exact_original_atlas_bytes': True, 'uniform_colour_replacement': False})
    cleanup_before = json.loads((base/'cleanup.json').read_text())['sessions']
    sessions, cleanup = {}, {}
    for name, session in run['sessions'].items():
        dest = args.output/name
        dest.mkdir()
        shutil.copyfile(base/name/'clean_structural.ply', dest/'clean_structural.ply')
        matches = replace_floor_references(base/name/'clean_textured.glb', dest/'clean_textured.glb', replacements)
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb', 'retained.glb']:
                shutil.copyfile(path, dest/path.name)
        sessions[name] = {**session, 'floor_material_references_changed': matches}
        cleanup[name] = {**cleanup_before[name], 'ply_sha256': sha256_file(dest/'clean_structural.ply'),
                         'glb_sha256': sha256_file(dest/'clean_textured.glb')}
        for path in [base/name/'clean_structural.ply', base/name/'clean_textured.glb']:
            inputs[str(path.resolve())] = sha256_file(path)
        print('Original photographic floor restored: '+name, flush=True)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, args.output/path.name)
    manifest = {**run, 'created_utc': utc_now(), 'sessions': sessions,
                'inputs_sha256': {**run['inputs_sha256'], **inputs},
                'floor_photo_preservation': True, 'floor_material_strategy': cfg['strategy'],
                'floor_material_source_manifest': str((base/'manifest.json').resolve()),
                'previous_material_manifest': str(previous.resolve()),
                'floor_material_config': cfg, 'floor_material_reports': reports, 'scope': cfg['scope']}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (args.output/'cleanup.json').write_text(json.dumps({
        'sessions': cleanup, 'source_manifest_sha256': sha256_file(args.output/'manifest.json'),
    }, indent=2)+'\n')
    for path, digest in manifest['inputs_sha256'].items():
        assert sha256_file(Path(path)) == digest, 'Input changed: '+path


if __name__ == '__main__':
    main()
