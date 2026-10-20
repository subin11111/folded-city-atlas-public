#!/usr/bin/env python3
"""Publish byte-identical real-data experiment figures and traceable summaries."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from optimize_session import sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]


def collect_summary(run, evaluation):
    summary = dict(created_utc=utc_now(), accepted=False, navigation=False,
        default_rviz_replaced=False, numerical_runs={}, revisit_runs={},
        evidence_classification='Repeated development and correlated heldout observations; no surveyed ground truth')
    sources = {}
    for p in sorted((run/'convergence').glob('*/report.json')):
        d = json.loads(p.read_text()); v = d['variants']['baseline_huber']
        sources[str(p.resolve())] = sha256_file(p)
        summary['numerical_runs'][p.parent.name] = dict(
            strict_converged=v['strict_converged'], all_inner_success=all(i['success'] for i in v['solver_iterations']),
            iterations=len(v['solver_iterations']), nfev=sum(i['nfev'] for i in v['solver_iterations']),
            start_cost=v['initial_weighted_cost'], final_cost=v['final_weighted_cost'],
            last_step=v['solver_iterations'][-1]['maximum_parameter_step'],
            input_hashes_unchanged=d['input_hashes_unchanged'],
            graph_byte_identical=d['graph_manifest_byte_identical'])
    for label in ('revisits', 'revisits_dense', 'revisits_ba_continued', 'revisits_ba_exact'):
        p = run/label/'report.json'
        d = json.loads(p.read_text())
        if not d['complete'] or not d['inputs_unchanged']: raise ValueError('Incomplete revisit run')
        sources[str(p.resolve())] = sha256_file(p)
        summary['revisit_runs'][label] = [dict(a=r['a'], b=r['b'],
            tracks=r['bundle']['cross_visit_tracks'], ba_executed=r['bundle']['ba_executed'],
            accepted=r['accepted'], reason=r['bundle'].get('reason'),
            reasons=r['bundle'].get('reasons'), candidates=r['bundle'].get('candidates', []),
            chosen=r['bundle'].get('chosen')) for r in d['records']]
    p = evaluation/'report.json'; d = json.loads(p.read_text())
    sources[str(p.resolve())] = sha256_file(p)
    summary['common_support_all_stages'] = d['common_support_all_stages']
    summary['whole_frame_validation'] = d['whole_frame_validation']
    summary['local_bundle_training_diagnostics'] = d.get('local_bundle_training_diagnostics', [])
    summary['integration_runs'] = {}
    for label in ('integration_nominal', 'integration_weak'):
        p = a_path = run/label/'report.json'
        data = json.loads(p.read_text()); variant = data['variants']['baseline_huber']
        sources[str(a_path.resolve())] = sha256_file(a_path)
        summary['integration_runs'][label] = dict(
            strict_converged=variant['strict_converged'],
            all_inner_success=all(i['success'] for i in variant['solver_iterations']),
            last_step=variant['solver_iterations'][-1]['maximum_parameter_step'],
            component_count=variant['component_count'],
            active_bounds=variant['solver_iterations'][-1]['active_bounds'],
            new_edges=[e for e in variant['edges'] if e['id'].startswith('revisit_rgbd_ba:')])
    summary['loop_weight_audit'] = []
    p = run/'convergence/frame_baseline_lsmr10000/manifest.json'
    sources[str(p.resolve())] = sha256_file(p)
    for edge in json.loads(p.read_text())['edges']:
        if edge['id'] not in ('floor2:native_loop:548:981', 'floor2:native_loop:656:1055'): continue
        sigma = np.sqrt(np.diag(edge['covariance']))
        summary['loop_weight_audit'].append(dict(id=edge['id'], role=edge['role'],
            translation_sigma_m=sigma[:3].tolist(), rotation_sigma_degrees=np.degrees(sigma[3:]).tolist(),
            original_source=edge['provenance']['frame_edge_covariance']['source'],
            interpretation='Existing native information retained, not newly calibrated; heldout does not enter fitting'))
    summary['input_hashes'] = sources
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--evaluation', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    summary = collect_summary(a.run, a.evaluation)
    images = {
        'numerical_progress.png': a.run/'convergence/numerical_progress.png',
        'ba_numerical_progress.png': a.run/'bundle_dense_exact/numerical_progress.png',
        'comparison.png': a.evaluation/'figures/common_support_all_stages.png',
        'wall_656_1055.png': a.evaluation/'figures/floor2_656_1055_wall_0_1.png',
        'wall_548_981.png': a.evaluation/'figures/floor2_548_981_wall_0_0.png',
        'upper_corridor.png': a.evaluation/'figures/upper_corridor_time_windows_xy.png',
        'withheld_windows.png': a.evaluation/'figures/whole_frame_validation.png',
        'withheld_rgb_656_1055.png': a.evaluation/'figures/withheld_region_656_1055_original_rgb.png',
        'training_rgb_548_981.png': a.run/'revisits_dense/548_981/source_training_frames.png',
        'training_rgb_656_1055.png': a.run/'revisits_dense/656_1055/source_training_frames.png',
        'feature_matches_656_1055.png': a.run/'revisits_ba_continued/656_1055/feature_inliers.png',
        'surface_hypothesis_548_981.png': a.run/'revisits_dense/548_981/withheld_surface_before_after.png',
        'surface_hypothesis_656_1055.png': a.run/'revisits_dense/656_1055/withheld_surface_before_after.png',
    }
    for source in sorted((a.evaluation/'figures').glob('local_ba_*.png')):
        images[source.name] = source
    for source in images.values():
        with Image.open(source) as im: im.verify()
    a.output.mkdir(parents=True)
    records = []
    for name, source in images.items():
        target = a.output/name
        shutil.copyfile(source, target)
        if sha256_file(source) != sha256_file(target): raise RuntimeError('Copy mismatch')
        with Image.open(target) as im: size = list(im.size)
        records.append(dict(file=name, source=str(source.resolve()), sha256=sha256_file(target),
            size_pixels=size, bytes=target.stat().st_size, byte_identical=True,
            synthetic=False, limitations='Diagnostic evidence; not an approved map or surveyed accuracy'))
    (a.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (a.output/'manifest.json').write_text(json.dumps(dict(created_utc=utc_now(),
        source_sha256=sha256_file(Path(__file__)), algorithm='byte-identical copy, no retouching',
        accepted=False, images=records, summary_sha256=sha256_file(a.output/'summary.json')), indent=2)+'\n')
    print(json.dumps(dict(output=str(a.output), images=len(records), accepted=False)))


if __name__ == '__main__': main()
