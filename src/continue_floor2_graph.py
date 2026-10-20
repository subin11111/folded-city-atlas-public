#!/usr/bin/env python3
"""Continue the identical floor2 frame graph from a graph-bound solved report.

Warm starts use world translation and left rotation corrections relative to
the ORIGINAL manifest initial poses. Gauge and correction bounds therefore do
not move. Only train factors enter the objective. The optional lsmr-10000
preset changes the linear solve budget (500 -> 10000), not the graph or loss.
The lsmr-10000-central preset additionally uses a three-point numerical
Jacobian to diagnose forward-difference roundoff near a stationary point.
Every outer step is persisted before the next step begins.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import sys
import time

import numpy as np
import scipy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from floor2_vectorized_solver import build_residual
from optimize_robust_submaps import (_as_transform, _decode_corrections,
    _edge_chi2, _robust_weights, connected_components, parse_manifest,
    select_gauges, transform_record)
from optimize_session import sha256_file, trajectory, utc_now

ROOT = Path(__file__).resolve().parents[1]


def encode_warm_start(nodes, gauges, poses, translation_bound, rotation_bound_degrees):
    """Inverse of original world-translation/left-rotation parameterization."""
    if set(poses) != {n.id for n in nodes}:
        raise ValueError('Warm poses and manifest node inventories differ')
    values = []
    for node in nodes:
        pose = _as_transform(poses[node.id], node.id)
        if node.id in gauges:
            if not np.allclose(pose, node.initial, atol=1e-12, rtol=0):
                raise ValueError('Warm start moves a fixed gauge')
            continue
        rotation = Rotation.from_matrix(pose[:3,:3] @ node.initial[:3,:3].T).as_rotvec()
        values.append(np.r_[pose[:3,3]-node.initial[:3,3], rotation])
    x = np.asarray(values).reshape(-1)
    bounds = np.tile([translation_bound]*3+[math.radians(rotation_bound_degrees)]*3,len(values))
    if not np.all(np.isfinite(x)) or np.any(np.abs(x)>bounds+1e-12):
        raise ValueError('Warm start is outside ORIGINAL componentwise correction bounds')
    # Only remove roundoff at a previously active bound, never re-center it.
    return np.clip(x,-bounds,bounds)


def strict_convergence(solver, weight_change, step, outer):
    return bool(outer >= 2 and solver['success'] and weight_change < 1e-5 and step < 1e-7)


def fixed_solve(problem, x, parameters, max_evaluations, linear_maxiter,
                jacobian_scheme='2-point'):
    limits=np.tile([parameters['translation_correction_bound_m']]*3+
        [math.radians(parameters['rotation_correction_bound_deg'])]*3,len(problem.variable_ids))
    if not len(x) or not len(problem.source_indices):
        return x.copy(),dict(success=True,status=1,nfev=0,njev=0,cost=0.,
            optimality=0.,message='nothing to optimize',active_bounds=0)
    fit=least_squares(problem,x,jac=jacobian_scheme,jac_sparsity=problem.jac_sparsity,
        bounds=(-limits,limits),method='trf',tr_solver='lsmr',x_scale='jac',
        max_nfev=max_evaluations,ftol=1e-9,xtol=1e-9,gtol=1e-9,
        tr_options={'atol':1e-10,'btol':1e-10,'maxiter':linear_maxiter})
    return fit.x,dict(success=bool(fit.success),status=int(fit.status),nfev=int(fit.nfev),
        njev=int(fit.njev),cost=float(fit.cost),optimality=float(fit.optimality),
        message=str(fit.message),active_bounds=int(np.count_nonzero(fit.active_mask)))


def continue_graph(nodes,edges,warm_variant,*,outer_iterations=20,max_evaluations=1000,
                   linear_maxiter=500,jacobian_scheme='2-point',checkpoint=None):
    if jacobian_scheme not in ('2-point','3-point'):
        raise ValueError('Unsupported numerical Jacobian scheme')
    parameters=warm_variant['parameters']
    method=warm_variant['method']
    train=[e for e in edges if e.used_for_fit]
    components=connected_components([n.id for n in nodes],train)
    gauge_map=select_gauges(nodes,components)
    gauges=set(gauge_map.values())
    x=encode_warm_start(nodes,gauges,warm_variant['poses'],
        parameters['translation_correction_bound_m'],parameters['rotation_correction_bound_deg'])
    variables=[n.id for n in nodes if n.id not in gauges]
    poses=_decode_corrections(x,nodes,variables)
    kwargs={k:parameters[k] for k in ('huber_delta','dcs_phi','group_switch_prior')}
    weights,groups=_robust_weights(method,train,poses,**kwargs)
    initial_poses={k:v.copy() for k,v in poses.items()}
    initial_problem=build_residual(nodes,train,gauges,weights)
    initial_cost=float(.5*np.sum(initial_problem(x)**2))
    iterations=[]
    converged=False
    for outer in range(1,outer_iterations+1):
        started=time.monotonic()
        previous=x.copy()
        problem=build_residual(nodes,train,gauges,weights)
        before_cost=float(.5*np.sum(problem(x)**2))
        x,solver=fixed_solve(problem,x,parameters,max_evaluations,linear_maxiter,jacobian_scheme)
        poses=_decode_corrections(x,nodes,variables)
        next_weights,groups=_robust_weights(method,train,poses,**kwargs)
        weight_change=max((abs(next_weights[k]-weights[k]) for k in weights),default=0.)
        step=float(np.max(np.abs(x-previous))) if len(x) else 0.
        converged=strict_convergence(solver,weight_change,step,outer)
        row=dict(outer_iteration=outer,maximum_weight_change=float(weight_change),
            maximum_parameter_step=step,fixed_weight_cost_before=before_cost,
            duration_seconds=time.monotonic()-started,strict_converged=converged,**solver)
        iterations.append(row)
        weights=next_weights
        if checkpoint is not None:
            checkpoint(row,poses,x)
        if converged:
            break
    final_problem=build_residual(nodes,train,gauges,weights)
    edge_rows=[]
    for e in edges:
        before,_=_edge_chi2(e,initial_poses)
        error,chi2=_edge_chi2(e,poses)
        edge_rows.append(dict(id=e.id,from_id=e.source,to_id=e.target,type=e.edge_type,
            role=e.role,used_for_fit=e.used_for_fit,
            information_weight=weights.get(e.id),
            translation_error_before_m=float(np.linalg.norm(before[:3])),
            translation_error_m=float(np.linalg.norm(error[:3])),
            rotation_error_deg=float(np.degrees(np.linalg.norm(error[3:]))),
            chi2_before_robust_weight=chi2))
    result=dict(method=method,parameters={**parameters,
            'outer_iterations_limit':outer_iterations,
            'max_evaluations_per_outer_iteration':max_evaluations},
        numerical_settings=dict(ftol=1e-9,xtol=1e-9,gtol=1e-9,
            linear_atol=1e-10,linear_btol=1e-10,linear_maxiter=linear_maxiter,
            reference_linear_maxiter=500,jacobian_scheme=jacobian_scheme,
            scipy_version=scipy.__version__),
        strict_converged=converged,
        convergence_rule='inner solver success AND max post-fit weight change < 1e-5 AND max original-coordinate parameter change < 1e-7; at least 2 outer steps',
        final_inner_solver_success=iterations[-1]['success'],
        final_inner_termination=iterations[-1]['message'],
        initial_weighted_cost=initial_cost,final_weighted_cost=float(.5*np.sum(final_problem(x)**2)),
        solver_iterations=iterations,fit_edge_ids=[e.id for e in train],
        heldout_edge_ids=[e.id for e in edges if not e.used_for_fit],
        heldout_edge_count_in_objective=0,component_count=len(components),
        components=[dict(node_ids=c,gauge_id=gauge_map[c[0]]) for c in components],
        groups=groups,edges=edge_rows,poses={k:transform_record(v) for k,v in poses.items()})
    return poses,result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--quality',type=Path,default=ROOT/'outputs/student_union_20261017_submap_quality/floor2/frames.csv')
    parser.add_argument('--outer-iterations',type=int,default=20)
    parser.add_argument('--max-evaluations',type=int,default=1000)
    parser.add_argument('--solver-preset',choices=('reference','lsmr-10000','lsmr-10000-central'),default='reference')
    args=parser.parse_args(argv)
    if args.output.exists(): raise FileExistsError(args.output)
    if args.outer_iterations<2 or args.max_evaluations<1: raise ValueError('Invalid solve budget')
    manifest_path=args.source/'manifest.json'
    report_path=args.source/'report.json'
    manifest=json.loads(manifest_path.read_text())
    prior=json.loads(report_path.read_text())
    if prior.get('graph_manifest_sha256')!=sha256_file(manifest_path):
        raise ValueError('Warm report is not bound to this exact manifest')
    nodes,edges=parse_manifest(manifest)
    warm=prior['variants']['baseline_huber']
    if set(warm['fit_edge_ids'])!={e.id for e in edges if e.used_for_fit}:
        raise ValueError('Warm report training-factor inventory mismatch')
    frames={int(r['node_id']):r for r in csv.DictReader(args.quality.open())}
    # Quality source must match the original graph's recorded source exactly.
    expected=manifest['metadata']['input_hashes'].get(str(args.quality.resolve()))
    if expected!=sha256_file(args.quality): raise ValueError('Quality source hash mismatch')
    paths=[manifest_path,report_path,args.quality,Path(__file__),
        ROOT/'src/floor2_vectorized_solver.py',ROOT/'src/optimize_robust_submaps.py',
        ROOT/'src/optimize_session.py']
    hashes={str(p.resolve()):sha256_file(p) for p in paths}
    args.output.mkdir(parents=True)
    shutil.copyfile(manifest_path,args.output/'manifest.json')
    for p in paths[3:]: shutil.copyfile(p,args.output/('source_'+p.name))
    (args.output/'run.json').write_text(json.dumps(dict(created_utc=utc_now(),command=sys.argv,
        input_hashes=hashes,solver_preset=args.solver_preset,accepted=False),indent=2)+'\n')
    checkpoint_dir=args.output/'checkpoints';checkpoint_dir.mkdir()
    def checkpoint(row,poses,x):
        out=checkpoint_dir/f"outer_{row['outer_iteration']:03d}.json"
        out.write_text(json.dumps(dict(**row,graph_manifest_sha256=hashes[str(manifest_path.resolve())],
            poses={k:transform_record(v) for k,v in poses.items()},
            corrections_in_original_parameterization=x.tolist()),indent=2)+'\n')
        with (args.output/'iterations.jsonl').open('a') as stream:
            stream.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    print(json.dumps(dict(stage='warm_start',nodes=len(nodes),edges=len(edges),
        source=str(args.source),output=str(args.output),solver_preset=args.solver_preset)),flush=True)
    poses,result=continue_graph(nodes,edges,warm,outer_iterations=args.outer_iterations,
        max_evaluations=args.max_evaluations,
        linear_maxiter=500 if args.solver_preset=='reference' else 10000,
        jacobian_scheme='3-point' if args.solver_preset.endswith('-central') else '2-point',
        checkpoint=checkpoint)
    frame_ids={n['id']:int(n['frame_id']) for n in manifest['submaps']}
    integer_poses={frame_ids[k]:v for k,v in poses.items()}
    stamps={n:float(r['stamp']) for n,r in frames.items()}
    trajectory(args.output/'trajectory.csv',integer_poses,stamps)
    ab={n:v for n,v in integer_poses.items() if frames[n]['quality_grade'] in ('A','B')}
    trajectory(args.output/'trajectory_AB.csv',ab,stamps)
    unchanged=all(sha256_file(Path(p))==h for p,h in hashes.items())
    report=dict(schema_version=1,complete=True,created_utc=utc_now(),command=sys.argv,
        accepted=False,navigation=False,default_rviz_replaced=False,other_sessions_modified=False,
        source_report_sha256=hashes[str(report_path.resolve())],input_hashes=hashes,
        input_hashes_unchanged=unchanged,graph_manifest_sha256=sha256_file(args.output/'manifest.json'),
        graph_manifest_byte_identical=True,full_frame_count=len(integer_poses),export_AB_count=len(ab),
        trajectory_sha256=sha256_file(args.output/'trajectory.csv'),
        trajectory_AB_sha256=sha256_file(args.output/'trajectory_AB.csv'),
        variants={'baseline_huber':result})
    if not unchanged: raise RuntimeError('Input changed during continuation')
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(stage='done',strict_converged=result['strict_converged'],
        initial_cost=result['initial_weighted_cost'],final_cost=result['final_weighted_cost'],
        output=str(args.output))),flush=True)


if __name__=='__main__': main()
