#!/usr/bin/env python3
"""Diagnose double walls and conditionally remeasure revisits with surface ICP.

Whole frames at +/-7 and +/-10 nodes are excluded from fitting. These windows
share a sensor and local pose chain, so they are not independent ground truth.
No point removal, common-floor constraint, scale change or DB write is used.
Geometry alone cannot establish place identity: exports remain hypotheses.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sqlite3
import sys
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from evaluate_map_surfaces import Settings, point_normals
from export_rtabmap_db import load_pose_csv
from optimize_session import apply, checked_transform, geometric_score, inverse, sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]
SPECS = {
    "outside": ("261016-234854-outside.db", "student_union_20261017_reviewed", "surface_evaluation_old"),
    "lobby": ("261017-000622-1f.db", "student_union_20261017_reviewed", "surface_evaluation_old"),
    "floor2": ("261017-013501-2f.db", "student_union_20261017_added_reviewed", "surface_evaluation"),
}
from optimize_session import Clouds

# Explicit observations from inspecting the saved source_frames.png montages.
# This establishes place identity, not metric correspondence or surveyed truth.
VISUAL_REVIEW = {
    ("outside",69,323): "Same brick column directly before glass entrance, stair edge/tactile paving and low rectangular concrete block beside window.",
    ("outside",113,351): "Same two brick columns facing plaza, poster on near column, branching tactile paving, low storefront and red fixture in background.",
    ("lobby",595,650): "Same KONKUK mascot, curved black display bench, framed-photo wall and neighboring glass storefront.",
    ("lobby",415,504): "Same open wooden shelving unit, tilted cardboard panel, boxes and bright drink vending machine at glass lobby exit.",
    ("lobby",445,495): "Same stair landing with mint-green lower wall, elevator marked 1, framed bulletin board and right-side doorway/handrail.",
    ("floor2",548,981): "Same dark locked grille under exit lamp, right-side railing/end wall; neighboring classical-music listening-room frontage is visible.",
    ("floor2",644,1035): "Same patterned right-side storefront, black notices, neighboring glass shopfront, flower wreaths and far glass corridor end.",
}


def reduce_voxels(points, voxel=.06):
    if not len(points):
        return points
    _, idx = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(idx)]


def cloud_window(clouds, poses, center, offsets):
    nodes = [center + offset for offset in offsets if center + offset in poses]
    points = [apply(np.vstack(clouds.get(node)), inverse(poses[center]) @ poses[node]) for node in nodes]
    return reduce_voxels(np.vstack(points)), nodes


def normals(points):
    n, good = point_normals(points, Settings(normal_neighbors=16, normal_radius_m=.32,
                                          normal_planarity_max=.06))
    return n, good


def correspondences(target, source, transform, target_normals, source_normals,
                    target_good, source_good, gate=.35):
    """Symmetric nearest neighbors; normal agreement excludes depth edges."""
    moved = apply(source, transform)
    rotated = source_normals @ transform[:3, :3].T
    ds, js = cKDTree(target).query(moved, workers=1)
    dt, jt = cKDTree(moved).query(target, workers=1)
    cos = np.cos(np.radians(30.))
    sv = (source_good & target_good[js] & (ds < gate)
          & (np.abs(np.einsum("ij,ij->i", rotated, target_normals[js])) > cos))
    tv = (target_good & source_good[jt] & (dt < gate)
          & (np.abs(np.einsum("ij,ij->i", target_normals, rotated[jt])) > cos))
    moving = np.vstack([moved[sv], moved[jt[tv]]])
    fixed = np.vstack([target[js[sv]], target[tv]])
    normal = np.vstack([target_normals[js[sv]], target_normals[tv]])
    return moving, fixed, normal


def observability(moving, normal):
    if len(moving) < 12:
        return {"rank": 0, "eigenvalues": [], "minimum_eigenvalue_ratio": 0.}
    center = moving.mean(axis=0)
    radius = max(float(np.sqrt(np.mean(np.sum((moving-center)**2, axis=1)))), .1)
    jac = np.column_stack([normal, np.cross((moving-center)/radius, normal)])
    eig = np.linalg.eigvalsh(jac.T @ jac / len(jac))
    return {"rank": int(np.sum(eig > eig[-1] * 1e-4)), "eigenvalues": eig.tolist(),
            "minimum_eigenvalue_ratio": float(eig[0]/max(eig[-1], 1e-12)),
            "rotation_length_normalization_m": radius,
            "note": "Data-only point-to-plane normal matrix; no prior used to disguise nullspaces."}


def refine(target, source, initial, iterations=35):
    """Cauchy IRLS point-to-plane Gauss-Newton, bounded increments in SE(3)."""
    tn, tg = normals(target)
    sn, sg = normals(source)
    transform = initial.copy()
    history = []
    for iteration in range(iterations):
        p, q, n = correspondences(target, source, transform, tn, sn, tg, sg)
        if len(p) < 100:
            break
        residual = np.einsum("ij,ij->i", p-q, n)
        center = p.mean(axis=0)
        jacobian = np.column_stack([n, np.cross(p-center, n)])
        weight = 1. / (1. + (residual/.03)**2)
        jac = jacobian * np.sqrt(weight[:, None])
        rhs = residual * np.sqrt(weight)
        # Tiny numerical damping only. Real data rank is separately evaluated.
        delta = np.linalg.solve(jac.T @ jac + np.eye(6)*1e-6, -jac.T @ rhs)
        delta[:3] *= min(1., .08/max(np.linalg.norm(delta[:3]), 1e-12))
        delta[3:] *= min(1., np.radians(1.)/max(np.linalg.norm(delta[3:]), 1e-12))
        increment = np.eye(4)
        increment[:3, :3] = Rotation.from_rotvec(delta[3:]).as_matrix()
        increment[:3, 3] = center + delta[:3] - increment[:3, :3] @ center
        proposed = increment @ transform
        correction = proposed @ inverse(initial)
        if (np.linalg.norm(correction[:3, 3]) > .75 or
                Rotation.from_matrix(correction[:3, :3]).magnitude() > np.radians(10.)):
            break
        transform = proposed
        history.append({"iteration": iteration, "matches": len(p),
                        "point_plane_median_abs_m": float(np.median(np.abs(residual))),
                        "cauchy_cost": float(np.mean(.03**2*np.log1p((residual/.03)**2))),
                        "step_translation_m": float(np.linalg.norm(delta[:3])),
                        "step_rotation_degrees": float(np.degrees(np.linalg.norm(delta[3:])))})
        if np.linalg.norm(delta[:3]) < 1e-5 and np.linalg.norm(delta[3:]) < 1e-5:
            break
    p, q, n = correspondences(target, source, transform, tn, sn, tg, sg)
    residual = np.einsum("ij,ij->i", p-q, n)
    data = observability(p, n)
    data.update({"history": history, "matches": len(p),
                 "final_point_plane_median_abs_m": float(np.median(np.abs(residual))) if len(p) else None})
    return transform, data


def geometric_diagnostic(target, source, transform, gravity_rotation):
    tn, tg = normals(target)
    sn, sg = normals(source)
    p, q, n = correspondences(target, source, transform, tn, sn, tg, sg, gate=.60)
    residual = np.einsum("ij,ij->i", p-q, n)
    world_normals = n @ gravity_rotation.T
    result = {"cloud": geometric_score(target, source, transform, .20),
              "normal_compatible_matches": len(p)}
    for kind, selected in [("wall", np.abs(world_normals[:, 2]) < np.sin(np.radians(15.))),
                           ("horizontal", np.abs(world_normals[:, 2]) > np.cos(np.radians(12.))),
                           ("all", np.ones(len(p), dtype=bool))]:
        rr = np.abs(residual[selected])
        result[kind] = {"count": len(rr), "median_abs_m": float(np.median(rr)) if len(rr) else None,
                        "p90_abs_m": float(np.percentile(rr,90)) if len(rr) else None}
    return result


def render_evidence(path, target, source, transforms, gravity_rotation, title):
    """All panels use identical withheld samples, projection and axes limits."""
    clouds = [target @ gravity_rotation.T] + [apply(source,t) @ gravity_rotation.T for t in transforms.values()]
    total = np.vstack(clouds)
    lo, hi = np.percentile(total, [1,99], axis=0)
    pad = np.maximum((hi-lo)*.05, .10)
    fig, axes = plt.subplots(2, len(transforms), figsize=(5.6*len(transforms),9.0), squeeze=False)
    for col, (label, transform) in enumerate(transforms.items()):
        moving = apply(source,transform) @ gravity_rotation.T
        for row, dims in enumerate([(0,1),(0,2)]):
            ax=axes[row,col]
            ax.scatter(clouds[0][::2,dims[0]], clouds[0][::2,dims[1]], s=.55, c="#0086b3",alpha=.65,label="visit A")
            ax.scatter(moving[::2,dims[0]], moving[::2,dims[1]], s=.55,c="#e25822",alpha=.65,label="visit B")
            ax.set(xlim=(lo[dims[0]]-pad[dims[0]],hi[dims[0]]+pad[dims[0]]),
                   ylim=(lo[dims[1]]-pad[dims[1]],hi[dims[1]]+pad[dims[1]]),
                   xlabel="X (m)",ylabel="Y (m)" if row==0 else "Z (m)",title=label)
            ax.set_aspect("equal");ax.grid(alpha=.2)
            if col==0:ax.legend(markerscale=6,loc="best")
    fig.suptitle(title+"\nWHOLE-FRAME HOLDOUTS; same samples and camera axes; conditional place identity",fontsize=13)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(path,dpi=160);plt.close(fig)


def render_rgb(connection, nodes, path):
    fig, axes=plt.subplots(2,3,figsize=(13,8))
    for ax,node in zip(axes.flat,nodes):
        row=connection.execute("SELECT image FROM Data WHERE id=?",(node,)).fetchone()
        rgb=cv2.imdecode(np.frombuffer(row[0],np.uint8),cv2.IMREAD_COLOR) if row and row[0] else None
        if rgb is not None:ax.imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB))
        ax.set_title(f"Frame {node}");ax.axis("off")
    fig.suptitle("Source images: revisit identity requires visual review",fontsize=14)
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def select_pairs(session, count):
    sub=SPECS[session][2]
    p=ROOT/"outputs/student_union_20261017_revalidated"/sub/session/"plane_pair_metrics.csv"
    rows=[r for r in csv.DictReader(p.open()) if r["trajectory"]=="app" and r["kind"]=="wall"]
    chosen=[]
    for r in sorted(rows,key=lambda r:-float(r["symmetric_plane_abs_median_m"])):
        a,b=int(r["from_id"]),int(r["to_id"])
        if any(abs(a-u)<20 and abs(b-v)<20 for u,v in chosen):continue
        chosen.append((a,b))
        if len(chosen)>=count:break
    return chosen


def run_session(session, output, count, baseline_path=None, training_single_frame=False):
    filename,prior_root,_=SPECS[session]
    database=ROOT/"data/raw/rtabmap"/filename
    original_hash=sha256_file(database)
    connection=sqlite3.connect(f"file:{database}?mode=ro",uri=True)
    clouds=Clouds(connection,None)
    app_path=ROOT/"outputs"/prior_root/"audit"/session/"app_optimized_trajectory.csv"
    baseline_path=baseline_path or app_path
    base=load_pose_csv(baseline_path)
    previous=load_pose_csv(ROOT/"outputs"/prior_root/session/"optimized_main_trajectory.csv")
    app=load_pose_csv(app_path)
    revalidation=json.loads((ROOT/"outputs/student_union_20261017_revalidated/intra_session"/session/"constraints.json").read_text())
    records={(r['a'],r['b']):r for r in revalidation['records']}
    directory=output/session;directory.mkdir(parents=True,exist_ok=True)
    rows=[]
    for a,b in select_pairs(session,count):
        pairdir=directory/f"{a}_{b}";pairdir.mkdir()
        print(f"{session}: surface revisit {a} -> {b}",flush=True)
        row=connection.execute("SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type IN (1,2)",(a,b)).fetchone()
        stored=checked_transform(row[0])
        train_offsets=[0] if training_single_frame else [-2,0,2]
        target,fit_a=cloud_window(clouds,base,a,train_offsets)
        source,fit_b=cloud_window(clouds,base,b,train_offsets)
        test_target,test_a=cloud_window(clouds,base,a,[-10,-7,7,10])
        test_source,test_b=cloud_window(clouds,base,b,[-10,-7,7,10])
        if set(fit_a+fit_b)&set(test_a+test_b):
            raise ValueError("Whole-frame holdout overlaps training")
        baseline=inverse(base[a])@base[b]
        candidates={}
        for seed_name,seed in [("stored_loop",stored),("baseline_pose",baseline)]:
            candidate,fit=refine(target,source,seed)
            candidates[seed_name]={"transform":candidate,"fit":fit,"train_geometry":geometric_score(target,source,candidate,.2)}
        # Select using training data only; unused-frame metrics never tune this selection.
        label=min(candidates,key=lambda k:candidates[k]["train_geometry"]["clipped_rmse_m"])
        candidate=candidates[label]["transform"]
        train=candidates[label]["fit"]
        gravity=base[a][:3,:3]
        transforms={"baseline":baseline,"stored_loop":stored,"surface_candidate":candidate}
        test={name:geometric_diagnostic(test_target,test_source,t,gravity) for name,t in transforms.items()}
        before=test['baseline']['cloud'];after=test['surface_candidate']['cloud']
        reasons=[]
        if train['rank']<6:reasons.append('surface_degeneracy_rank_below_6')
        if after['overlap']<.45:reasons.append('heldout_window_overlap_below_45pct')
        if after['clipped_rmse_m']>before['clipped_rmse_m']-.005:reasons.append('heldout_window_rmse_improvement_below_5mm')
        if after['rmse_m'] is None or after['rmse_m']>.10:reasons.append('heldout_window_inlier_rmse_above_10cm')
        historical=records.get((a,b),{})
        geometry_passed=not reasons
        if not historical.get('accepted',False):reasons.append('place_identity_not_independently_verified')
        if historical.get('held_out',False):reasons.append('reserved_existing_graph_holdout')
        correction=candidate@inverse(baseline)
        # Model covariance, not inverse-Hessian statistical precision.
        info=np.diag([1/.10**2]*3+[1/np.radians(3.)**2]*3)
        result={"session":session,"a":a,"b":b,"T_a_b":candidate.tolist(),
                "session_a":session,"session_b":session,"node_a":a,"node_b":b,
                "geometry_candidate_passed":geometry_passed,"accepted":not reasons,
                "eligible_for_joint":not reasons,"rejection_reasons":reasons,
                "historical_revalidation_reason":historical.get('reason'),
                "held_out":bool(historical.get('held_out',False)),"selected_training_seed":label,
                "training_nodes_a":fit_a,"training_nodes_b":fit_b,"whole_frame_holdout_a":test_a,"whole_frame_holdout_b":test_b,
                "training_single_frame_ablation":training_single_frame,
                "training_points":[len(target),len(source)],"holdout_points":[len(test_target),len(test_source)],
                "same_sensor_temporally_correlated_holdout":True,"observability_and_solver":train,
                "pose_relative_deltas":{"translation_m":float(np.linalg.norm(correction[:3,3])),
                                        "rotation_degrees":float(np.degrees(Rotation.from_matrix(correction[:3,:3]).magnitude()))},
                "baseline_T_a_b":baseline.tolist(),"stored_T_a_b":stored.tolist(),
                "app_T_a_b":(inverse(app[a])@app[b]).tolist(),
                "previous_offline_T_a_b":(inverse(previous[a])@previous[b]).tolist(),
                "whole_frame_holdout_evaluation":test,"information_6x6":info.tolist(),
                "covariance_note":"Conservative isotropic modelling choice: 10cm/3deg; not statistically calibrated.",
                "candidate_seeds":{k:{**v,"transform":v['transform'].tolist()} for k,v in candidates.items()}}
        render_evidence(pairdir/"holdout_before_after.png",test_target,test_source,transforms,gravity,f"{session}: revisit {a} / {b}")
        render_rgb(connection,[a-2,a,a+2,b-2,b,b+2],pairdir/"source_frames.png")
        np.savez_compressed(pairdir/"holdout_geometry.npz",target=test_target,source=test_source,gravity_rotation=gravity,
                            baseline=baseline,stored_loop=stored,surface_candidate=candidate)
        (pairdir/"report.json").write_text(json.dumps(result,indent=2)+"\n")
        rows.append(result)
        print(json.dumps({"session":session,"pair":[a,b],"geometry_pass":geometry_passed,
                          "baseline_rmse":before['clipped_rmse_m'],"candidate_rmse":after['clipped_rmse_m'],
                          "data_rank":train['rank'],"reasons":reasons}),flush=True)
    connection.close()
    after_hash=sha256_file(database)
    if original_hash!=after_hash:raise RuntimeError("Source database changed")
    report={"session":session,"baseline_trajectory":str(baseline_path),"baseline_sha256":sha256_file(baseline_path),
            "database_sha256_before":original_hash,"database_sha256_after":after_hash,"original_database_unchanged":True,
            "records":rows}
    (directory/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


def audit_existing(input_root, output):
    """Review the same fitted candidates on nearby excluded frames, no refitting.

    Distant windows have poor field-of-view overlap. Nearby +/-1 frames provide
    a separate local check, explicitly correlated and selected after distant
    audit failure. They cannot support an independent global success claim.
    """
    if output.exists():
        raise ValueError("Near-frame audit output already exists")
    output.mkdir(parents=True)
    constraints=[]
    for session in SPECS:
        session_path=input_root/session/'report.json'
        if not session_path.exists():continue
        previous=json.loads(session_path.read_text())
        base=load_pose_csv(Path(previous['baseline_trajectory']))
        database=ROOT/'data/raw/rtabmap'/SPECS[session][0]
        connection=sqlite3.connect(f'file:{database}?mode=ro',uri=True)
        clouds=Clouds(connection,None)
        for record in previous['records']:
            a,b=record['a'],record['b']
            directory=output/session/f'{a}_{b}';directory.mkdir(parents=True)
            target,ta=cloud_window(clouds,base,a,[-1,1])
            source,tb=cloud_window(clouds,base,b,[-1,1])
            if set(ta+tb)&set(record['training_nodes_a']+record['training_nodes_b']):
                raise ValueError('Nearby frame audit overlaps fitted frames')
            transforms={'baseline':np.asarray(record['baseline_T_a_b']),
                        'stored_loop':np.asarray(record['stored_T_a_b']),
                        'surface_candidate':np.asarray(record['T_a_b'])}
            metrics={k:geometric_diagnostic(target,source,t,base[a][:3,:3]) for k,t in transforms.items()}
            before=metrics['baseline']['cloud'];after=metrics['surface_candidate']['cloud']
            reasons=[]
            if record['observability_and_solver']['rank']<6:reasons.append('surface_degenerate')
            if after['overlap']<.45:reasons.append('near_frame_overlap_below_45pct')
            if after['clipped_rmse_m']>before['clipped_rmse_m']-.005:reasons.append('near_frame_improvement_below_5mm')
            if after['rmse_m'] is None or after['rmse_m']>.10:reasons.append('near_frame_inlier_rmse_above_10cm')
            locally_passed=not reasons
            visual=VISUAL_REVIEW.get((session,a,b))
            if not visual:reasons.append('visual_identity_not_reviewed')
            if record['held_out']:reasons.append('reserved_existing_graph_holdout')
            result={**record,'accepted':not reasons,'eligible_for_joint':False,
                    'eligible_for_experimental_graph':not reasons,
                    'local_geometry_verified':bool(locally_passed and visual),
                    'reason':';'.join(reasons) if reasons else 'visual_identity_and_correlated_near_frame_geometry_passed',
                    'rejection_reasons':reasons,'geometry_candidate_passed':locally_passed,
                    'visual_identity_review':{'confirmed':bool(visual),'observed':visual,
                        'source_image':str((input_root/session/f'{a}_{b}'/'source_frames.png').resolve()),
                        'method':'Assistant inspected rendered original RGB images; no manual metric clicks.'},
                    'near_frame_holdout_a':ta,'near_frame_holdout_b':tb,'near_frame_holdout_evaluation':metrics,
                    'distant_holdout_result_retained':True,
                    'validation_scope':'Post-hoc local audit; whole frames unused in fitting but temporally correlated. Not independent global validation.',
                    'original_report':str((input_root/session/f'{a}_{b}'/'report.json').resolve())}
            render_evidence(directory/'near_frame_before_after.png',target,source,transforms,base[a][:3,:3],
                            f'{session}: revisit {a}/{b}; NEARBY CORRELATED AUDIT')
            (directory/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            constraints.append(result)
            print(json.dumps({'pair':[session,a,b],'experimental_edge':not reasons,'rank':record['observability_and_solver']['rank'],
                              'before':before,'after':after,'reasons':reasons}),flush=True)
        connection.close()
    manifest={'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),
              'input':str(input_root.resolve()),'constraints':constraints,
              'accepted_means':'Eligible only for an explicit experimental pose-graph trial, not accepted final map.',
              'post_hoc_audit':True,'independent_validation':False,'navigation_ready':False}
    (output/'constraints.json').write_text(json.dumps(manifest,indent=2)+'\n')


def summarize(root):
    files=[root/'near_holdouts/constraints.json',root/'outside_near_holdouts/constraints.json',
           root/'single_frame_near_holdouts/constraints.json']
    rows=[]
    for path in files:
        if not path.exists():continue
        for r in json.loads(path.read_text())['constraints']:
            rows.append({'session':r['session'],'a':r['a'],'b':r['b'],
                'single_frame_ablation':r.get('training_single_frame_ablation',False),
                'accepted_experimental_constraint':r['accepted'],'held_out':r['held_out'],
                'reason':r['reason'],'source_report':str(path.resolve()),
                'near_frame_evaluation':r['near_frame_holdout_evaluation'],
                'distant_frame_evaluation':r['whole_frame_holdout_evaluation'],
                'visual_review':r['visual_identity_review'],'data_rank':r['observability_and_solver']['rank']})
    multi=[r for r in rows if not r['single_frame_ablation']]
    fig,axes=plt.subplots(2,1,figsize=(14,9),sharex=True)
    x=np.arange(len(multi));width=.25
    for i,(name,color) in enumerate([('baseline','#b04f50'),('stored_loop','#c99a2b'),('surface_candidate','#2589a8')]):
        axes[0].bar(x+(i-1)*width,[100*r['near_frame_evaluation'][name]['cloud']['clipped_rmse_m'] for r in multi],
                    width,label=name,color=color)
        axes[1].bar(x+(i-1)*width,[100*r['near_frame_evaluation'][name]['cloud']['overlap'] for r in multi],
                    width,label=name,color=color)
    labels=[f"{r['session']}\n{r['a']}/{r['b']}\n"+('EXPERIMENT' if r['accepted_experimental_constraint'] else
              'RESERVED' if r['held_out'] else 'REJECTED') for r in multi]
    axes[0].set(ylabel='20cm-clipped symmetric cloud RMSE (cm)',ylim=(0,21))
    axes[1].set(ylabel='Symmetric overlap within 20cm (%)',ylim=(0,100),xticks=x,xticklabels=labels)
    for ax in axes:ax.grid(axis='y',alpha=.2);ax.legend(ncol=3)
    fig.suptitle('Surface remeasurement: same unused nearby frames at each stage\n'
                 'Post-hoc, temporally correlated local audit; no independent global validation',fontsize=14)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(root/'surface_refinement_progression.png',dpi=170);plt.close(fig)
    result={'created_utc':utc_now(),'rows':rows,'records_evaluated':len(rows),
            'accepted_experimental_constraints':sum(r['accepted_experimental_constraint'] for r in rows),
            'heldout_pairs_promoted':sum(r['accepted_experimental_constraint'] and r['held_out'] for r in rows),
            'independent_global_validation_passed':False,'navigation_ready':False,
            'diagnosis':'Large wall duplication contains relative translation/yaw drift; small roll/pitch adjustment alone cannot remove it. '
               'Varying camera view makes distant frame pooling low overlap; nearby audit is explicitly correlated. '
               'Single-frame ablation is recorded separately and cannot be called independent validation.',
            'algorithms':['Calibrated RGB-D + confidence>=50 + depth0.25..4m','6cm voxel reduction',
                'Local PCA normals (k16, radius32cm, planarity<=0.06)',
                'Symmetric nearest-neighbor point-to-plane correspondences within35cm and normal angle30deg',
                'Cauchy IRLS sigma3cm, SE(3) Gauss-Newton, max35 iterations',
                'Per-step8cm/1deg and per-seed75cm/10deg guards',
                'Training-only stored-loop/baseline-pose multistart selection',
                'Data-only radius-normalized Hessian rank check',
                'Entire-frame exclusions and side-by-side equal-axis visualization',
                'Manual original RGB place-identity review with explicit observations'],
            'commands':[
                'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python src/refine_geometry_surfaces.py --pairs-per-session 4',
                'python src/refine_geometry_surfaces.py --audit-existing <surfaces> --output <surfaces>/near_holdouts',
                'python src/refine_geometry_surfaces.py --sessions outside --pairs-per-session 2 --output <surfaces>/outside_trial',
                'python src/refine_geometry_surfaces.py --audit-existing <surfaces>/outside_trial --output <surfaces>/outside_near_holdouts',
                'python src/refine_geometry_surfaces.py --sessions floor2 --training-single-frame --pairs-per-session 4 --output <surfaces>/single_frame_ablation',
                'python src/refine_geometry_surfaces.py --audit-existing <surfaces>/single_frame_ablation --output <surfaces>/single_frame_near_holdouts'],
            'source_sha256':sha256_file(Path(__file__))}
    (root/'summary.json').write_text(json.dumps(result,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/student_union_20261017_geometry_review/surfaces')
    parser.add_argument('--sessions',nargs='+',choices=SPECS,default=['lobby','floor2'])
    parser.add_argument('--pairs-per-session',type=int,default=4)
    parser.add_argument('--baseline',action='append',default=[],help='session:path.csv')
    parser.add_argument('--training-single-frame',action='store_true',help='Use each center frame only to test local pose pooling distortion')
    parser.add_argument('--audit-existing',type=Path,help='Audit existing fitted candidates on unused +/-1 frames without refitting')
    parser.add_argument('--summarize-root',type=Path,help='Write progression chart and compact report from existing experiment/audit files')
    args=parser.parse_args()
    if args.summarize_root:
        summarize(args.summarize_root)
        return
    if args.audit_existing:
        audit_existing(args.audit_existing,args.output)
        return
    if args.output.exists() and any(args.output.iterdir()):parser.error('Use a fresh evidence directory')
    args.output.mkdir(parents=True,exist_ok=True)
    baselines={s:Path(p).resolve() for s,p in (v.split(':',1) for v in args.baseline)}
    start=time.monotonic()
    reports=[run_session(s,args.output,args.pairs_per_session,baselines.get(s),args.training_single_frame) for s in args.sessions]
    report={'created_utc':utc_now(),'command':sys.argv,'source_sha256':sha256_file(Path(__file__)),
            'elapsed_seconds':time.monotonic()-start,
            'algorithms':['Confidence-filtered calibrated depth unprojection','6cm voxel downsampling',
              'PCA local surface normals and planarity filtering','Symmetric normal-compatible nearest-neighbor correspondences',
              'Cauchy IRLS robust point-to-plane SE(3) Gauss-Newton','Data-only scaled Hessian observability',
              'Training-only multi-start selection','Excluded whole-frame window geometry validation'],
            'settings':{'train_node_offsets':[0] if args.training_single_frame else [-2,0,2],'holdout_node_offsets':[-10,-7,7,10],
              'voxel_m':.06,'cauchy_sigma_m':.03,'nearest_neighbor_gate_m':.35,'maximum_seed_translation_m':.75,
              'maximum_seed_rotation_degrees':10.,'max_iterations':35,'geometry_holdout_overlap_min':.45},
            'thread_environment':{k:os.environ.get(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},
            'limitations':['No independently verified place identity is invented from surface coincidence.',
              'Holdout windows share sensor/local poses and are correlated; no absolute accuracy claim.',
              'No single global floor, smoothing, scale change or DB modification.',
              'Stair heights are not constrained; raw and canonical poses are unchanged.',
              'Surface candidates requiring visual/feature confirmation are not eligible graph edges.'],
            'sessions':reports}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
