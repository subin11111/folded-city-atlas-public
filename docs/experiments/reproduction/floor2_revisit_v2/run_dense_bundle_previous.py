#!/usr/bin/env python3
"""Frozen-observation numerical ablation: vectorized residual + dense exact TRF."""
from pathlib import Path
import json
import shutil
import sys
import time

import numpy as np
import scipy
from scipy.optimize import least_squares
from scipy.optimize._numdiff import approx_derivative

ROOT=Path(__file__).resolve().parents[2]
RUN=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src'))
from check_session_overlap import Session
from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, sha256_file, utc_now
from recover_geometry_connections import intrinsics_at_rgb
from remeasure_floor2_revisits import SETTINGS, make_bundle_problem as reference, rigid_observability
from rgbd_bundle_vectorized import make_bundle_problem


def huber_cost(residual):
    return float(.5*np.sum(np.where(np.abs(residual)<=1,residual**2,2*np.abs(residual)-1)))


def main():
    output=RUN/'bundle_dense_exact_previous'
    if output.exists(): raise FileExistsError(output)
    source=RUN/'revisits_ba_continued/656_1055/bundle.json'
    observations_path=RUN/'revisits_ba_continued/656_1055/frozen_observations.json'
    poses_path=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv'
    database=ROOT/'data/raw/rtabmap/261017-013501-2f.db'
    sources=[Path(__file__),ROOT/'src/rgbd_bundle_vectorized.py',ROOT/'src/remeasure_floor2_revisits.py',
        ROOT/'src/floor2_vectorized_solver.py',ROOT/'src/optimize_session.py']
    paths=sources+[source,observations_path,poses_path,database]
    hashes={str(p):sha256_file(p) for p in paths}
    output.mkdir()
    for path in sources: shutil.copyfile(path,output/('source_'+path.name))
    bundle=json.loads(source.read_text())
    old=next(c for c in bundle['candidates'] if c['name']=='previous_preview')
    initial={int(n):np.array(p) for n,p in old['poses'].items()}
    nodes=list(range(652,661))+list(range(1051,1060))
    gauge=656
    session=Session(database,'floor2')
    calibrations={n:intrinsics_at_rgb(session,n) for n in nodes}
    session.connection.close()
    frozen=json.loads(observations_path.read_text())
    observations=[(r['node'],r['landmark'],np.array(r['rgb_pixel']),
                   np.nan if r['optical_depth_m'] is None else r['optical_depth_m']) for r in frozen]
    selected=load_pose_csv(poses_path)
    priors=[(r['a'],r['b'],inverse(selected[r['a']])@selected[r['b']],r['sigma_m'],
             np.radians(r['sigma_deg'])) for r in old['priors']]
    args=(nodes,gauge,initial,observations,np.array(old['landmarks']),priors,calibrations)
    x0,original,pattern,unpack_original=reference(*args)
    xb,residual,sparsity,unpack=make_bundle_problem(*args)
    if not np.array_equal(x0,xb) or (pattern!=sparsity).nnz:
        raise ValueError('Parameter vector or sparsity changed')
    max_error=0.
    for x in (x0,x0+np.random.default_rng(2026).normal(size=x0.size)*.005):
        max_error=max(max_error,float(np.max(np.abs(original(x)-residual(x)))))
    if max_error>2e-10: raise ValueError(f'Residual differs: {max_error}')
    start=time.monotonic()
    for _ in range(40): original(x0)
    scalar_seconds=(time.monotonic()-start)/40
    start=time.monotonic()
    for _ in range(400): residual(x0)
    batch_seconds=(time.monotonic()-start)/400
    def jacobian(x):
        return approx_derivative(residual,x,method='3-point',sparsity=sparsity).toarray()
    j1=jacobian(x0)
    j0=approx_derivative(original,x0,method='3-point')
    jac_error=float(np.max(np.abs(j1-j0)))
    if jac_error>2e-7: raise ValueError(f'Jacobian differs: {jac_error}')
    diagnostic=dict(created_utc=utc_now(),input_hashes=hashes,source_candidate=old['name'],
        original_cost=old['cost'],recomputed_cost=huber_cost(residual(x0)),
        observations=len(observations),depth_observations=sum(np.isfinite(o[3]) for o in observations),
        parameters=len(x0),maximum_residual_difference=max_error,
        maximum_central_jacobian_difference=jac_error,
        scalar_residual_seconds=scalar_seconds,vectorized_residual_seconds=batch_seconds,
        speedup=scalar_seconds/batch_seconds,scipy_version=scipy.__version__,
        solver='TRF exact dense SVD; colored 3-point finite difference converted to dense',
        settings=dict(loss='huber',f_scale=1.,max_nfev=300,ftol=1e-7,xtol=1e-7,gtol=1e-7,x_scale='jac'),
        changed='Numerical solver and equivalent batched residual only; no observation, prior, gauge, loss or gate changes')
    (output/'equivalence.json').write_text(json.dumps(diagnostic,indent=2,default=int)+'\n')
    print(json.dumps(diagnostic,default=int),flush=True)
    start=time.monotonic()
    fit=least_squares(residual,x0,jac=jacobian,method='trf',tr_solver='exact',loss='huber',f_scale=1.,
        x_scale='jac',max_nfev=300,ftol=1e-7,xtol=1e-7,gtol=1e-7,verbose=1)
    elapsed=time.monotonic()-start
    poses,points=unpack(fit.x)
    raw=residual(fit.x)[:3*len(observations)].reshape(-1,3)
    reprojection=np.linalg.norm(raw[:,:2],axis=1)*SETTINGS['reprojection_sigma_px']
    valid=np.array([np.isfinite(o[3]) for o in observations])
    depth=np.abs(raw[valid,2])*SETTINGS['depth_sigma_m']
    positive=all((inverse(poses[n]@calibrations[n][1])@np.r_[points[j],1.])[2]>.1 for n,j,*_ in observations)
    raw_singular=np.linalg.svd(jacobian(fit.x),compute_uv=False)
    robust_singular=np.linalg.svd(fit.jac,compute_uv=False)
    candidate=dict(old,success=bool(fit.success),message=fit.message,nfev=fit.nfev,
        previous_nfev=old['nfev'],previous_cost=old['cost'],cost=float(fit.cost),optimality=float(fit.optimality),
        reprojection_median_px=float(np.median(reprojection)),reprojection_p90_px=float(np.percentile(reprojection,90)),
        depth_median_abs_m=float(np.median(depth)),depth_p90_abs_m=float(np.percentile(depth,90)),
        positive_depth=positive,fixed_landmark_rigid_observability=rigid_observability(points),
        T_a_b=(inverse(poses[gauge])@poses[1055]).tolist(),
        poses={str(n):p.tolist() for n,p in poses.items()},landmarks=points.tolist(),
        continuation_settings=diagnostic['settings'],numerical_backend=diagnostic['solver'])
    report=dict(complete=True,accepted=False,candidate=candidate,elapsed_seconds=elapsed,
        equivalence=diagnostic,whole_frame_evaluation_pending=True,
        input_hashes_unchanged=all(sha256_file(Path(p))==h for p,h in hashes.items()),
        raw_jacobian_singular_values=raw_singular.tolist(),robust_jacobian_singular_values=robust_singular.tolist(),
        rank_note='Full BA Jacobians INCLUDE all local priors; not data-only observability or calibrated marginal uncertainty')
    if not report['input_hashes_unchanged']: raise RuntimeError('Input changed during solve')
    (output/'report.json').write_text(json.dumps(report,indent=2,default=int)+'\n')
    print(json.dumps(dict(output=str(output),success=bool(fit.success),nfev=fit.nfev,cost=fit.cost,
        optimality=fit.optimality,seconds=elapsed)),flush=True)


if __name__=='__main__':main()
