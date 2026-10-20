import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from remeasure_floor2_revisits import make_bundle_problem as reference
from rgbd_bundle_vectorized import make_bundle_problem as batched


def problem_args(seed=817):
    rng=np.random.default_rng(seed)
    nodes=[7,14,20,19]
    poses={};calibrations={}
    for n in nodes:
        t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(rng.normal(size=3)*.2).as_matrix()
        t[:3,3]=rng.normal(size=3)*.1;poses[n]=t
        local=np.eye(4);local[:3,:3]=Rotation.from_rotvec(rng.normal(size=3)*.2).as_matrix()
        local[:3,3]=rng.normal(size=3)*.02
        calibrations[n]=(np.array([[510.,0,321.],[0,490.,240.],[0,0,1.]]),local)
    points=rng.normal(size=(12,3))*.3;points[:,2]+=2
    observations=[(n,j,rng.normal(size=2)*40+300,z) for n in nodes for j,z in
        enumerate([np.nan,np.inf,-np.inf,.01,.049,.05,.051,-1.,2.,2.1,3.,3.2])]
    priors=[(7,14,np.linalg.inv(poses[7])@poses[14],.08,.05),
            (20,19,np.linalg.inv(poses[20])@poses[19],.4,.1)]
    return nodes,7,poses,observations,points,priors,calibrations


@pytest.mark.parametrize('mode',['normal','behind','no_observations','no_priors'])
def test_exact_residual_unpack_sparsity_and_missing_depth_policy(mode):
    args=list(problem_args())
    if mode=='behind': args[4][:,2]=-1.
    if mode=='no_observations': args[3]=[]
    if mode=='no_priors': args[5]=[]
    x0,r0,s0,u0=reference(*args);x1,r1,s1,u1=batched(*args)
    np.testing.assert_array_equal(x0,x1)
    np.testing.assert_array_equal(s0.toarray(),s1.toarray())
    for perturb in (np.zeros_like(x0),np.random.default_rng(822).normal(size=len(x0))*.03):
        x=x0+perturb
        np.testing.assert_allclose(r0(x),r1(x),atol=2e-11,rtol=1e-13)
        p0,l0=u0(x);p1,l1=u1(x)
        assert list(p0)==list(p1)
        for node in p0: np.testing.assert_allclose(p0[node],p1[node],atol=2e-15,rtol=0)
        np.testing.assert_array_equal(l0,l1)
        if args[3]:
            depths=np.array([o[3] for o in args[3]])
            assert np.all(r1(x)[:len(args[3])*3].reshape(-1,3)[~np.isfinite(depths),2]==0)


def test_small_dense_and_sparse_solves_share_the_same_objective():
    nodes=[1,2,3];initial={n:np.eye(4) for n in nodes}
    for n in nodes: initial[n][0,3]=.2*(n-1)
    k=np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
    calibration={n:(k,np.eye(4)) for n in nodes}
    points=np.array([[x,y,z] for x in (-.3,.1,.4) for y in (-.2,.2) for z in (2.,2.5)])
    obs=[]
    for n in nodes:
        for j,p in enumerate(points):
            c=p-initial[n][:3,3];pixel=(k@c)[:2]/c[2]
            obs.append((n,j,pixel,c[2] if n!=3 else np.nan))
    initial[2][0,3]+=.1;initial[3][1,3]+=.03
    args=(nodes,1,initial,obs,points+[.02,.02,.04],[],calibration)
    x0,r0,s0,u0=reference(*args);x1,r1,s1,u1=batched(*args)
    old=least_squares(r0,x0,jac_sparsity=s0,x_scale='jac',max_nfev=100)
    new=least_squares(r1,x1,x_scale='jac',tr_solver='exact',max_nfev=100)
    assert old.success and new.success
    assert old.cost<1e-12 and new.cost<1e-12
    np.testing.assert_allclose(u0(old.x)[1],u1(new.x)[1],atol=1e-6)
