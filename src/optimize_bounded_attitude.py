#!/usr/bin/env python3
"""Experimental SE(3) corrections with a soft *original-attitude* regularizer.

The regularizer is not a fresh IMU observation. Its sigma is a modelling choice.
No floor constraint is introduced. Genuine stair height changes remain free.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from optimize_session import block_huber


def optimize_attitude(raw, odometry, loops, components, gravity_sigma_degrees=2.,
                      max_evaluations=500, maximum_tilt_degrees=5.):
    if gravity_sigma_degrees <= 0:
        raise ValueError('Attitude regularizer sigma must be positive')
    ids=sorted(raw)
    index={n:k for k,n in enumerate(ids)}
    anchors={c[0] for c in components}
    variables=[n for n in ids if n not in anchors]
    vindex={n:k for k,n in enumerate(variables)}
    vi=np.array([index[n] for n in variables],dtype=int)
    active=odometry+[e for e in loops if not e.held_out]
    ri=np.array([raw[n][:3,:3] for n in ids])
    ti=np.array([raw[n][:3,3] for n in ids])
    ai=np.array([index[e.a] for e in active],dtype=int)
    bi=np.array([index[e.b] for e in active],dtype=int)
    mr=np.array([e.measurement[:3,:3].T for e in active])
    mt=np.array([e.measurement[:3,3] for e in active])
    whitening=np.array([np.linalg.cholesky(e.effective_information).T for e in active])
    original_up=ri.transpose(0,2,1)[:,:,2]
    sigma=np.radians(gravity_sigma_degrees)

    def decode(parameters):
        corr=np.zeros((len(ids),6))
        corr[vi]=parameters.reshape(-1,6)
        rotations=Rotation.from_rotvec(corr[:,3:]).as_matrix()@ri
        return rotations,ti+corr[:,:3]

    def residual(parameters):
        rotations,translations=decode(parameters)
        inverse_a=rotations[ai].transpose(0,2,1)
        pr=inverse_a@rotations[bi]
        pt=np.einsum('nij,nj->ni',inverse_a,translations[bi]-translations[ai])
        er=mr@pr
        et=np.einsum('nij,nj->ni',mr,pt-mt)
        errors=np.column_stack((et,Rotation.from_matrix(er).as_rotvec()))
        edges=block_huber(np.einsum('nij,nj->ni',whitening,errors),3.).ravel()
        current_up=rotations.transpose(0,2,1)[:,:,2]
        # Chordal up-vector difference is yaw-invariant, unlike roll/pitch Euler angles.
        gravity=((current_up[vi]-original_up[vi])/sigma).ravel()
        return np.r_[edges,gravity]

    sparsity=lil_matrix((len(active)*6+len(variables)*3,len(variables)*6),dtype=np.int8)
    for k,e in enumerate(active):
        for node in (e.a,e.b):
            if node in vindex:
                j=vindex[node]*6
                sparsity[k*6:(k+1)*6,j:j+6]=1
    for j in range(len(variables)):
        sparsity[len(active)*6+j*3:len(active)*6+(j+1)*3,j*6+3:j*6+6]=1
    initial=np.zeros(len(variables)*6)
    # Separate final physical tilt guard accounts for the nonlinear rotation-vector bounds.
    tilt=np.radians(maximum_tilt_degrees)
    limits=np.tile([3.,3.,1.,tilt,tilt,np.radians(20.)],len(variables))
    initial_cost=float(.5*np.sum(residual(initial)**2))
    fit=least_squares(residual,initial,jac_sparsity=sparsity.tocsr(),bounds=(-limits,limits),
        method='trf',tr_solver='lsmr',loss='linear',x_scale='jac',max_nfev=max_evaluations,
        ftol=1e-6,xtol=1e-7,gtol=1e-6,tr_options={'atol':1e-7,'btol':1e-7,'maxiter':300},verbose=1)
    rotations,translations=decode(fit.x)
    poses={}
    for i,node in enumerate(ids):
        poses[node]=np.eye(4)
        poses[node][:3,:3]=rotations[i]
        poses[node][:3,3]=translations[i]
    up=rotations.transpose(0,2,1)[:,:,2]
    angles=np.degrees(np.arccos(np.clip(np.sum(up*original_up,axis=1),-1,1)))
    tilt_ok=float(angles.max()) <= maximum_tilt_degrees+1e-6
    report={'success':bool(fit.success and tilt_ok),'status':int(fit.status),'message':str(fit.message),
        'nfev':int(fit.nfev),'njev':int(fit.njev),'initial_block_huber_cost':initial_cost,
        'final_block_huber_cost':float(fit.cost),'cost_note':'Block-Huber edge cost PLUS original-attitude quadratic regularization',
        'active_bounds':int(np.count_nonzero(fit.active_mask)),'optimality':float(fit.optimality),
        'component_anchors':sorted(anchors),'variable_nodes':len(variables),'accepted_for_output':False,
        'gravity_sigma_degrees':gravity_sigma_degrees,'maximum_allowed_tilt_change_degrees':maximum_tilt_degrees,
        'maximum_tilt_change_degrees':float(angles.max()),'physical_tilt_guard_passed':tilt_ok,
        'gravity_source':'regularization to original node attitude; NOT independent IMU evidence',
        'floor_factors':0}
    return poses,report
