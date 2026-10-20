#!/usr/bin/env python3
"""Batch-equivalent RGB-D pose/landmark residual with the original BA API.

Preserves right pose corrections, fixed optical calibration, observation order,
safe projection depth, missing-depth zero residuals, local priors and sparsity.
No matcher, gate, robust loss, measurement or parameterization is changed here.
"""
from __future__ import annotations

import numpy as np
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from floor2_vectorized_solver import batch_inverse
from remeasure_floor2_revisits import SETTINGS


def make_bundle_problem(nodes, gauge, initial, observations, landmark_initial, priors, calibrations):
    nodes = list(nodes)
    index = {n: i for i, n in enumerate(nodes)}
    varying = [n for n in nodes if n != gauge]
    pose_index = {n: i for i, n in enumerate(varying)}
    variable_indices = np.array([index[n] for n in varying], dtype=int)
    point_start = 6*len(varying)
    x0 = np.r_[np.zeros(point_start), np.asarray(landmark_initial).ravel()]
    initial_array = np.stack([initial[n] for n in nodes])
    optical = np.stack([calibrations[n][1] for n in nodes])
    obs_nodes = np.array([index[o[0]] for o in observations], dtype=int)
    tracks = np.array([o[1] for o in observations], dtype=int)
    pixels = np.array([o[2] for o in observations]).reshape(-1,2)
    depths = np.array([o[3] for o in observations])
    valid_depth = np.isfinite(depths)
    intrinsics = np.array([calibrations[o[0]][0] for o in observations]).reshape(-1,3,3)
    prior_a = np.array([index[p[0]] for p in priors],dtype=int)
    prior_b = np.array([index[p[1]] for p in priors],dtype=int)
    inv_measurement = batch_inverse(np.array([p[2] for p in priors]).reshape(-1,4,4))
    sigma_t = np.array([p[3] for p in priors])[:,None]
    sigma_r = np.array([p[4] for p in priors])[:,None]

    def poses_array(x):
        poses = initial_array.copy()
        if varying:
            delta = np.zeros((len(varying),4,4))
            delta[:,3,3] = 1.
            corrections = x[:point_start].reshape(-1,6)
            delta[:,:3,:3] = Rotation.from_rotvec(corrections[:,3:]).as_matrix()
            delta[:,:3,3] = corrections[:,:3]
            poses[variable_indices] = np.matmul(initial_array[variable_indices],delta)
        return poses

    def unpack(x):
        poses = poses_array(x)
        # Match the reference gauge identity and dictionary insertion order.
        result = {gauge: initial[gauge]}
        result.update({n: poses[index[n]] for n in varying})
        return result, x[point_start:].reshape(-1,3)

    def residual(x):
        poses = poses_array(x)
        landmarks = x[point_start:].reshape(-1,3)
        obs = np.zeros((len(observations),3))
        if len(observations):
            camera_inverse = batch_inverse(np.matmul(poses,optical))
            transforms = camera_inverse[obs_nodes]
            p = np.matmul(transforms[:,:3,:3],landmarks[tracks,:,None])[:,:,0]+transforms[:,:3,3]
            safe_z = np.maximum(p[:,2],.05)
            q = np.column_stack((intrinsics[:,0,0]*p[:,0]/safe_z+intrinsics[:,0,2],
                                 intrinsics[:,1,1]*p[:,1]/safe_z+intrinsics[:,1,2]))
            obs[:,:2] = (q-pixels)/SETTINGS['reprojection_sigma_px']
            obs[valid_depth,2] = (p[valid_depth,2]-depths[valid_depth])/SETTINGS['depth_sigma_m']
        prior_errors = np.zeros((len(priors),6))
        if len(priors):
            # Preserve the scalar backend's LEFT associative multiplication.
            error = np.matmul(np.matmul(inv_measurement,batch_inverse(poses[prior_a])),poses[prior_b])
            prior_errors[:,:3] = error[:,:3,3]/sigma_t
            prior_errors[:,3:] = Rotation.from_matrix(error[:,:3,:3]).as_rotvec()/sigma_r
        return np.r_[obs.ravel(),prior_errors.ravel()]

    sparsity = lil_matrix((len(observations)*3+len(priors)*6,len(x0)),dtype=int)
    for i,(node,track,*_) in enumerate(observations):
        if node in pose_index:
            j=pose_index[node]*6;sparsity[3*i:3*i+3,j:j+6]=1
        j=point_start+track*3;sparsity[3*i:3*i+3,j:j+3]=1
    for i,(a,b,*_) in enumerate(priors):
        row=3*len(observations)+6*i
        for node in (a,b):
            if node in pose_index:
                j=pose_index[node]*6;sparsity[row:row+6,j:j+6]=1
    return x0,residual,sparsity.tocsr(),unpack
