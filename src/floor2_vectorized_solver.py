#!/usr/bin/env python3
"""Batch-equivalent fixed-weight solver; does not modify the original backend.

The parameterization, residual order, Cholesky whitening, finite-difference
sparsity, componentwise bounds and SciPy solver options match
``optimize_robust_submaps._solve_fixed_weights``. Only evaluation of all frame
transforms and all edge residuals is batched. There is no changed objective,
new analytic Jacobian, altered convergence tolerance, or graph selection.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


def batch_inverse(transforms):
    """Rigid inverse with the same multiplication order as the scalar backend."""
    result = np.zeros_like(transforms)
    result[:, 3, 3] = 1.
    result[:, :3, :3] = transforms[:, :3, :3].transpose(0, 2, 1)
    result[:, :3, 3] = -np.matmul(result[:, :3, :3], transforms[:, :3, 3, None])[:, :, 0]
    return result


@dataclass
class ResidualProblem:
    initial: np.ndarray
    variable_ids: list[str]
    variable_indices: np.ndarray
    source_indices: np.ndarray
    target_indices: np.ndarray
    inverse_measurements: np.ndarray
    whitening: np.ndarray
    weight_scale: np.ndarray
    jac_sparsity: object

    def poses(self, x):
        result = self.initial.copy()
        if len(self.variable_ids):
            correction = np.asarray(x).reshape(-1, 6)
            rotations = Rotation.from_rotvec(correction[:, 3:]).as_matrix()
            result[self.variable_indices, :3, :3] = np.matmul(
                rotations, self.initial[self.variable_indices, :3, :3])
            result[self.variable_indices, :3, 3] = (
                self.initial[self.variable_indices, :3, 3] + correction[:, :3])
        return result

    def __call__(self, x):
        if not len(self.source_indices):
            return np.zeros(0)
        poses = self.poses(x)
        # Keep the exact scalar backend order: inv(Ta) @ Tb, then inv(M) @ P.
        # Using Ra.T @ (tb-ta) instead is algebraically equivalent but changes
        # floating-point cancellation for large world translations.
        predicted = np.matmul(batch_inverse(poses[self.source_indices]),
                              poses[self.target_indices])
        error = np.matmul(self.inverse_measurements, predicted)
        errors = np.concatenate((error[:, :3, 3],
                                 Rotation.from_matrix(error[:, :3, :3]).as_rotvec()), axis=1)
        whitened = np.matmul(self.whitening, errors[:, :, None])[:, :, 0]
        return (self.weight_scale[:, None] * whitened).ravel()


def build_residual(nodes, train_edges, gauges, weights):
    """Expose a callable residual and identical sparsity for independent tests."""
    node_lookup = {node.id: index for index, node in enumerate(nodes)}
    variable_ids = [node.id for node in nodes if node.id not in gauges]
    variable_lookup = {node_id: index for index, node_id in enumerate(variable_ids)}
    initial = np.stack([node.initial for node in nodes]) if nodes else np.empty((0, 4, 4))
    source = np.asarray([node_lookup[e.source] for e in train_edges], dtype=np.int64)
    target = np.asarray([node_lookup[e.target] for e in train_edges], dtype=np.int64)
    measurement = (np.stack([edge.measurement for edge in train_edges])
                   if train_edges else np.empty((0, 4, 4)))
    # Per-edge Cholesky is prepared once, identically to the reference solver.
    whitening = (np.stack([np.linalg.cholesky(edge.information).T for edge in train_edges])
                 if train_edges else np.empty((0, 6, 6)))
    sparsity = lil_matrix((len(train_edges)*6, len(variable_ids)*6), dtype=np.int8)
    for edge_index, edge in enumerate(train_edges):
        for node_id in (edge.source, edge.target):
            if node_id in variable_lookup:
                column = variable_lookup[node_id]*6
                sparsity[edge_index*6:(edge_index+1)*6, column:column+6] = 1
    return ResidualProblem(initial, variable_ids,
                           np.asarray([node_lookup[n] for n in variable_ids], dtype=np.int64),
                           source, target, batch_inverse(measurement), whitening,
                           np.asarray([math.sqrt(weights[e.id]) for e in train_edges]),
                           sparsity.tocsr())


def solve_fixed_weights(nodes, train_edges, gauges, weights, x0,
                        max_evaluations, translation_bound, rotation_bound_degrees):
    """Drop-in compatible implementation of the reference private function."""
    problem = build_residual(nodes, train_edges, gauges, weights)
    if not problem.variable_ids or not train_edges:
        return x0.copy(), {'success': True, 'nfev': 0, 'cost': 0., 'message': 'nothing to optimize'}
    bounds = np.tile([translation_bound]*3 + [math.radians(rotation_bound_degrees)]*3,
                     len(problem.variable_ids))
    fit = least_squares(problem, x0, jac_sparsity=problem.jac_sparsity, bounds=(-bounds, bounds),
                        method='trf', tr_solver='lsmr', x_scale='jac', max_nfev=max_evaluations,
                        ftol=1e-9, xtol=1e-9, gtol=1e-9,
                        tr_options={'atol': 1e-10, 'btol': 1e-10, 'maxiter': 500})
    return fit.x, {'success': bool(fit.success), 'nfev': int(fit.nfev),
                   'cost': float(fit.cost), 'message': str(fit.message),
                   'active_bounds': int(np.count_nonzero(fit.active_mask))}


_solve_fixed_weights = solve_fixed_weights
