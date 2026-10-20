#!/usr/bin/env python3
"""Targeted cross-visit RGB-D track BA and separately labelled surface hypotheses.

All DB reads use SQLite mode=ro. Whole frames are reserved before feature
extraction. BA variables really include each training-camera SE(3) and shared
3-D landmarks; calibrated intrinsics/extrinsics and depth scale stay fixed.
The historically inspected neighborhoods are development data, not a new test.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import shlex
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from check_session_overlap import (Session, contact_sheet, mutual_ratio_matches,
                                   rigid_ransac, root_sift)
from export_rtabmap_db import load_pose_csv
from optimize_session import apply, inverse, sha256_file, utc_now
from recover_geometry_connections import intrinsics_at_rgb, fit_pnp
from refine_geometry_surfaces import refine, reduce_voxels, geometric_diagnostic
from revalidate_loop_constraints import covariance_model, delta_summary

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {
    "targets": [[548, 981], [656, 1055]],
    "train_offsets": list(range(-4, 5)), "test_offsets": [-9, -7, 7, 9],
    "development_revision": "dense adjacency and one-sided depth PnP after first pass yielded 0/3 persistent 2+2-view tracks; withheld IDs unchanged",
    "feature_variant": "CLAHE(2,8x8)+RootSIFT", "maximum_features": 6000,
    "sift_contrast": .02, "ratio": .75, "ransac_iterations": 2000,
    "ransac_threshold_m": .10, "minimum_pair_inliers": 8,
    "minimum_pair_ratio": .35, "minimum_tracks": 12,
    "minimum_track_frames_each_visit": 2, "maximum_tracks": 250,
    "reprojection_sigma_px": 3., "depth_sigma_m": .04,
    "local_prior_sigma_m": .08, "local_prior_sigma_deg": 3.,
    "boundary_prior_sigma_m": .40, "boundary_prior_sigma_deg": 10.,
    "ba_loss": "huber", "ba_loss_scale": 1., "ba_max_evaluations": 250,
    "maximum_reprojection_p90_px": 8., "maximum_depth_p90_m": .10,
    "minimum_rigid_eigenvalue_ratio": 1e-4,
    "minimum_train_surface_overlap": .4, "maximum_surface_rmse_m": .08,
    "minimum_surface_improvement_m": .002,
    "model_sigma_translation_m": .12, "model_sigma_rotation_deg": 4.,
    "voxel_m": .06,
}


class Features(Session):
    """One predeclared low-light feature extractor; raw RGB is kept for evidence."""
    def feature(self, node, full=True):
        key = (node, full)
        if key not in self.features:
            gray = cv2.cvtColor(self.image(node), cv2.COLOR_BGR2GRAY)
            gray = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(gray)
            keys, descriptors = cv2.SIFT_create(nfeatures=SETTINGS["maximum_features"],
                    contrastThreshold=SETTINGS["sift_contrast"]).detectAndCompute(gray, None)
            self.features[key] = keys, root_sift(descriptors), 1.
        return self.features[key]


def freeze_selection(available):
    records = []
    for a, b in SETTINGS["targets"]:
        train = [[center + d for d in SETTINGS["train_offsets"]] for center in (a, b)]
        test = [[center + d for d in SETTINGS["test_offsets"]] for center in (a, b)]
        if not set(sum(train + test, [])) <= set(available):
            raise ValueError("Predeclared frame inventory is incomplete")
        if set(sum(train, [])) & set(sum(test, [])):
            raise ValueError("Whole-frame training/test leakage")
        records.append({"a": a, "b": b, "train_frames": train, "test_frames": test,
            "role": "development", "historical_holdout_promoted_to_development": a == 656,
            "test_scope": "New fitting excludes complete test frames; historically inspected same sensor and correlated local pose chain, not ground truth"})
    return records


def conflict_free_tracks(edges):
    """Descending confidence union, rejecting components with duplicate cameras."""
    parent, members = {}, {}
    def find(x):
        parent.setdefault(x, x); members.setdefault(x, {x})
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]
    rejected = 0
    for score, u, v in sorted(edges, key=lambda e: (-e[0], e[1], e[2])):
        pu, pv = find(u), find(v)
        if pu == pv:
            continue
        if {n for n, _ in members[pu]} & {n for n, _ in members[pv]}:
            rejected += 1; continue
        parent[pv] = pu; members[pu] |= members.pop(pv)
    return [sorted(v) for v in members.values() if len(v) >= 2], rejected


def pair_features(session, a, b):
    ka, da, _ = session.feature(a)
    kb, db, _ = session.feature(b)
    matches = mutual_ratio_matches(da, db, SETTINGS["ratio"])
    record = {"a": a, "b": b, "mutual_matches": len(matches), "usable": False}
    if not matches:
        return record, [], None
    pixels_a = np.array([ka[m.queryIdx].pt for m in matches])
    pixels_b = np.array([kb[m.trainIdx].pt for m in matches])
    target, source = session.unproject(a, pixels_a), session.unproject(b, pixels_b)
    valid = np.isfinite(target).all(1) & np.isfinite(source).all(1)
    ids = np.flatnonzero(valid)
    record["both_depth_matches"] = len(ids)
    hypotheses = []
    if len(ids) >= SETTINGS["minimum_pair_inliers"]:
        transform, inliers, trials = rigid_ransac(source[ids], target[ids], 20261017+a*10007+b,
            SETTINGS["ransac_threshold_m"], SETTINGS["ransac_iterations"])
        if inliers.sum() >= SETTINGS["minimum_pair_inliers"] and inliers.mean() >= SETTINGS["minimum_pair_ratio"]:
            hypotheses.append(("symmetric_depth_ransac", ids[inliers], transform, float(inliers.mean())))
    # Missing depth in one camera must not delete a visible RGB feature track.
    # Metric seed still needs at least 12 source depths; no pure essential-matrix scale.
    for direction, points, pixels, node, forward in [
            ("depth_b_to_rgb_a_pnp", source, pixels_a, a, True),
            ("depth_a_to_rgb_b_pnp", target, pixels_b, b, False)]:
        available = np.flatnonzero(np.isfinite(points).all(1))
        if len(available) < 12: continue
        intrinsic, local = intrinsics_at_rgb(session, node)
        transform, detail = fit_pnp(points[available], pixels[available], intrinsic, local, 20261017+a*10007+b)
        if transform is not None and detail["inlier_ratio"] >= SETTINGS["minimum_pair_ratio"]:
            indices = available[np.array(detail["inlier_indices"])]
            hypotheses.append((direction, indices, transform if forward else inverse(transform), detail["inlier_ratio"]))
    if not hypotheses:
        return record, [], None
    method, indices, transform, ratio = max(hypotheses, key=lambda x: len(x[1]))
    record.update(inliers=len(indices), ratio=ratio, method=method, T_a_b=transform.tolist(), usable=True)
    good = [(1./(1.+matches[i].distance), (a, matches[i].queryIdx),
             (b, matches[i].trainIdx)) for i in indices] if record["usable"] else []
    evidence = {"pixels_a": pixels_a[indices], "pixels_b": pixels_b[indices],
                "target": target[indices], "source": source[indices]}
    return record, good, evidence


def skew(p):
    x, y, z = p
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rigid_observability(points):
    """Fixed corresponding 3-D landmarks, no odometry/pose priors in this rank."""
    if len(points) < 3:
        return {"rank": 0, "minimum_eigenvalue_ratio": 0.}
    centered = points - np.mean(points, axis=0)
    radius = max(np.sqrt(np.mean(np.sum(centered**2, axis=1))), .1)
    jacobian = np.vstack([np.column_stack((np.eye(3), -skew(p/radius))) for p in centered])
    eig = np.linalg.eigvalsh(jacobian.T @ jacobian / len(points))
    return {"rank": int(np.sum(eig > eig[-1]*1e-4)), "eigenvalues": eig.tolist(),
            "minimum_eigenvalue_ratio": float(eig[0]/max(eig[-1], 1e-12)),
            "rotation_normalization_m": float(radius),
            "meaning": "Fixed-landmark 3-D rigid alignment rank, NOT full BA Schur uncertainty"}


def pose_delta(vector):
    out = np.eye(4)
    out[:3, 3] = vector[:3]
    out[:3, :3] = Rotation.from_rotvec(vector[3:]).as_matrix()
    return out


def make_bundle_problem(nodes, gauge, initial, observations, landmark_initial, priors, calibrations):
    """Return genuine pose/landmark residual and sparse block incidence pattern."""
    varying = [n for n in nodes if n != gauge]
    pose_index = {n: i for i, n in enumerate(varying)}
    point_start = 6*len(varying)
    x0 = np.r_[np.zeros(point_start), np.asarray(landmark_initial).ravel()]
    def unpack(x):
        poses = {gauge: initial[gauge]}
        poses.update({n: initial[n] @ pose_delta(x[6*i:6*i+6]) for n, i in pose_index.items()})
        return poses, x[point_start:].reshape(-1, 3)
    def residual(x):
        poses, landmarks = unpack(x)
        values = []
        for node, track, pixel, z in observations:
            intrinsic, local = calibrations[node]
            p = apply(landmarks[track:track+1], inverse(poses[node] @ local))[0]
            safe_z = max(p[2], .05)
            q = np.array([intrinsic[0, 0]*p[0]/safe_z + intrinsic[0, 2],
                          intrinsic[1, 1]*p[1]/safe_z + intrinsic[1, 2]])
            values.extend((q - pixel)/SETTINGS["reprojection_sigma_px"])
            values.append((p[2]-z)/SETTINGS["depth_sigma_m"] if np.isfinite(z) else 0.)
        for a, b, measurement, sigma_t, sigma_r in priors:
            error = inverse(measurement) @ inverse(poses[a]) @ poses[b]
            values.extend(error[:3, 3]/sigma_t)
            values.extend(Rotation.from_matrix(error[:3, :3]).as_rotvec()/sigma_r)
        return np.asarray(values)
    sparsity = lil_matrix((len(observations)*3 + len(priors)*6, len(x0)), dtype=int)
    for i, (node, track, *_rest) in enumerate(observations):
        if node in pose_index:
            j = pose_index[node]*6; sparsity[3*i:3*i+3, j:j+6] = 1
        j = point_start + track*3; sparsity[3*i:3*i+3, j:j+3] = 1
    for i, (a, b, *_rest) in enumerate(priors):
        row = 3*len(observations) + 6*i
        for n in (a, b):
            if n in pose_index:
                j = pose_index[n]*6; sparsity[row:row+6, j:j+6] = 1
    return x0, residual, sparsity.tocsr(), unpack


def feature_bundle(session, spec, starts, frames, directory):
    nodes = sum(spec["train_frames"], [])
    membership = {n: i for i, visit in enumerate(spec["train_frames"]) for n in visit}
    records, edges, evidence = [], [], {}
    for a, b in itertools.combinations(nodes, 2):
        record, pair_edges, pair_evidence = pair_features(session, a, b)
        record["cross_visit"] = membership[a] != membership[b]
        records.append(record); edges += pair_edges
        if pair_evidence is not None:
            evidence[(a, b)] = pair_evidence
        print(f"match {a}/{b}: {record.get('inliers', 0)}/{record.get('both_depth_matches', 0)} usable={record['usable']}", flush=True)
    components, conflicts = conflict_free_tracks(edges)
    tracks = [t for t in components if all(sum(membership[n] == visit for n, _ in t) >=
              SETTINGS["minimum_track_frames_each_visit"] for visit in (0, 1))]
    # Every landmark has a measured metric depth on both visits; other cameras may
    # contribute RGB-only observations when their depth is missing/low confidence.
    tracks = [t for t in tracks if all(any(membership[n] == visit and
        np.isfinite(session.unproject(n, np.array([session.feature(n)[0][k].pt]))).all()
        for n, k in t) for visit in (0, 1))]
    tracks.sort(key=lambda t: (-len(t), t))
    tracks = tracks[:SETTINGS["maximum_tracks"]]
    result = {"pair_records": records, "track_components": len(components),
              "duplicate_frame_unions_rejected": conflicts, "cross_visit_tracks": len(tracks),
              "tracks": [[[int(n), int(k)] for n, k in t] for t in tracks],
              "ba_executed": False, "accepted": False}
    cross = [r for r in records if r["cross_visit"] and r["usable"]]
    if cross:
        best = max(cross, key=lambda r: r["inliers"])
        render_matches(session, best["a"], best["b"], evidence[(best["a"], best["b"])], directory/"feature_inliers.png")
    if len(tracks) < SETTINGS["minimum_tracks"]:
        result["reason"] = "fewer_than_12_tracks_observed_in_two_frames_of_each_visit"
        return result, None
    a, b = spec["a"], spec["b"]
    starts = dict(starts)
    if cross:
        best = max(cross, key=lambda r: r["inliers"])
        pa, pb = best["a"], best["b"]
        reference = starts["selected_initial"]
        starts["visual_ransac"] = dict(reference)
        corrected_pb = reference[pa] @ np.array(best["T_a_b"])
        for n in spec["train_frames"][1]:
            starts["visual_ransac"][n] = corrected_pb @ inverse(reference[pb]) @ reference[n]
    calibrations = {n: intrinsics_at_rgb(session, n) for n in nodes}
    observations, observed_node_points = [], {}
    for j, track in enumerate(tracks):
        for node, feature in track:
            pixel = np.array(session.feature(node)[0][feature].pt)
            point = session.unproject(node, pixel[None])[0]
            optical = apply(point[None], inverse(calibrations[node][1]))[0]
            observations.append((node, j, pixel, optical[2]))
            observed_node_points[node, j] = point
    candidates = []
    for name, absolute in starts.items():
        initial = {n: inverse(absolute[a]) @ absolute[n] for n in nodes}
        landmarks = [np.mean([apply(observed_node_points[n, j][None], initial[n])[0]
                     for n, _ in track if np.isfinite(observed_node_points[n, j]).all()], axis=0)
                     for j, track in enumerate(tracks)]
        priors, prior_records = [], []
        for visit in spec["train_frames"]:
            for n, m in zip(visit[:-1], visit[1:]):
                weak = any(frames[i]["quality_grade"] == "C" or frames[i]["hard_boundary"] == "True"
                           for i in range(n, m+1))
                st = SETTINGS["boundary_prior_sigma_m" if weak else "local_prior_sigma_m"]
                sr = np.radians(SETTINGS["boundary_prior_sigma_deg" if weak else "local_prior_sigma_deg"])
                # All starts use frozen selected relative motion, not a fitted cross-visit prior.
                base = starts["selected_initial"]
                priors.append((n, m, inverse(base[n]) @ base[m], st, sr))
                prior_records.append({"a": n, "b": m, "weak_quality_boundary": weak,
                                      "sigma_m": st, "sigma_deg": float(np.degrees(sr))})
        x0, residual, sparse, unpack = make_bundle_problem(nodes, a, initial, observations,
                                                           landmarks, priors, calibrations)
        fit = least_squares(residual, x0, jac_sparsity=sparse, loss="huber", f_scale=1.,
                max_nfev=SETTINGS["ba_max_evaluations"], x_scale="jac",
                ftol=1e-7, xtol=1e-7, gtol=1e-7)
        poses, points = unpack(fit.x)
        raw = residual(fit.x)[:3*len(observations)].reshape(-1, 3)
        reprojection = np.linalg.norm(raw[:, :2], axis=1)*SETTINGS["reprojection_sigma_px"]
        depth_valid = np.array([np.isfinite(o[3]) for o in observations])
        depth = np.abs(raw[depth_valid, 2])*SETTINGS["depth_sigma_m"]
        positive = all(apply(points[j:j+1], inverse(poses[n] @ calibrations[n][1]))[0, 2] > .1
                       for n, j, *_ in observations)
        rank = rigid_observability(points)
        candidate = {"name": name, "success": bool(fit.success), "message": fit.message,
            "nfev": fit.nfev, "cost": float(fit.cost), "optimality": float(fit.optimality),
            "reprojection_median_px": float(np.median(reprojection)),
            "reprojection_p90_px": float(np.percentile(reprojection, 90)),
            "depth_median_abs_m": float(np.median(depth)),
            "depth_p90_abs_m": float(np.percentile(depth, 90)), "positive_depth": positive,
            "rgb_observations": len(observations), "depth_observations": int(depth_valid.sum()),
            "fixed_landmark_rigid_observability": rank, "priors": prior_records,
            "T_a_b": (inverse(poses[a]) @ poses[b]).tolist(),
            "poses": {str(n): p.tolist() for n, p in poses.items()}, "landmarks": points.tolist()}
        candidates.append(candidate)
        print(f"BA {a}/{b} {name}: success={fit.success} cost={fit.cost:.2f}", flush=True)
    chosen = min(candidates, key=lambda c: c["cost"])
    reasons = []
    if not chosen["success"]: reasons.append("bundle_optimizer_not_converged")
    if not chosen["positive_depth"]: reasons.append("nonpositive_landmark_depth")
    if chosen["reprojection_p90_px"] > SETTINGS["maximum_reprojection_p90_px"]:
        reasons.append("reprojection_p90_above_8px")
    if chosen["depth_p90_abs_m"] > SETTINGS["maximum_depth_p90_m"]:
        reasons.append("depth_p90_above_10cm")
    if chosen["fixed_landmark_rigid_observability"]["rank"] < 6:
        reasons.append("rigid_correspondence_geometry_degenerate")
    result.update(ba_executed=True, candidates=candidates, chosen=chosen["name"],
        accepted=not reasons, reasons=reasons, T_a_b=chosen["T_a_b"],
        scope="Training-only gates; whole-frame withheld metrics must be evaluated separately")
    return result, np.array(chosen["T_a_b"])


def continue_bundle(session, spec, previous, starts, max_evaluations, directory):
    """Continue the same tracks/objective, changing only linear solve accuracy/budget."""
    if not previous["ba_executed"]:
        return previous, None
    result = dict(previous)
    nodes = sum(spec["train_frames"], [])
    a = spec["a"]
    calibrations = {n: intrinsics_at_rgb(session, n) for n in nodes}
    observations = []
    for j, track in enumerate(previous["tracks"]):
        for node, feature in track:
            pixel = np.array(session.feature(node)[0][feature].pt)
            point = session.unproject(node, pixel[None])[0]
            optical = apply(point[None], inverse(calibrations[node][1]))[0]
            observations.append((node, j, pixel, optical[2]))
    frozen_observations = [{"node": n, "landmark": j, "rgb_pixel": uv.tolist(),
        "optical_depth_m": float(z) if np.isfinite(z) else None} for n, j, uv, z in observations]
    frozen_bytes = (json.dumps(frozen_observations, separators=(",", ":"), allow_nan=False)+"\n").encode()
    (directory/"frozen_observations.json").write_bytes(frozen_bytes)
    candidates = []
    for old in previous["candidates"]:
        if old["name"] not in ("visual_ransac", "previous_preview"):
            continue
        initial = {int(n): np.array(p) for n, p in old["poses"].items()}
        landmarks = np.array(old["landmarks"])
        priors = [(r["a"], r["b"], inverse(starts["selected_initial"][r["a"]]) @ starts["selected_initial"][r["b"]],
            r["sigma_m"], np.radians(r["sigma_deg"])) for r in old["priors"]]
        x0, residual, sparse, unpack = make_bundle_problem(nodes, a, initial, observations, landmarks, priors, calibrations)
        fit = least_squares(residual, x0, jac_sparsity=sparse, loss="huber", f_scale=1.,
            max_nfev=max_evaluations, x_scale="jac", ftol=1e-7, xtol=1e-7, gtol=1e-7,
            tr_options={"atol": 1e-9, "btol": 1e-9, "maxiter": 1500})
        poses, points = unpack(fit.x)
        raw = residual(fit.x)[:3*len(observations)].reshape(-1, 3)
        reprojection = np.linalg.norm(raw[:, :2], axis=1)*SETTINGS["reprojection_sigma_px"]
        valid_depth = np.array([np.isfinite(o[3]) for o in observations])
        depth = np.abs(raw[valid_depth, 2])*SETTINGS["depth_sigma_m"]
        positive = all(apply(points[j:j+1], inverse(poses[n] @ calibrations[n][1]))[0, 2] > .1
                       for n, j, *_ in observations)
        candidate = dict(old, success=bool(fit.success), message=fit.message,
            nfev=fit.nfev, previous_nfev=old["nfev"], previous_cost=old["cost"],
            cost=float(fit.cost), optimality=float(fit.optimality),
            reprojection_median_px=float(np.median(reprojection)),
            reprojection_p90_px=float(np.percentile(reprojection, 90)),
            depth_median_abs_m=float(np.median(depth)), depth_p90_abs_m=float(np.percentile(depth, 90)),
            positive_depth=positive, fixed_landmark_rigid_observability=rigid_observability(points),
            T_a_b=(inverse(poses[a]) @ poses[spec["b"]]).tolist(),
            poses={str(n): p.tolist() for n, p in poses.items()}, landmarks=points.tolist(),
            continuation_settings={"maximum_evaluations": max_evaluations, "lsmr_atol": 1e-9,
                "lsmr_btol": 1e-9, "lsmr_maxiter": 1500,
                "same_objective": "Identical observations, local-prior measurements/sigmas, Huber scale, gauge and ftol/xtol/gtol; poses reparameterized around previous iterate"})
        candidates.append(candidate)
        print(f"BA continuation {spec['a']}/{spec['b']} {old['name']}: success={fit.success} nfev={fit.nfev} cost={fit.cost:.3f}", flush=True)
    chosen = min(candidates, key=lambda c: c["cost"])
    reasons = []
    if not chosen["success"]: reasons.append("bundle_optimizer_not_converged")
    if not chosen["positive_depth"]: reasons.append("nonpositive_landmark_depth")
    if chosen["reprojection_p90_px"] > SETTINGS["maximum_reprojection_p90_px"]: reasons.append("reprojection_p90_above_8px")
    if chosen["depth_p90_abs_m"] > SETTINGS["maximum_depth_p90_m"]: reasons.append("depth_p90_above_10cm")
    if chosen["fixed_landmark_rigid_observability"]["rank"] < 6: reasons.append("rigid_correspondence_geometry_degenerate")
    result.update(candidates=candidates, chosen=chosen["name"], accepted=not reasons, reasons=reasons,
        T_a_b=chosen["T_a_b"], frozen_observations_sha256=hashlib.sha256(frozen_bytes).hexdigest(),
        continuation=True, observation_count=len(observations),
        numeric_rank_note="Full nonlinear BA convergence is separate from conditional fixed-landmark rank. Local priors may still tether cameras without visual tracks.")
    best_pair = max((p for p in result["pair_records"] if p["cross_visit"] and p["usable"]), key=lambda p: p["inliers"])
    # Render only the existing frozen pair observations; no new matches used in fit.
    p, q = best_pair["a"], best_pair["b"]
    pairs = []
    for track in result["tracks"]:
        observed = dict(track)
        if p in observed and q in observed:
            pairs.append([session.feature(p)[0][observed[p]].pt, session.feature(q)[0][observed[q]].pt])
    if pairs:
        pairs = np.asarray(pairs)
        render_matches(session, p, q, {"pixels_a": pairs[:, 0], "pixels_b": pairs[:, 1]}, directory/"feature_inliers.png")
    return result, np.array(chosen["T_a_b"])


def cloud_window(session, poses, center, nodes):
    return reduce_voxels(np.vstack([apply(session.cloud(n), inverse(poses[center]) @ poses[n])
                                    for n in nodes]), SETTINGS["voxel_m"])


def surface_hypotheses(session, spec, starts, native, directory, ba_transform):
    a, b = spec["a"], spec["b"]
    poses = starts["selected_initial"]
    train_a, train_b = [cloud_window(session, poses, center, nodes)
                for center, nodes in zip((a, b), spec["train_frames"])]
    seeds = {name: inverse(p[a]) @ p[b] for name, p in starts.items()}
    for edge in native["verified_loops"]:
        if (edge["from_id"], edge["to_id"]) == (a, b):
            seeds["native_stored_loop"] = np.array(edge["measurement"])
        elif (edge["to_id"], edge["from_id"]) == (a, b):
            seeds["native_stored_loop"] = inverse(np.array(edge["measurement"]))
    if ba_transform is not None: seeds["multi_frame_ba"] = ba_transform
    candidates = []
    for name, seed in seeds.items():
        fitted, info = refine(train_a, train_b, seed, iterations=50)
        before = geometric_diagnostic(train_a, train_b, seed, poses[a][:3, :3])
        after = geometric_diagnostic(train_a, train_b, fitted, poses[a][:3, :3])
        reverse, _ = refine(train_b, train_a, inverse(fitted), iterations=40)
        candidates.append({"name": name, "seed_T_a_b": seed.tolist(), "T_a_b": fitted.tolist(),
            "train_before": before, "train_after": after, "fit": info,
            "cycle": delta_summary(np.eye(4), fitted @ reverse),
            "correction": delta_summary(seed, fitted)})
    chosen = min(candidates, key=lambda c: c["train_after"]["cloud"]["clipped_rmse_m"])
    # Selection is already fixed before any complete heldout frame is loaded.
    test_a, test_b = [cloud_window(session, poses, center, nodes)
                for center, nodes in zip((a, b), spec["test_frames"])]
    transform = np.array(chosen["T_a_b"])
    before = geometric_diagnostic(test_a, test_b, seeds["selected_initial"], poses[a][:3, :3])
    after = geometric_diagnostic(test_a, test_b, transform, poses[a][:3, :3])
    render_clouds(directory/"withheld_surface_before_after.png", test_a, test_b,
                  seeds["selected_initial"], transform, poses[a][:3, :3], f"{a}/{b}: complete withheld RGB-D frames")
    return {"kind": "rigid_multi_frame_point_plane_surface_hypothesis_not_bundle_adjustment",
        "candidates": candidates, "chosen": chosen["name"], "T_a_b": chosen["T_a_b"],
        "heldout_before": before, "heldout_after": after,
        "accepted": False, "reason": "Geometry alone does not establish new cross-visit feature identity; parent may review native-loop hypothesis with independent evidence",
        "local_pose_policy": "All window clouds use frozen selected-initial local poses; poor internal local geometry is not repaired by this rigid hypothesis"}


def render_matches(session, a, b, evidence, path):
    first, second = session.image(a), session.image(b)
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.imshow(cv2.cvtColor(np.hstack([first, second]), cv2.COLOR_BGR2RGB))
    width = first.shape[1]
    for p, q in zip(evidence["pixels_a"], evidence["pixels_b"]):
        ax.plot([p[0], q[0]+width], [p[1], q[1]], lw=.4, alpha=.65, color="#42e6d2")
    ax.set_title(f"Actual CLAHE-RootSIFT metric correspondences (3-D RANSAC / depth PnP): {a} / {b}"); ax.axis("off")
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def render_clouds(path, target, source, initial, fitted, gravity, title):
    clouds = [target @ gravity.T, apply(source, initial) @ gravity.T, apply(source, fitted) @ gravity.T]
    low, high = np.percentile(np.vstack(clouds), [1, 99], axis=0)
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    for j, label in enumerate(["Selected initial", "Training-selected surface hypothesis"]):
        for i, dims in enumerate([(0, 1), (0, 2)]):
            ax = axes[i, j]
            for points, color in [(clouds[0], "#0077bb"), (clouds[j+1], "#ee7733")]:
                ax.scatter(points[:, dims[0]], points[:, dims[1]], s=.5, c=color, alpha=.6)
            ax.set(xlim=(low[dims[0]]-.2, high[dims[0]]+.2), ylim=(low[dims[1]]-.2, high[dims[1]]+.2),
                   xlabel="X (m)", ylabel="Y (m)" if i == 0 else "Z (m)", title=label)
            ax.set_aspect("equal"); ax.grid(alpha=.2)
    fig.suptitle(title + "\nSame points / axes; diagnostic hypothesis, not approved map")
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(path, dpi=140); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--selection-only", action="store_true")
    parser.add_argument("--resume-report", type=Path)
    parser.add_argument("--maximum-evaluations", type=int, default=1500)
    args = parser.parse_args()
    paths = {
        "database": ROOT/"data/raw/rtabmap/261017-013501-2f.db",
        "selected_initial": ROOT/"outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv",
        "previous_preview": ROOT/"outputs/student_union_20261017_all_preview/bundle/floor2_trajectory.csv",
        "native": ROOT/"outputs/student_union_20261017_added_reviewed/floor2/validated_edges.json",
        "quality": ROOT/"outputs/student_union_20261017_submap_quality/floor2/frames.csv",
    }
    previous_report = None
    if args.resume_report:
        previous_report = json.loads(args.resume_report.read_text())
        if not previous_report["complete"] or not previous_report["inputs_unchanged"]:
            raise ValueError("Cannot continue incomplete or mutable previous experiment")
        for p in paths.values():
            if previous_report["input_hashes"].get(str(p)) != sha256_file(p):
                raise ValueError(f"Changed observation or initial-pose input: {p}")
        paths["previous_report"] = args.resume_report.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), ROOT/"src/check_session_overlap.py", ROOT/"src/recover_geometry_connections.py",
               ROOT/"src/refine_geometry_surfaces.py", ROOT/"src/optimize_session.py",
               ROOT/"src/export_rtabmap_db.py", ROOT/"src/revalidate_loop_constraints.py"]
    hashes = {str(p): sha256_file(p) for p in list(paths.values()) + sources}
    for p in sources: (args.output/("source_"+p.name)).write_bytes(p.read_bytes())
    starts = {name: load_pose_csv(paths[name]) for name in ("selected_initial", "previous_preview")}
    session = Features(paths["database"], "floor2")
    selection = {"created_utc": utc_now(), "settings": SETTINGS,
        "records": freeze_selection(set.intersection(set(session.ids), *(set(p) for p in starts.values()))),
        "input_hashes": hashes, "recorded_before_feature_extraction": True}
    selection["training_node_ids"] = sorted({n for spec in selection["records"] for n in sum(spec["train_frames"], [])})
    selection["validation_windows"] = [{"name": f"region_{spec['a']}_{spec['b']}",
            "visit_a": spec["test_frames"][0], "visit_b": spec["test_frames"][1]}
            for spec in selection["records"]]
    (args.output/"selection.json").write_text(json.dumps(selection, indent=2)+"\n")
    if args.selection_only:
        session.connection.close(); return
    with paths["quality"].open() as f:
        frames = {int(r["node_id"]): r for r in csv.DictReader(f)}
    native = json.loads(paths["native"].read_text())
    report = {"created_utc": utc_now(), "complete": False, "settings": SETTINGS,
        "command": shlex.join(sys.argv), "input_hashes": hashes,
        "selection_sha256": sha256_file(args.output/"selection.json"), "records": [],
        "coordinate_convention": "T_a_b maps node b to node a; RGB-D BA poses map node to center-a gauge. Optical-to-node extrinsics fixed.",
        "limitations": ["No surveyed ground truth; previously inspected target centers are development.",
            "Complete withheld frames do not enter matching, RANSAC, BA or candidate selection.",
            "Image features only establish their sampled structure; reflective glazing and darkness remain failure modes.",
            "Local pose priors are modeling assumptions, weakened at C-quality or hard boundaries.",
            "Fixed-landmark rigid rank is not calibrated full BA marginal uncertainty."]}
    for spec in selection["records"]:
        directory = args.output/f"{spec['a']}_{spec['b']}"; directory.mkdir()
        contact_sheet(session, sum(spec["train_frames"], []), directory/"source_training_frames.png", columns=5)
        if previous_report:
            old = next(r for r in previous_report["records"] if (r["a"], r["b"]) == (spec["a"], spec["b"]))
            if old["train_frames"] != spec["train_frames"] or old["test_frames"] != spec["test_frames"]:
                raise ValueError("Continuation must preserve frozen frame partitions")
            bundle, transform = continue_bundle(session, spec, old["bundle"], starts, args.maximum_evaluations, directory)
        else:
            bundle, transform = feature_bundle(session, spec, starts, frames, directory)
        (directory/"bundle.json").write_text(json.dumps(bundle, indent=2)+"\n")
        surfaces = surface_hypotheses(session, spec, starts, native, directory, transform)
        contact_sheet(session, sum(spec["test_frames"], []), directory/"source_withheld_frames.png", columns=4)
        record = dict(spec, bundle=bundle, surface_hypotheses=surfaces,
            accepted=bool(bundle["accepted"]), measurement_kind="joint_multiframe_RGBD_pose_landmark_BA" if bundle["ba_executed"] else "no_BA_measurement",
            T_a_b=bundle.get("T_a_b"), **covariance_model(SETTINGS["model_sigma_translation_m"], SETTINGS["model_sigma_rotation_deg"]))
        # Passing BA training gates only makes an integration candidate, never automatic publication.
        record["requires_whole_frame_validation_before_integration"] = True
        report["records"].append(record)
        (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    session.connection.close()
    report["inputs_unchanged"] = all(sha256_file(Path(p)) == digest for p, digest in hashes.items())
    if not report["inputs_unchanged"]: raise RuntimeError("Input/source hash changed during experiment")
    report.update(complete=True, finished_utc=utc_now())
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
