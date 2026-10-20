#!/usr/bin/env python3
"""Conservative sequential RGB-D remeasurement inside floor2 quality runs.

Original databases are opened read-only. No old cross-session constraint,
whole-pair holdout neighbourhood, or test depth sample is fitted. Temporal
proximity establishes a local-registration hypothesis, not distant place
identity. Test pixels remain correlated observations, not ground truth.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shlex
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation

from check_session_overlap import Session, mutual_ratio_matches, rigid_ransac
from export_rtabmap_db import load_pose_csv
from optimize_session import apply, geometric_score, inverse, sha256_file, utc_now
from refine_geometry_surfaces import refine, reduce_voxels
from revalidate_loop_constraints import covariance_model, delta_summary

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {
    "first_node": 250, "last_node": 1100, "maximum_pairs": 160,
    "node_spans": [1, 3, 5], "maximum_elapsed_s": 5.5,
    "maximum_initial_translation_m": 1.5, "maximum_initial_rotation_deg": 40.,
    "whole_pair_holdout_neighbourhood_nodes": 10,
    "depth_min_m": .25, "depth_max_m": 4., "confidence_min": 50,
    "depth_test_tile_side_px": 16, "depth_test_tile_modulus": 5,
    "depth_train_guard_px": 3, "cloud_pixel_stride": 2, "voxel_m": .06,
    "minimum_cloud_points": 200, "minimum_visual_train_matches": 12,
    "minimum_visual_consensus_ratio": .5, "visual_ransac_threshold_m": .08,
    "visual_ransac_iterations": 900, "visual_max_prior_delta_m": .25,
    "visual_max_prior_delta_deg": 5., "point_plane_iterations": 35,
    "minimum_observability_rank": 6, "minimum_eigenvalue_ratio": 1e-4,
    "minimum_train_overlap": .45, "maximum_train_inlier_rmse_m": .07,
    "minimum_train_clipped_rmse_improvement_m": .0005,
    "maximum_correction_translation_m": .15, "maximum_correction_rotation_deg": 3.,
    "maximum_bidirectional_cycle_m": .03, "maximum_bidirectional_cycle_deg": 1.,
    "model_sigma_translation_m": .10, "model_sigma_rotation_deg": 3.,
}


def partition_masks(shape):
    """Fixed image-depth tile split; no train depth within 3px of test depth."""
    v, u = np.indices(shape)
    side = SETTINGS["depth_test_tile_side_px"]
    test = (u // side + 2 * (v // side)) % SETTINGS["depth_test_tile_modulus"] == 0
    radius = SETTINGS["depth_train_guard_px"]
    train = ~binary_dilation(test, structure=np.ones((2*radius+1, 2*radius+1), bool))
    return train, test


def holdout_exclusions(records, radius=10):
    pairs = sorted({tuple(sorted((int(r["a"]), int(r["b"]))))
                    for r in records if r.get("held_out", False)})
    nodes = {node for pair in pairs for center in pair
             for node in range(center-radius, center+radius+1)}
    return pairs, nodes


def choose_pairs(frames, submaps, poses, excluded, maximum=160):
    """Selection uses quality/time only, never candidate fit or test metrics."""
    run = {}
    for submap in submaps:
        for node in submap["node_ids"]:
            if node in run and run[node] != submap["run_id"]:
                raise ValueError("Node belongs to inconsistent quality runs")
            run[node] = submap["run_id"]
    candidates = []
    for a in sorted(frames):
        for span in SETTINGS["node_spans"]:
            b = a + span
            if not (SETTINGS["first_node"] <= a < b <= SETTINGS["last_node"]):
                continue
            interval = range(a, b+1)
            if any(n not in frames or n not in poses or n not in run or n in excluded
                   or frames[n]["quality_grade"] not in ("A", "B") for n in interval):
                continue
            if len({run[n] for n in interval}) != 1:
                continue
            elapsed = float(frames[b]["stamp"]) - float(frames[a]["stamp"])
            if not 0 < elapsed <= SETTINGS["maximum_elapsed_s"]:
                continue
            prior = inverse(poses[a]) @ poses[b]
            motion = delta_summary(np.eye(4), prior)
            if motion["translation_m"] > SETTINGS["maximum_initial_translation_m"]:
                continue
            if motion["rotation_deg"] > SETTINGS["maximum_initial_rotation_deg"]:
                continue
            candidates.append({"a": a, "b": b, "run_id": run[a], "span": span,
                               "elapsed_s": elapsed, "initial_motion": motion})
    # Uniform order statistics over chronological (node, span) sequence, without
    # choosing the largest/smallest errors after registration.
    indices = np.unique(np.rint(np.linspace(0, len(candidates)-1,
                            min(maximum, len(candidates)))).astype(int)) if candidates else []
    return [candidates[i] for i in indices], len(candidates)


def depth_cloud(session, node, keep):
    depth, confidence, cw, ch, k, local = session.depth(node)
    h, w = depth.shape
    stride = SETTINGS["cloud_pixel_stride"]
    vv, uu = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = (np.isfinite(z) & (z >= SETTINGS["depth_min_m"]) &
             (z <= SETTINGS["depth_max_m"]) &
             (confidence[::stride, ::stride] >= SETTINGS["confidence_min"]) &
             keep[::stride, ::stride])
    z = z[valid]
    optical = np.column_stack(((uu[valid]*cw/w-k[0, 2])*z/k[0, 0],
                               (vv[valid]*ch/h-k[1, 2])*z/k[1, 1], z))
    return reduce_voxels(apply(optical, local), SETTINGS["voxel_m"])


def feature_train_mask(session, node, rgb_pixels, train):
    image_h, image_w = session.image(node).shape[:2]
    h, w = train.shape
    depth, confidence, *_ = session.depth(node)
    valid_depth = (np.isfinite(depth) & (depth >= SETTINGS["depth_min_m"]) &
                   (depth <= SETTINGS["depth_max_m"]) & (confidence >= SETTINGS["confidence_min"]))
    pixels = np.rint(np.asarray(rgb_pixels)*[w/image_w, h/image_h]).astype(int)
    selected = np.zeros(len(pixels), bool)
    for i, (u, v) in enumerate(pixels):
        if 1 <= u < w-1 and 1 <= v < h-1:
            selected[i] = (train[v-1:v+2, u-1:u+2].all() and
                           valid_depth[v-1:v+2, u-1:u+2].all())
    return selected


def visual_seed(session, a, b, masks, prior):
    ka, da, _ = session.feature(a, full=True)
    kb, db, _ = session.feature(b, full=True)
    matches = mutual_ratio_matches(da, db, .75)
    result = {"mutual_root_sift_matches": len(matches), "usable": False,
              "rgb_scope": "RGB descriptors not held out; only depth sample partition is reserved"}
    if not matches:
        return None, result
    pa = np.asarray([ka[m.queryIdx].pt for m in matches])
    pb = np.asarray([kb[m.trainIdx].pt for m in matches])
    train = feature_train_mask(session, a, pa, masks[0]) & feature_train_mask(session, b, pb, masks[1])
    pa, pb = pa[train], pb[train]
    target, source = session.unproject(a, pa), session.unproject(b, pb)
    valid = (np.isfinite(target).all(axis=1) & np.isfinite(source).all(axis=1))
    target, source = target[valid], source[valid]
    result["depth_disjoint_training_matches"] = len(source)
    if len(source) < SETTINGS["minimum_visual_train_matches"]:
        return None, result
    fitted, inliers, trials = rigid_ransac(source, target, 20261017+a*10007+b,
        threshold=SETTINGS["visual_ransac_threshold_m"], iterations=SETTINGS["visual_ransac_iterations"])
    change = delta_summary(prior, fitted)
    result.update(train_inliers=int(inliers.sum()), train_inlier_ratio=float(inliers.mean()),
                  valid_trials=trials, prior_delta=change, T_a_b=fitted.tolist())
    result["usable"] = bool(inliers.sum() >= SETTINGS["minimum_visual_train_matches"]
        and inliers.mean() >= SETTINGS["minimum_visual_consensus_ratio"]
        and change["translation_m"] <= SETTINGS["visual_max_prior_delta_m"]
        and change["rotation_deg"] <= SETTINGS["visual_max_prior_delta_deg"])
    return (fitted if result["usable"] else None), result


def acceptance_reasons(before, after, fit, change, cycle):
    failures = []
    conditions = [
        (fit["rank"] >= SETTINGS["minimum_observability_rank"], "point_plane_rank_below_6"),
        (fit["minimum_eigenvalue_ratio"] >= SETTINGS["minimum_eigenvalue_ratio"], "point_plane_degenerate"),
        (after["overlap"] >= SETTINGS["minimum_train_overlap"], "train_overlap_below_45pct"),
        (after["rmse_m"] is not None and after["rmse_m"] <= SETTINGS["maximum_train_inlier_rmse_m"], "train_rmse_above_7cm"),
        (after["clipped_rmse_m"] <= before["clipped_rmse_m"] - SETTINGS["minimum_train_clipped_rmse_improvement_m"], "train_improvement_below_half_mm"),
        (change["translation_m"] <= SETTINGS["maximum_correction_translation_m"], "translation_correction_above_15cm"),
        (change["rotation_deg"] <= SETTINGS["maximum_correction_rotation_deg"], "rotation_correction_above_3deg"),
        (cycle["translation_m"] <= SETTINGS["maximum_bidirectional_cycle_m"], "cycle_translation_above_3cm"),
        (cycle["rotation_deg"] <= SETTINGS["maximum_bidirectional_cycle_deg"], "cycle_rotation_above_1deg"),
    ]
    for passed, reason in conditions:
        if not passed:
            failures.append(reason)
    return failures


def render_pair(path, target, source, prior, candidate, rotation, a, b, accepted):
    clouds = [target @ rotation.T, apply(source, prior) @ rotation.T,
              apply(source, candidate) @ rotation.T]
    lo, hi = np.percentile(np.vstack(clouds), [1, 99], axis=0)
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    for column, label in enumerate(["Frozen selected initial", "Local remeasurement"]):
        for row, dims in enumerate([(0, 1), (0, 2)]):
            ax = axes[row, column]
            for cloud, color, name in [(clouds[0], "#0077bb", "Frame A"),
                                       (clouds[column+1], "#ee7733", "Frame B")]:
                ax.scatter(cloud[:, dims[0]], cloud[:, dims[1]], s=1, c=color, alpha=.65, label=name)
            ax.set(xlim=(lo[dims[0]]-.2, hi[dims[0]]+.2), ylim=(lo[dims[1]]-.2, hi[dims[1]]+.2),
                   xlabel="X (m)", ylabel="Y (m)" if row == 0 else "Z (m)", title=label)
            ax.set_aspect("equal"); ax.grid(alpha=.2)
            if row == 0: ax.legend(markerscale=4)
    fig.suptitle(f"floor2 local {a} -> {b}; training-gate accepted={accepted}\n"
                "Same reserved depth tiles; not independent ground truth; no floor flattening")
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(path, dpi=140); plt.close(fig)


def remeasure(session, selection, poses, frames, directory, render=False):
    a, b = selection["a"], selection["b"]
    prior = inverse(poses[a]) @ poses[b]
    masks = [partition_masks(session.depth(n)[0].shape) for n in (a, b)]
    target, source = [depth_cloud(session, n, mask[0]) for n, mask in zip((a, b), masks)]
    record = dict(selection, accepted=False, T_a_b=prior.tolist(), initial_T_a_b=prior.tolist(),
                  primary_submap_ids=[frames[n]["primary_submap_id"] for n in (a, b)],
                  quality_grades=[frames[n]["quality_grade"] for n in (a, b)],
                  quality_scores=[float(frames[n]["quality_score"]) for n in (a, b)],
                  camera_height_m=[float(poses[n][2, 3]) for n in (a, b)],
                  classification="upstairs_temporal_interval_by_predeclared_node_range_not_floor_semantics",
                  train_points=[len(target), len(source)], reason="not_evaluated")
    record["depth_split"] = [{"shape": list(mask[0].shape), "guard_pixels": SETTINGS["depth_train_guard_px"],
        "train_mask_sha256": hashlib.sha256(np.packbits(mask[0]).tobytes()).hexdigest(),
        "test_mask_sha256": hashlib.sha256(np.packbits(mask[1]).tobytes()).hexdigest(),
        "train_depth_pixels": int(mask[0].sum()), "test_depth_pixels": int(mask[1].sum())} for mask in masks]
    record["correction_definition"] = "inv(initial_T_a_b) @ fitted_T_a_b; translation norm equals displacement of source frame origin, rotation geodesic angle"
    record.update(covariance_model(SETTINGS["model_sigma_translation_m"], SETTINGS["model_sigma_rotation_deg"]))
    if min(len(target), len(source)) < SETTINGS["minimum_cloud_points"]:
        record["reason"] = "too_few_train_cloud_points"
        return record
    visual, visual_data = visual_seed(session, a, b, [m[0] for m in masks], prior)
    before = geometric_score(target, source, prior, .2)
    seeds = {"temporal_initial": prior}
    if visual is not None:
        seeds["root_sift_rgbd_ransac"] = visual
    candidates = {}
    for name, seed in seeds.items():
        transform, fit = refine(target, source, seed, SETTINGS["point_plane_iterations"])
        candidates[name] = (transform, fit, geometric_score(target, source, transform, .2))
    name = min(candidates, key=lambda n: candidates[n][2]["clipped_rmse_m"])
    candidate, fit, after = candidates[name]
    reverse, reverse_fit = refine(source, target, inverse(candidate), SETTINGS["point_plane_iterations"])
    cycle = delta_summary(np.eye(4), candidate @ reverse)
    change = delta_summary(prior, candidate)
    failures = acceptance_reasons(before, after, fit, change, cycle)
    if reverse_fit["rank"] < 6:
        failures.append("reverse_point_plane_rank_below_6")
    record.update(accepted=not failures, reason=";".join(failures) or "train_only_bounded_rgbd_surface_pass",
        T_a_b=candidate.tolist(), visual=visual_data, selected_seed=name,
        training_candidates={key: {"geometry": value[2], "T_a_b": value[0].tolist()} for key, value in candidates.items()},
        fit={"before": before, "after": after, "point_plane": fit, "reverse_point_plane": reverse_fit,
             "cycle": cycle, "correction": change},
        place_identity="temporal_same_run_only_not_a_revisit_or_loop_assertion",
        acceptance_scope="training samples and fixed gates only; test not used in acceptance or seed selection")
    # No test arrays are constructed until the decision has been frozen above.
    test_target, test_source = [depth_cloud(session, n, mask[1]) for n, mask in zip((a, b), masks)]
    record["test"] = {"before": geometric_score(test_target, test_source, prior, .2),
                      "after": geometric_score(test_target, test_source, candidate, .2),
                      "points": [len(test_target), len(test_source)],
                      "scope": "diagnostic_only_depth_tile_holdout_correlated_sensor_samples_not_independent"}
    if render and min(len(test_target), len(test_source)) >= 30:
        path = directory/"figures"/f"local_{a}_{b}_reserved_depth.png"
        render_pair(path, test_target, test_source, prior, candidate, poses[a][:3, :3], a, b, record["accepted"])
        record["figure"] = str(path.resolve())
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, default=ROOT/"data/raw/rtabmap/261017-013501-2f.db")
    parser.add_argument("--quality", type=Path, default=ROOT/"outputs/student_union_20261017_submap_quality/floor2")
    parser.add_argument("--poses", type=Path, default=ROOT/"outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv")
    parser.add_argument("--old-constraints", type=Path, default=ROOT/"outputs/student_union_20261017_revalidated/intra_session/floor2/constraints.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-pairs", type=int, default=SETTINGS["maximum_pairs"])
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new output directory; no experiment is overwritten")
    if args.maximum_pairs <= 0:
        parser.error("maximum-pairs must be positive")
    cv2.setNumThreads(1)
    frames = {int(row["node_id"]): row for row in csv.DictReader((args.quality/"frames.csv").open())}
    submaps = json.loads((args.quality/"submaps.json").read_text())["submaps"]
    poses = load_pose_csv(args.poses)
    old = json.loads(args.old_constraints.read_text())["records"]
    pairs, excluded = holdout_exclusions(old, SETTINGS["whole_pair_holdout_neighbourhood_nodes"])
    chosen, candidates = choose_pairs(frames, submaps, poses, excluded, args.maximum_pairs)
    source_files = [Path(__file__), ROOT/"src/refine_geometry_surfaces.py", ROOT/"src/check_session_overlap.py",
                    ROOT/"src/revalidate_loop_constraints.py", ROOT/"src/evaluate_map_surfaces.py",
                    ROOT/"src/optimize_session.py", ROOT/"src/export_rtabmap_db.py"]
    inputs = [args.database, args.quality/"frames.csv", args.quality/"submaps.json", args.poses, args.old_constraints]
    source_hashes = {str(p.resolve()): sha256_file(p) for p in source_files}
    input_hashes = {str(p.resolve()): sha256_file(p) for p in inputs}
    (args.output/"figures").mkdir(parents=True)
    (args.output/"source_snapshot").mkdir()
    for path in source_files:
        (args.output/"source_snapshot"/path.name).write_bytes(path.read_bytes())
    selected_figures = {min(range(len(chosen)), key=lambda i: abs(chosen[i]["a"]-center))
                        for center in (320, 550, 1000)} if chosen else set()
    report = {"schema_version": 1, "complete": False, "created_utc": utc_now(), "session": "floor2",
        "command": shlex.join(sys.argv), "settings": dict(SETTINGS, maximum_pairs=args.maximum_pairs),
        "coordinate_convention": "T_a_b maps frame b local points into frame a local points",
        "source_hashes": source_hashes, "input_hashes": input_hashes,
        "whole_pair_exclusion": {"reserved_pairs": pairs, "radius_nodes": SETTINGS["whole_pair_holdout_neighbourhood_nodes"],
             "excluded_node_ids": sorted(excluded), "scope": "Both endpoints and every intermediate selected node excluded; original pose/old factors can still correlate with holdouts"},
        "selection": {"eligible_pair_count": candidates, "selected_pairs": chosen,
             "policy": "Even chronological order statistics; fixed node/time/quality/prior gates before measurement; same quality run; no C frame crossed",
             "rendered_selection_indices": sorted(selected_figures)},
        "limitations": ["No semantic dynamic mask; moving people may remain", "RGB descriptor texture is not held out; only depth samples are",
            "Sequential local data only; no new distant loop identity", "Camera z is not measured floor height",
            "No bundle adjustment; independently remeasured local frame edges for downstream frame-level PGO",
            "Neither covariance nor test values imply surveyed accuracy"], "records": []}
    (args.output/"selection.json").write_text(json.dumps(report, indent=2)+"\n")
    session = Session(args.database, "floor2")
    try:
        for index, selected in enumerate(chosen):
            record = remeasure(session, selected, poses, frames, args.output, index in selected_figures)
            report["records"].append(record)
            print(f"{index+1}/{len(chosen)} {record['a']}->{record['b']}: {record['accepted']} {record['reason']}", flush=True)
            with (args.output/"records.jsonl").open("a") as stream:
                stream.write(json.dumps(record)+"\n")
            session.geometry.clear()
    finally:
        session.connection.close()
    report["accepted_count"] = sum(r["accepted"] for r in report["records"])
    report["rejection_counts"] = dict(Counter(reason for r in report["records"] if not r["accepted"] for reason in r["reason"].split(";")))
    report["inputs_unchanged"] = all(sha256_file(Path(path)) == digest for path, digest in input_hashes.items())
    if not report["inputs_unchanged"]:
        raise RuntimeError("Input changed during experiment; report not complete")
    report["complete"] = True
    report["finished_utc"] = utc_now()
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"accepted": report["accepted_count"], "total": len(report["records"]),
                      "rejected": report["rejection_counts"], "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
