#!/usr/bin/env python3
"""Read-only v2 diagnostics: frozen historical patches and withheld RGB-D windows.

Whole-frame withholding concerns the NEW revisit estimator only. Existing
odometry, old loop selection and shared sensor errors prevent independence.
No ICP, alignment fit or candidate-dependent surface selection is performed.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import evaluate_floor2_local as old
from evaluate_map_surfaces import pair_metrics
from evaluate_submap_candidates import fixed_patches, ensure_fresh_output, write_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now
from refine_geometry_surfaces import SPECS


def support_metrics(a, b, radius=.20, clip=.30, normals_a=None, normals_b=None):
    """Symmetric finite support, including failures rather than inliers alone."""
    if min(len(a), len(b)) == 0:
        return {"available": False, "reason": "empty_input", "count_a": len(a), "count_b": len(b)}
    da, ia = cKDTree(b).query(a, workers=1)
    db, ib = cKDTree(a).query(b, workers=1)
    oa, ob = float(np.mean(da < radius)), float(np.mean(db < radius))
    result = {
        "available": True, "count_a": len(a), "count_b": len(b),
        "support_radius_m": radius, "clip_m": clip,
        "a_within_radius_fraction": oa, "b_within_radius_fraction": ob,
        "minimum_bidirectional_support_fraction": min(oa, ob),
        "symmetric_clipped_rmse_m": float(np.sqrt((np.mean(np.minimum(da, clip)**2)
                                                     + np.mean(np.minimum(db, clip)**2)) / 2)),
        "a_nn_median_m": float(np.median(da)), "b_nn_median_m": float(np.median(db)),
        "a_nn_p90_m": float(np.percentile(da, 90)), "b_nn_p90_m": float(np.percentile(db, 90)),
    }
    if normals_a is not None and normals_b is not None:
        angles = []
        counts = []
        for source, target, dist, index in [(normals_a, normals_b, da, ia),
                                             (normals_b, normals_a, db, ib)]:
            valid = (dist < radius) & np.all(np.isfinite(source), axis=1) & np.all(np.isfinite(target[index]), axis=1)
            dots = np.abs(np.sum(source[valid] * target[index[valid]], axis=1))
            angles.extend(np.degrees(np.arccos(np.clip(dots, 0., 1.))).tolist())
            counts.append(int(np.count_nonzero(valid)))
        result.update(normal_correspondence_count_by_direction=counts,
                      normal_difference_median_degrees=float(np.median(angles)) if angles else None,
                      normal_difference_p90_degrees=float(np.percentile(angles, 90)) if angles else None)
    return result


def local_normals(points, neighbors=20, radius=.25):
    """PCA normals in each raw frame; invalid/edge neighborhoods stay NaN."""
    result = np.full_like(points, np.nan)
    if len(points) < 6:
        return result
    distances, indices = cKDTree(points).query(points, k=min(neighbors, len(points)), workers=1)
    for i, (d, idx) in enumerate(zip(distances, indices)):
        xyz = points[idx[d <= radius]]
        if len(xyz) < 6:
            continue
        xyz = xyz - xyz.mean(axis=0)
        values, vectors = np.linalg.eigh(xyz.T @ xyz / len(xyz))
        if values[1] > 1e-8 and values[0] / max(values.sum(), 1e-12) <= .1:
            result[i] = vectors[:, 0]
    return result


def read_selection(path):
    selection = json.loads(path.read_text())
    # Producer schema is deliberately small and explicitly identifies both visits.
    windows = selection.get("validation_windows")
    if not windows:
        raise ValueError("selection requires validation_windows")
    all_train = set(selection["training_node_ids"])
    all_test = set()
    for window in windows:
        for key in ("visit_a", "visit_b"):
            ids = window[key]
            if not ids or len(ids) != len(set(ids)):
                raise ValueError("empty or duplicated validation window")
            all_test.update(ids)
    if all_train & all_test:
        raise ValueError("training and whole-frame validation overlap")
    return selection


def check_ba_selection(report_path, selection_path):
    report = json.loads(report_path.read_text())
    if not report.get("complete"):
        raise ValueError("BA producer report is incomplete")
    if report.get("selection_sha256") != sha256_file(selection_path):
        raise ValueError("BA producer selection hash mismatch")


def graph_binding(trajectory, graph_path=None, report_path=None):
    result = {"verified": False, "meaning": "hash linkage only, not independent solver verification"}
    if graph_path is None or report_path is None:
        result["reason"] = "graph or producing report not supplied"
        return result
    report = json.loads(report_path.read_text())
    expected_pose = report.get("trajectory_sha256")
    if trajectory.name == "trajectory_AB.csv":
        expected_pose = report.get("trajectory_AB_sha256", expected_pose)
    expected_graph = report.get("graph_manifest_sha256")
    actual_pose, actual_graph = sha256_file(trajectory), sha256_file(graph_path)
    result.update(trajectory_sha256=actual_pose, graph_sha256=actual_graph,
                  report_path=str(report_path), report_sha256=sha256_file(report_path),
                  graph_path=str(graph_path), expected_trajectory_sha256=expected_pose,
                  expected_graph_sha256=expected_graph)
    if expected_pose is not None and expected_pose != actual_pose:
        raise ValueError("producing report trajectory hash mismatch")
    if expected_graph is not None and expected_graph != actual_graph:
        raise ValueError("producing report graph hash mismatch")
    result["verified"] = expected_pose == actual_pose and expected_graph == actual_graph
    if not result["verified"]:
        result["reason"] = "producer does not record both expected hashes"
    return result


def frame_window_metrics(clouds, poses, selection):
    """Same raw point inventory per window, never shrink to a candidate's support."""
    nodes = sorted({n for w in selection["validation_windows"] for k in ("visit_a", "visit_b") for n in w[k]})
    cache = {}
    for node in nodes:
        pts = np.vstack(clouds.get(node))
        pts = pts[::max(1, math.ceil(len(pts) / 2500))]
        cache[node] = pts, local_normals(pts)
    records, payloads = [], {}
    for window in selection["validation_windows"]:
        for label, trajectory in poses.items():
            missing = sorted((set(window["visit_a"]) | set(window["visit_b"])) - set(trajectory))
            row = {"window": window["name"], "stage": label, "missing_node_ids": missing,
                   "classification": "whole_frames_withheld_from_new_revisit_fit_not_independent_truth"}
            if missing:
                row.update(available=False, reason="missing_pose_no_imputation")
            else:
                sides = []
                for key in ("visit_a", "visit_b"):
                    points = np.vstack([apply(cache[n][0], trajectory[n]) for n in window[key]])
                    normals = np.vstack([cache[n][1] @ trajectory[n][:3, :3].T for n in window[key]])
                    sides.append((points, normals))
                row.update(support_metrics(sides[0][0], sides[1][0], normals_a=sides[0][1], normals_b=sides[1][1]))
                payloads[window["name"], label] = sides[0][0], sides[1][0]
            records.append(row)
    return records, payloads, {str(n): {"point_count": len(p), "valid_normal_count": int(np.sum(np.all(np.isfinite(v), axis=1)))}
                               for n, (p, v) in cache.items()}


def window_observability(records):
    """Descriptive warning, never a tuned accept/reject gate for registration."""
    result = {}
    for name in dict.fromkeys(row["window"] for row in records):
        baselines = [r for r in records if r["window"] == name and r["stage"] in old.BASELINES and r.get("available")]
        maximum = max((r["minimum_bidirectional_support_fraction"] for r in baselines), default=None)
        result[name] = {
            "historical_baseline_maximum_minimum_bidirectional_support": maximum,
            "descriptive_low_support_threshold": .20,
            "status": "low_overlap_inconclusive" if maximum is None or maximum < .20 else "overlap_present_identity_not_proven",
            "threshold_status": "posthoc descriptive warning only; not a fitted gate or navigation criterion",
            "meaning": "Common visible footprint is not established. Whole-cloud NN mixes pose error, FOV, occlusion, camera motion and depth holes; it cannot alone accept or reject BA.",
        }
    return result


def augment_historical(connection, rows, paths, poses, raw, selection):
    clouds = Clouds(connection, None)
    nodes = sorted({r[k] for r in rows for k in ("a", "b")})
    cache = {n: fixed_patches(clouds, raw, n) for n in nodes}
    train = set(selection["training_node_ids"])
    metrics = []
    for row in rows:
        a, b = row["a"], row["b"]
        row["v2_role"] = "historical_development_touches_new_training" if {a, b} & train else "historical_repeated_untargeted_diagnostic"
        pa, pb = cache[a][row["kind"], row["plane_a"]], cache[b][row["kind"], row["plane_b"]]
        xa, xb = np.vstack((pa["training"], pa["testing"])), np.vstack((pb["training"], pb["testing"]))
        for label, trajectory in poses.items():
            if a not in trajectory or b not in trajectory:
                metrics.append({"patch_id": old.patch_id(row), "stage": label, "available": False})
                continue
            transform = inverse(trajectory[a]) @ trajectory[b]
            m = support_metrics(xa, apply(xb, transform))
            m.update(pair_metrics(pa, pb, trajectory[a], trajectory[b]))
            metrics.append({"patch_id": old.patch_id(row), "stage": label, "role": row["v2_role"], **m})
    return metrics


def plot_patch_grid(path, patch_a, patch_b, transforms, metrics, title):
    """Readable stage grid with identical axes; one dominant tangent view."""
    _, _, vectors = np.linalg.svd(patch_a["training"] - patch_a["center"], full_matrices=False)
    basis = np.vstack((vectors[0], patch_a["normal"]))
    target = (patch_a["testing"] - patch_a["center"]) @ basis.T
    moved = {s: (apply(patch_b["testing"], t) - patch_a["center"]) @ basis.T for s, t in transforms.items()}
    total = np.vstack([target, *moved.values()])
    lo, hi = total.min(axis=0), total.max(axis=0)
    pad = np.maximum(.03, .04 * (hi - lo))
    fig, axes = plt.subplots(math.ceil(len(moved) / 3), 3, figsize=(16, 4 * math.ceil(len(moved) / 3)), squeeze=False)
    for axis, (stage, points) in zip(axes.flat, moved.items()):
        axis.scatter(target[:, 0], target[:, 1], s=3, c="#147ba7", alpha=.5, label="visit A")
        axis.scatter(points[:, 0], points[:, 1], s=3, c="#dc702c", alpha=.5, label="visit B")
        metric = metrics[stage]
        axis.set(title=f"{stage}\nplane distance {metric['symmetric_plane_abs_median_m']*100:.1f} cm; "
                       f"normals {metric['normal_difference_degrees']:.1f} deg",
                 xlim=(lo[0]-pad[0], hi[0]+pad[0]), ylim=(lo[1]-pad[1], hi[1]+pad[1]),
                 xlabel="dominant patch tangent (m)", ylabel="signed normal offset (m)")
        axis.grid(alpha=.2)
    for axis in list(axes.flat)[len(moved):]:
        axis.axis("off")
    axes.flat[0].legend(fontsize=8)
    fig.suptitle(title + "\nSame original patch samples / axes; dominant tangent view. Finite support is reported separately.")
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_historical_summary(path, common):
    stages = list(common["metrics"])
    fig, axes = plt.subplots(2, 1, figsize=(15, 9), gridspec_kw={"height_ratios": [2, 1]})
    x = np.arange(len(stages))
    for offset, statistic, color in [(-.17, "median_m", "#3978ac"), (.17, "p90_m", "#e29532")]:
        values = [common["metrics"][stage][statistic] for stage in stages]
        axes[0].bar(x + offset, [v * 100 if v is not None else np.nan for v in values], .34, color=color, label=statistic)
    axes[0].set(xticks=x, xticklabels=[s.replace("_", "\n") for s in stages], ylabel="distance to corresponding plane (cm)")
    axes[0].grid(axis="y", alpha=.2)
    axes[0].legend()
    candidates = list(common["severe_regressions_by_candidate"])
    axes[1].bar(range(len(candidates)), [common["severe_regressions_by_candidate"][s]["per_patch_best_baseline"] for s in candidates], color="#b84c42")
    axes[1].set(xticks=range(len(candidates)), xticklabels=[s.replace("_", "\n") for s in candidates],
                ylabel="regressions >5 cm (count)", title="Against best available historical baseline per same patch")
    axes[1].grid(axis="y", alpha=.2)
    fig.suptitle(f"Same {common['matched_count']}/{common['historical_patch_count']} historical patches at every stage\n"
                 "Repeated DEVELOPMENT diagnostics; plane distance alone does not certify finite overlap")
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def height_diagnostics(aligned):
    reference = aligned["selected_initial"]
    result = {}
    for label, poses in aligned.items():
        shared = sorted(set(poses) & set(reference))
        change = np.array([poses[n][2, 3] - reference[n][2, 3] for n in shared])
        pairs = [(a, b) for a, b in zip(shared[:-1], shared[1:]) if b == a + 1]
        local_change = [(poses[b][2, 3] - poses[a][2, 3]) - (reference[b][2, 3] - reference[a][2, 3]) for a, b in pairs]
        result[label] = {"matched_frame_count": len(shared), "matched_adjacent_count": len(pairs),
                         "camera_z_change_abs_median_m": float(np.median(np.abs(change))),
                         "camera_z_change_abs_p90_m": float(np.percentile(np.abs(change), 90)),
                         "camera_z_change_abs_max_m": float(np.max(np.abs(change))),
                         "adjacent_delta_z_change_abs_p90_m": float(np.percentile(np.abs(local_change), 90)) if pairs else None,
                         "meaning": "pose change from selected initial in frame-1 gauge; not surveyed floor or stair error"}
    return result


def plot_windows(directory, records, payloads, stages):
    figures = []
    for name in dict.fromkeys(r["window"] for r in records):
        available = [s for s in stages if (name, s) in payloads]
        total = np.vstack([p for s in available for p in payloads[name, s]])
        lo, hi = total.min(axis=0), total.max(axis=0)
        pad = np.maximum(.1, (hi - lo) * .03)
        fig, axes = plt.subplots(math.ceil(len(stages) / 3), 3, figsize=(16, 5.2 * math.ceil(len(stages) / 3)), squeeze=False)
        for axis, stage in zip(axes.flat, stages):
            if (name, stage) not in payloads:
                axis.text(.5, .5, f"{stage}\nmissing pose; not scored", ha="center")
                axis.axis("off")
                continue
            a, b = payloads[name, stage]
            axis.scatter(a[:, 0], a[:, 1], s=.7, c="#147ba7", alpha=.4, label="withheld visit A")
            axis.scatter(b[:, 0], b[:, 1], s=.7, c="#dc702c", alpha=.4, label="withheld visit B")
            metric = next(r for r in records if r["window"] == name and r["stage"] == stage)
            axis.set(title=f"{stage}\nmin overlap {metric['minimum_bidirectional_support_fraction']:.1%}; "
                           f"clipped RMSE {metric['symmetric_clipped_rmse_m'] * 100:.1f} cm",
                     xlim=(lo[0]-pad[0], hi[0]+pad[0]), ylim=(lo[1]-pad[1], hi[1]+pad[1]),
                     xlabel="X (m)", ylabel="Y (m)", aspect="equal")
            axis.grid(alpha=.15)
        for axis in list(axes.flat)[len(stages):]:
            axis.axis("off")
        axes.flat[0].legend(fontsize=8)
        warning = window_observability(records)[name]["status"].replace("_", " ")
        fig.suptitle(f"{name}: fixed withheld whole-frame windows | {warning}\n"
                     "Common visible footprint unverified; FOV / occlusion / motion affect NN. Not a BA acceptance gate.")
        fig.tight_layout(rect=(0, 0, 1, .94))
        target = directory / f"withheld_{name}_xy.png"
        fig.savefig(target, dpi=150)
        plt.close(fig)
        figures.append(target.name)
    return figures


def plot_window_summary(path, records, stages):
    windows = list(dict.fromkeys(r["window"] for r in records))
    fig, axes = plt.subplots(len(windows), 2, figsize=(16, 4 * len(windows)), squeeze=False)
    for row, name in enumerate(windows):
        by_stage = {r["stage"]: r for r in records if r["window"] == name}
        for col, (key, multiplier, ylabel) in enumerate([
            ("minimum_bidirectional_support_fraction", 100, "minimum support within 20 cm (%)"),
            ("symmetric_clipped_rmse_m", 100, "symmetric NN RMSE clipped at 30 cm (cm)")]):
            values = [by_stage[s].get(key, np.nan) * multiplier for s in stages]
            axes[row, col].bar(range(len(stages)), values, color="#3978ac" if col == 0 else "#e29532")
            axes[row, col].set(xticks=range(len(stages)), xticklabels=[s.replace("_", "\n") for s in stages],
                               ylabel=ylabel, title=name)
            axes[row, col].tick_params(axis="x", labelsize=7)
            axes[row, col].grid(axis="y", alpha=.2)
    warnings = window_observability(records)
    warning = "LOW OVERLAP / INCONCLUSIVE" if all(v["status"] == "low_overlap_inconclusive" for v in warnings.values()) else "common visible footprint unverified"
    fig.suptitle(f"Whole-frame windows frozen before new fitting; {warning}\n"
                 "Common visible footprint unverified; these whole-cloud NN metrics cannot alone accept or reject BA")
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(path, dpi=160)
    plt.close(fig)


def withheld_rgb(connection, directory, selection):
    for window in selection["validation_windows"]:
        fig, axes = plt.subplots(2, max(len(window["visit_a"]), len(window["visit_b"])), figsize=(16, 9), squeeze=False)
        for row, side in enumerate(("visit_a", "visit_b")):
            for col, node in enumerate(window[side]):
                blob = connection.execute("SELECT image FROM Data WHERE id=?", (node,)).fetchone()
                img = cv2.imdecode(np.frombuffer(blob[0], np.uint8), cv2.IMREAD_COLOR) if blob and blob[0] else None
                if img is not None:
                    axes[row, col].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                axes[row, col].set_title(f"{side} | withheld frame {node}")
                axes[row, col].axis("off")
        fig.suptitle(f"{window['name']}: original RGB from frames excluded from new revisit fitting")
        fig.tight_layout(rect=(0, 0, 1, .96))
        fig.savefig(directory / f"withheld_{window['name']}_original_rgb.png", dpi=140)
        plt.close(fig)


def render_ba_training(connection, directory, report_path, reference, reference_path):
    """Actual per-frame BA poses, explicitly distinct from rigid ICP hypotheses."""
    producer = json.loads(report_path.read_text())
    expected = producer.get("input_hashes", {}).get(str(reference_path.resolve()))
    if expected != sha256_file(reference_path):
        raise ValueError("BA producer does not bind the selected initial trajectory")
    clouds = Clouds(connection, None)
    diagnostics = []
    for record in producer["records"]:
        bundle = record.get("bundle", {})
        if not bundle.get("ba_executed"):
            continue
        chosen = next(c for c in bundle["candidates"] if c["name"] == bundle["chosen"])
        a, b = record["a"], record["b"]
        frames = sorted({n for visit in record["train_frames"] for n in visit})
        supported = {int(observation[0]) for track in bundle["tracks"] for observation in track}
        refined = {int(n): reference[a] @ np.asarray(pose) for n, pose in chosen["poses"].items()}
        if set(frames) - set(refined):
            raise ValueError("BA candidate omitted a selected training frame")
        local = {}
        for n in frames:
            pts = np.vstack(clouds.get(n))
            local[n] = pts[::max(1, math.ceil(len(pts) / 1800))]
        for scope, keep in [("all_pose_variables", set(frames)), ("visual_observations_only", supported)]:
            clouds_by_stage = {}
            for stage, poses in [("selected_initial", reference), ("local_RGBD_BA", refined)]:
                clouds_by_stage[stage] = [np.vstack([apply(local[n], poses[n]) for n in visit if n in keep])
                                         for visit in record["train_frames"]]
            total = np.vstack([pts for sides in clouds_by_stage.values() for pts in sides])
            lo, hi = total.min(axis=0), total.max(axis=0)
            pad = np.maximum(.1, .03 * (hi - lo))
            fig, axes = plt.subplots(2, 2, figsize=(13, 11))
            scores = {}
            for col, (stage, sides) in enumerate(clouds_by_stage.items()):
                scores[stage] = support_metrics(*sides)
                for row, dimensions in enumerate(((0, 1), (0, 2))):
                    axis = axes[row, col]
                    for points, color, label in zip(sides, ("#147ba7", "#dc702c"), ("visit A", "visit B")):
                        axis.scatter(points[:, dimensions[0]], points[:, dimensions[1]], s=.5, c=color, alpha=.4, label=label)
                    d0, d1 = dimensions
                    axis.set(title=f"{stage}: {len(keep)} frames | {sum(map(len, sides)):,} original points",
                             xlim=(lo[d0]-pad[d0], hi[d0]+pad[d0]), ylim=(lo[d1]-pad[d1], hi[d1]+pad[d1]),
                             xlabel=f"{'XYZ'[d0]} (m)", ylabel=f"{'XYZ'[d1]} (m)", aspect="equal")
                    axis.grid(alpha=.15)
            axes[0, 0].legend(fontsize=8)
            fig.suptitle(f"{a}/{b}: actual per-frame RGB-D bundle adjustment | {scope.replace('_', ' ')}\n"
                         f"Seed {bundle['chosen']}; {len(frames)} poses, {len(supported)} with visual tracks, {len(bundle['tracks'])} landmarks\n"
                         "TRAINING diagnostic; shared frame-a display gauge; includes initialization and BA effects, not rigid ICP")
            fig.tight_layout(rect=(0, 0, 1, .91))
            figure = directory / f"local_ba_{a}_{b}_{scope}.png"
            fig.savefig(figure, dpi=160)
            plt.close(fig)
            diagnostics.append({"a": a, "b": b, "scope": scope, "frame_ids": sorted(keep),
                                "all_variable_frames": frames, "visual_support_frames": sorted(supported),
                                "prior_only_frames": sorted(set(frames)-supported), "chosen_seed": bundle["chosen"],
                                "bundle_accepted": bundle.get("accepted"), "solver_success": chosen["success"],
                                "surface_diagnostics_training_only": scores, "figure": figure.name})
    return diagnostics


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", nargs=2, action="append", required=True)
    parser.add_argument("--graph-manifest", nargs=2, action="append", default=[])
    parser.add_argument("--binding-report", nargs=2, action="append", default=[])
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--ba-report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-stride", type=int, default=6)
    args = parser.parse_args(argv)
    if args.frame_stride < 1:
        parser.error("frame stride must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    output = args.output.resolve()
    ensure_fresh_output(output)
    selection = read_selection(args.selection)
    if args.ba_report:
        check_ba_selection(args.ba_report, args.selection)
    sources = [Path(__file__), Path(old.__file__), Path(__file__).with_name("evaluate_submap_candidates.py"),
               Path(__file__).with_name("evaluate_map_surfaces.py"), Path(__file__).with_name("optimize_session.py"),
               Path(__file__).with_name("export_rtabmap_db.py"), Path(__file__).with_name("refine_geometry_surfaces.py")]
    source_hashes = {p.name: sha256_file(p) for p in sources}
    paths, candidates = old.candidate_paths(args.trajectory)
    graphs = {label: Path(path).resolve() for label, path in args.graph_manifest}
    reports = {label: Path(path).resolve() for label, path in args.binding_report}
    if (set(graphs) | set(reports)) - set(paths):
        raise ValueError("graph/report labels must match a trajectory")
    bindings = {label: graph_binding(path, graphs.get(label), reports.get(label)) for label, path in paths.items()}
    input_paths = {"selection": args.selection.resolve(), **{f"trajectory:{k}": p for k, p in paths.items()},
                   **{f"graph:{k}": p for k, p in graphs.items()}, **{f"producer:{k}": p for k, p in reports.items()}}
    if args.ba_report:
        input_paths["ba_report"] = args.ba_report.resolve()
    input_hashes = {k: {"path": str(p), "sha256": sha256_file(p)} for k, p in input_paths.items()}
    rows, _, payloads, poses, hashes, database = old.evaluate(paths)
    aligned, gauge = old.display_align(poses)
    output.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=f".{output.name}.tmp-", dir=output.parent))
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        figures = work / "figures"
        figures.mkdir()
        raw = old.load_pose_csv(Path(hashes["raw_pose"]["path"]))
        historical_metrics = augment_historical(connection, rows, paths, poses, raw, selection)
        window_rows, window_payloads, sampled = frame_window_metrics(Clouds(connection, None), aligned, selection)
        common = old.all_stage_common_support(rows, candidates)
        summaries = {candidate: old.matched_support(rows, candidate) for candidate in candidates}
        plot_historical_summary(figures / "common_support_all_stages.png", common)
        old.plot_heights(figures / "trajectory_height_by_node.png", aligned, sorted(set(selection["training_node_ids"])))
        for (a, b, kind, pa, pb), payload in payloads.items():
            plot_patch_grid(figures / f"floor2_{a}_{b}_{kind}_{pa}_{pb}.png", *payload,
                           f"floor2 {a}/{b}: HISTORICAL DEVELOPMENT diagnostic; distance to patch plane")
        points, ids, bands, sampling = old.point_sample_views(connection, aligned, args.frame_stride)
        old.render_views(figures, points, ids, bands, sampling)
        old.rgb_montage(connection, figures / "original_rgb_critical_pairs.png")
        plot_windows(figures, window_rows, window_payloads, list(paths))
        plot_window_summary(figures / "whole_frame_validation.png", window_rows, list(paths))
        withheld_rgb(connection, figures, selection)
        ba_training = render_ba_training(connection, figures, args.ba_report, poses["selected_initial"],
                                          paths["selected_initial"]) if args.ba_report else []
        # Preserve every historical observation and quantify support separately.
        role_summary = {role: old.all_stage_common_support([r for r in rows if r["v2_role"] == role], candidates)
                        for role in sorted({r["v2_role"] for r in rows})}
        changed = [key for key, p in input_paths.items() if sha256_file(p) != input_hashes[key]["sha256"]]
        if changed or sha256_file(database) != hashes["database"]["sha256"]:
            raise RuntimeError(f"evaluation inputs changed: {changed}")
        if any(sha256_file(p) != source_hashes[p.name] for p in sources):
            raise RuntimeError("evaluation source changed while running")
        snapshots = work / "source_snapshot"
        snapshots.mkdir()
        for source in sources:
            shutil.copyfile(source, snapshots / source.name)
        report = {
            "schema_version": 2, "created_utc": utc_now(), "command": sys.argv, "accepted": False,
            "navigation": False, "status": "HISTORICAL_DEVELOPMENT_PLUS_CORRELATED_WHOLE_FRAME_DIAGNOSTICS",
            "input_hashes": input_hashes, "historical_inputs": hashes, "inputs_unchanged": True,
            "original_database_unchanged": True, "source_sha256": source_hashes, "source_unchanged": True,
            "graph_bindings": bindings, "validation_selection": selection,
            "historical_patch_count": len(rows), "historical_comparisons": rows,
            "historical_metric_rows": historical_metrics, "matched_support_by_candidate": summaries,
            "common_support_all_stages": common, "historical_role_summary": role_summary,
            "whole_frame_validation": window_rows, "whole_frame_sampling": sampled,
            "whole_frame_observability": window_observability(window_rows),
            "local_bundle_training_diagnostics": ba_training,
            "height_change_diagnostics": height_diagnostics(aligned),
            "display_gauge": gauge, "visual_sampling": sampling,
            "figures": [str(p.relative_to(work)) for p in sorted(figures.glob("*.png"))],
            "limitations": [
                "Historical61patches including656/1055 are repeatedly consulted development diagnostics; none are fresh truth.",
                "Whole-frame withholding applies only to the new revisit estimator; native trajectory factors and old loops may use these frames.",
                "Nearby heldout frames share motion/depth errors and do not establish independent map accuracy.",
                "Plane distance may hide tangential sliding; finite support, symmetric NN distances and normals are reported alongside it.",
                "Whole-window NN compares all observed geometry, so viewpoint/occlusion/dynamic differences contribute to residuals.",
                "Frozen windows have no established common visible footprint; low-overlap scores are inconclusive, not calibrated accuracy or BA pass/fail.",
                "Normal statistics are conditional on finite support and valid PCA neighborhoods; low support cannot be rescued by a small angle.",
                "All raw sample IDs and window membership are fixed; no validation ICP or candidate-based patch rematching is used.",
                "No pose imputation; missing complete windows are explicitly unavailable rather than evaluated on a reduced subset.",
                "Display gauge is one rigid frame anchor only; height bands are initial-pose proxies, not surveyed floor or stair heights.",
                "Hash binding links supplied artifacts; it does not prove the solver equations or convergence are correct.",
            ],
        }
        (work / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        write_csv(work / "historical_patch_metrics.csv", historical_metrics)
        write_csv(work / "whole_frame_validation.csv", window_rows)
        write_csv(work / "historical_comparisons.csv", [{"patch_id": old.patch_id(r), "role": r["v2_role"], **r["values"]} for r in rows])
        ensure_fresh_output(output)
        work.rename(output)
    except Exception:
        shutil.rmtree(work, ignore_errors=True)
        raise
    finally:
        connection.close()
    print(json.dumps({"output": str(output), "historical_patches": len(rows), "validation_windows": len(selection["validation_windows"]),
                      "figures": len(report["figures"]), "accepted": False}))


if __name__ == "__main__":
    main()
