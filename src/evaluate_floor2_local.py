#!/usr/bin/env python3
"""Floor-2 local candidates: frozen-patch diagnostics and actual RGB-D views.

This is repeated historical conditional validation, NOT surveyed truth. Plane
IDs and depth samples are reproduced once from the historical inventory and
never re-selected or refitted after seeing a candidate. No DB or RViz writes.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np

from evaluate_submap_candidates import (
    CRITICAL_PAIRS, HISTORICAL_ROOT, ROOT, fixed_patches, pair_graph_provenance,
    plot_patch, ensure_fresh_output, write_csv,
)
from evaluate_map_surfaces import pair_metrics
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now
from refine_geometry_surfaces import SPECS

BASELINES = {
    "previous_preview": ROOT / "outputs/student_union_20261017_all_preview/bundle/floor2_trajectory.csv",
    "selected_initial": ROOT / "outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv",
    "integrated_dcs": ROOT / "outputs/student_union_20261017_submap_robust/expanded_extended_v7/dcs/floor2/trajectory.csv",
}
SEVERE_M = .05
CRITICAL = CRITICAL_PAIRS["floor2"]
TIME_EDGES = np.array([0, 450, 700, 900, 1300])
TIME_COLORS = ["#4575b4", "#1a9850", "#fdae61", "#d73027"]


def patch_id(row):
    return f"{row['a']}/{row['b']}:{row['kind']}:{row['plane_a']}/{row['plane_b']}"


def distribution(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values),
            "median_m": float(np.median(values)) if len(values) else None,
            "p90_m": float(np.percentile(values, 90)) if len(values) else None,
            "maximum_m": float(np.max(values)) if len(values) else None}


def matched_support(rows, candidate, baseline_names=tuple(BASELINES)):
    """Never compare a filtered candidate against a different baseline subset."""
    available = [r for r in rows if r["values"].get(candidate) is not None]
    result = {
        "historical_patch_count": len(rows), "candidate_available_count": len(available),
        "candidate_missing_count": len(rows) - len(available),
        "candidate_coverage_fraction": len(available) / len(rows) if rows else 0.,
        "candidate_missing_patch_ids": [patch_id(r) for r in rows if r["values"].get(candidate) is None],
        "candidate_distribution_all_available": distribution([r["values"][candidate] for r in available]),
        "per_baseline": {},
    }
    for baseline in baseline_names:
        support = [r for r in available if r["values"].get(baseline) is not None]
        severe = [patch_id(r) for r in support if r["values"][candidate] - r["values"][baseline] > SEVERE_M]
        result["per_baseline"][baseline] = {
            "matched_count": len(support), "patch_ids": [patch_id(r) for r in support],
            "candidate": distribution([r["values"][candidate] for r in support]),
            "baseline": distribution([r["values"][baseline] for r in support]),
            "severe_regression_count": len(severe), "severe_patch_ids": severe,
        }
    common = [r for r in available if all(r["values"].get(b) is not None for b in baseline_names)]
    result["common_all_baselines"] = {
        "matched_count": len(common), "patch_ids": [patch_id(r) for r in common],
        "metrics": {name: distribution([r["values"][name] for r in common])
                    for name in [*baseline_names, candidate]},
    }
    best_support = [r for r in available if any(r["values"].get(b) is not None for b in baseline_names)]
    best = [min(r["values"][b] for b in baseline_names if r["values"].get(b) is not None)
            for r in best_support]
    severe = [patch_id(r) for r, b in zip(best_support, best) if r["values"][candidate] - b > SEVERE_M]
    result["per_patch_best_available_baseline"] = {
        "matched_count": len(best_support), "baseline": distribution(best),
        "candidate": distribution([r["values"][candidate] for r in best_support]),
        "severe_regression_count": len(severe), "severe_patch_ids": severe,
        "baseline_available_count_by_patch": {
            patch_id(r): sum(r["values"].get(b) is not None for b in baseline_names) for r in best_support},
    }
    return result


def all_stage_common_support(rows, candidates, baseline_names=tuple(BASELINES)):
    """One common patch inventory across every candidate and baseline."""
    stages = [*baseline_names, *candidates]
    common = [r for r in rows if all(r["values"].get(stage) is not None for stage in stages)]
    return {
        "matched_count": len(common), "historical_patch_count": len(rows),
        "patch_ids": [patch_id(r) for r in common],
        "excluded_patch_ids": [patch_id(r) for r in rows
                               if any(r["values"].get(stage) is None for stage in stages)],
        "metrics": {stage: distribution([r["values"][stage] for r in common]) for stage in stages},
        "severe_regressions_by_candidate": {
            candidate: {
                **{baseline: sum(r["values"][candidate] - r["values"][baseline] > SEVERE_M for r in common)
                   for baseline in baseline_names},
                "per_patch_best_baseline": sum(
                    r["values"][candidate] - min(r["values"][b] for b in baseline_names) > SEVERE_M
                    for r in common),
            } for candidate in candidates
        },
    }


def provenance(graph, a, b):
    if graph is None:
        return {"graph_partition_status": "not_provided", "graph_edge_ids": [],
                "independent_ground_truth": False,
                "graph_correlation_reason": "training graph not supplied; no independence claim"}
    return pair_graph_provenance(graph, "floor2", a, b)


def display_align(trajectories, reference="selected_initial"):
    """Single rigid gauge alignment, for plots only; never deform a candidate."""
    shared = sorted(set.intersection(*(set(p) for p in trajectories.values())))
    if not shared:
        raise ValueError("no common pose to fix a display gauge")
    anchor = shared[0]
    transforms = {label: trajectories[reference][anchor] @ inverse(poses[anchor])
                  for label, poses in trajectories.items()}
    aligned = {label: {n: transforms[label] @ pose for n, pose in poses.items()}
               for label, poses in trajectories.items()}
    return aligned, {"reference": reference, "shared_anchor_node": anchor,
                     "transform_by_stage": {k: v.tolist() for k, v in transforms.items()},
                     "used_for_metrics": False}


def candidate_paths(entries):
    paths = dict(BASELINES)
    candidates = []
    for label, value in entries:
        if label in paths:
            raise ValueError(f"duplicate or reserved trajectory label: {label}")
        paths[label] = Path(value).resolve()
        candidates.append(label)
    if not candidates:
        raise ValueError("at least one candidate trajectory is required")
    for path in paths.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    return paths, candidates


def checked_trajectory(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    ids = [int(row["id"]) for row in rows]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate pose IDs: {path}")
    poses = load_pose_csv(path)
    if not poses:
        raise ValueError(f"empty trajectory: {path}")
    for node, pose in poses.items():
        if not np.all(np.isfinite(pose)):
            raise ValueError(f"nonfinite pose: {path}/{node}")
        if not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6):
            raise ValueError(f"invalid pose rotation: {path}/{node}")
    return poses


def evaluate(paths, graph_path=None):
    database = ROOT / "data/raw/rtabmap" / SPECS["floor2"][0]
    raw_path = ROOT / "outputs" / SPECS["floor2"][1] / "floor2/raw_main_trajectory.csv"
    app_path = ROOT / "outputs" / SPECS["floor2"][1] / "audit/floor2/app_optimized_trajectory.csv"
    historical_path = HISTORICAL_ROOT / SPECS["floor2"][2] / "floor2/plane_pair_metrics.csv"
    inputs = {"database": database, "raw_pose": raw_path, "app_pose": app_path,
              "historical_inventory": historical_path}
    if graph_path is not None:
        inputs["graph_manifest"] = graph_path
    hashes = {key: {"path": str(path.resolve()), "sha256": sha256_file(path)} for key, path in inputs.items()}
    graph = json.loads(graph_path.read_text()) if graph_path else None
    raw, app = load_pose_csv(raw_path), load_pose_csv(app_path)
    with historical_path.open(newline="") as stream:
        historical = [r for r in csv.DictReader(stream) if r["trajectory"] == "app"]
    # Candidate poses cannot influence patch extraction or correspondence choice.
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        clouds = Clouds(connection, None)
        nodes = sorted({int(r[k]) for r in historical for k in ("from_id", "to_id")})
        cache = {node: fixed_patches(clouds, raw, node) for node in nodes}
        poses = {label: checked_trajectory(path) for label, path in paths.items()}
        comparisons, metrics_rows, payloads = [], [], {}
        for historical_row in historical:
            a, b = int(historical_row["from_id"]), int(historical_row["to_id"])
            kind = historical_row["kind"]
            pa, pb = int(historical_row["from_plane"]), int(historical_row["to_plane"])
            fixed_a, fixed_b = cache[a][kind, pa], cache[b][kind, pb]
            reproduced = pair_metrics(fixed_a, fixed_b, app[a], app[b])["symmetric_plane_abs_median_m"]
            error = abs(reproduced - float(historical_row["symmetric_plane_abs_median_m"]))
            if error > 5e-5:
                raise ValueError(f"historical patch reproduction failed: {a}/{b} {kind}: {error}")
            row = {"a": a, "b": b, "kind": kind, "plane_a": pa, "plane_b": pb,
                   "historical_reproduction_error_m": error, **provenance(graph, a, b), "values": {}}
            transforms, stage_metrics = {}, {}
            for label, trajectory in poses.items():
                if a not in trajectory or b not in trajectory:
                    row["values"][label] = None
                    metrics_rows.append({"patch_id": patch_id(row), "stage": label, "available": False})
                    continue
                metric = pair_metrics(fixed_a, fixed_b, trajectory[a], trajectory[b])
                row["values"][label] = metric["symmetric_plane_abs_median_m"]
                stage_metrics[label] = metric
                transforms[label] = inverse(trajectory[a]) @ trajectory[b]
                metrics_rows.append({"patch_id": patch_id(row), "stage": label, "available": True, **metric})
            comparisons.append(row)
            if (a, b) in CRITICAL:
                payloads[(a, b, kind, pa, pb)] = (fixed_a, fixed_b, transforms, stage_metrics)
    finally:
        connection.close()
    return comparisons, metrics_rows, payloads, poses, hashes, database


def plot_matched_support(path, summaries):
    fig, axes = plt.subplots(len(summaries), 1, figsize=(12, 4.5 * len(summaries)), squeeze=False)
    for axis, (candidate, summary) in zip(axes.flat, summaries.items()):
        common = summary["common_all_baselines"]
        labels = list(common["metrics"])
        x = np.arange(len(labels))
        for offset, stat, color in [(-.17, "median_m", "#3978ac"), (.17, "p90_m", "#e29532")]:
            values = [common["metrics"][s][stat] for s in labels]
            axis.bar(x + offset, [v * 100 if v is not None else np.nan for v in values], .34,
                     label=stat.replace("_m", ""), color=color)
        axis.set(xticks=x, xticklabels=labels, ylabel="finite-patch separation (cm)",
                 title=f"{candidate}: same {common['matched_count']} patches for all bars; "
                 f"candidate coverage {summary['candidate_available_count']}/{summary['historical_patch_count']}")
        axis.grid(axis="y", alpha=.2)
        axis.legend()
    fig.suptitle("Repeated historical conditional validation, not surveyed ground truth", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_all_stage_common_support(path, summary):
    stages = list(summary["metrics"])
    candidates = list(summary["severe_regressions_by_candidate"])
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), gridspec_kw={"height_ratios": [2, 1]})
    x = np.arange(len(stages))
    for offset, statistic, color in [(-.17, "median_m", "#3978ac"), (.17, "p90_m", "#e29532")]:
        values = [summary["metrics"][s][statistic] for s in stages]
        axes[0].bar(x + offset, [100 * v if v is not None else np.nan for v in values], .34,
                    color=color, label=statistic.replace("_m", ""))
    axes[0].set(xticks=x, xticklabels=[s.replace("_", "\n") for s in stages],
                ylabel="finite-patch separation (cm)", title="One identical patch inventory for every stage")
    axes[0].grid(axis="y", alpha=.2)
    axes[0].legend()
    severe = [summary["severe_regressions_by_candidate"][s]["per_patch_best_baseline"] for s in candidates]
    axes[1].bar(candidates, severe, color="#b84c42")
    axes[1].set(ylabel="severe regressions (count)",
                title="More than 5 cm worse than the best baseline on that same patch")
    axes[1].grid(axis="y", alpha=.2)
    fig.suptitle(f"Common support: {summary['matched_count']}/{summary['historical_patch_count']} historical patches\n"
                 "Repeated conditional diagnostics, not surveyed truth; no navigation approval", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_heights(path, poses, critical_nodes):
    fig, axes = plt.subplots(len(poses), 1, figsize=(13, 2.5 * len(poses)), sharex=True, sharey=True)
    axes = np.atleast_1d(axes)
    for axis, (label, trajectory) in zip(axes, poses.items()):
        nodes = sorted(trajectory)
        heights = [trajectory[n][2, 3] for n in nodes]
        axis.scatter(nodes, heights, c=nodes, cmap="viridis", vmin=1, vmax=1250, s=4)
        for node in critical_nodes:
            if node in trajectory:
                axis.scatter([node], [trajectory[node][2, 3]], s=24, facecolors="none", edgecolors="#c61e28")
                axis.annotate(str(node), (node, trajectory[node][2, 3]), fontsize=6,
                              xytext=(0, 6), textcoords="offset points", rotation=45)
        axis.set(ylabel="camera Z (m)", title=f"{label}; {len(nodes)} poses; points only, no lines over gaps")
        axis.grid(alpha=.2)
    axes[-1].set_xlabel("frame/node ID (capture order; not elapsed seconds)")
    fig.suptitle("Camera-height diagnostic in a shared display gauge; no floor-height constraint applied")
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def common_sample_nodes(poses, stride):
    """Use identical input observations in all XY stages; expose lost support."""
    reference = sorted(poses["selected_initial"])[::stride]
    shared = [n for n in reference if all(n in p for p in poses.values())]
    return shared, {"reference_sampled_nodes": reference, "common_sampled_nodes": shared,
                    "excluded_by_missing_stage": {s: [n for n in reference if n not in p] for s, p in poses.items()}}


def point_sample_views(connection, poses, stride):
    nodes, sampling = common_sample_nodes(poses, stride)
    if not nodes:
        raise ValueError("no common RGB-D nodes to render")
    clouds = Clouds(connection, None)
    # Frozen reference camera-height thresholds, not surveyed floor labels.
    reference = poses["selected_initial"]
    anchor_heights = [reference[n][2, 3] for pair in CRITICAL for n in pair if n in reference]
    upper_center = float(np.median(anchor_heights))
    upper_cut = upper_center - .8
    lower_cut = min(p[2, 3] for p in reference.values()) + 1.5
    local, source_nodes = [], []
    for node in nodes:
        points = np.vstack(clouds.get(node))
        # Deterministic thin view of the actual depth data; no geometry invention.
        points = points[::max(1, int(np.ceil(len(points) / 1100)))]
        local.append(points)
        source_nodes.append(np.full(len(points), node, dtype=np.int32))
    ids = np.concatenate(source_nodes)
    band_by_node = {n: (0 if reference[n][2, 3] < lower_cut else
                        1 if reference[n][2, 3] < upper_cut else 2) for n in nodes}
    bands = np.concatenate([np.full(len(p), band_by_node[n], dtype=np.int8) for n, p in zip(nodes, local)])
    result = {label: np.vstack([apply(points, trajectory[node]) for node, points in zip(nodes, local)])
              for label, trajectory in poses.items()}
    sampling.update({"point_count_per_stage": len(ids), "max_local_points_per_frame": 1100,
                     "frame_stride": stride, "depth_range_m": [.25, 4.], "confidence_minimum": 50,
                     "voxel_m": .06, "pixel_stride": 2,
                     "camera_height_thresholds_m": [lower_cut, upper_cut],
                     "point_z_slice_m": [upper_center - .5, upper_center + .5],
                     "upper_camera_reference_median_m": upper_center,
                     "node_window_edges": TIME_EDGES.tolist(),
                     "selection": "identical selected-initial camera-height membership in every stage",
                     "not_surveyed_floor_labels": True})
    return result, ids, bands, sampling


def render_views(directory, points, node_ids, bands, sampling):
    stages = list(points)
    all_points = np.vstack(list(points.values()))
    lower, upper = np.min(all_points[:, :2], axis=0), np.max(all_points[:, :2], axis=0)
    pad = np.maximum(.3, (upper - lower) * .025)
    limits = dict(xlim=(lower[0] - pad[0], upper[0] + pad[0]),
                  ylim=(lower[1] - pad[1], upper[1] + pad[1]))
    figures = []
    cmap, norm = ListedColormap(TIME_COLORS), BoundaryNorm(TIME_EDGES, len(TIME_COLORS))
    for name, rows in [("camera_height_bands_xy.png", 3), ("upper_corridor_time_windows_xy.png", 2)]:
        fig, axes = plt.subplots(rows, len(stages), figsize=(4.4 * len(stages), 5 * rows), squeeze=False)
        for col, stage in enumerate(stages):
            xyz = points[stage]
            for row in range(rows):
                axis = axes[row, col]
                if rows == 3:
                    keep = bands == row
                    caption = ["lower camera band", "transition camera band", "upper camera band"][row]
                else:
                    keep = bands == 2
                    caption = "upper-camera observations, all point heights"
                    if row == 1:
                        lo, hi = sampling["point_z_slice_m"]
                        keep &= (xyz[:, 2] >= lo) & (xyz[:, 2] <= hi)
                        caption = f"upper cameras; point Z {lo:.2f} to {hi:.2f} m"
                axis.scatter(xyz[keep, 0], xyz[keep, 1], c=node_ids[keep], cmap=cmap, norm=norm,
                             s=.13, alpha=.6, rasterized=True)
                axis.set(**limits, xlabel="X (m)", ylabel="Y (m)",
                         aspect="equal")
                axis.set_title(f"{stage}\n{caption}\n{np.count_nonzero(keep):,} points", fontsize=8)
                axis.grid(alpha=.13)
        fig.suptitle("Actual original depth reprojected by candidate poses; same axes / input samples\n"
                     "Exploratory height/time slices, NOT surveyed floor labels; "
                     "blue IDs 1-449 | green 450-699 | orange 700-899 | red 900+", fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, .94))
        path = directory / name
        fig.savefig(path, dpi=140)
        plt.close(fig)
        figures.append(path.name)
    sampling["xy_axis_limits"] = {k: list(v) for k, v in limits.items()}
    return figures


def rgb_montage(connection, path):
    pairs = sorted(CRITICAL)
    fig, axes = plt.subplots(len(pairs), 2, figsize=(12, 3.2 * len(pairs)))
    records = []
    for row, (a, b) in enumerate(pairs):
        for col, node in enumerate((a, b)):
            blob = connection.execute("SELECT image FROM Data WHERE id=?", (node,)).fetchone()
            decoded = cv2.imdecode(np.frombuffer(blob[0], np.uint8), cv2.IMREAD_COLOR) if blob and blob[0] else None
            if decoded is not None:
                axes[row, col].imshow(cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB))
            else:
                axes[row, col].text(.5, .5, "missing image", ha="center")
            axes[row, col].set_title(f"original RGB node {node} | historical pair {a}/{b}")
            axes[row, col].axis("off")
            records.append({"node": node, "decoded": decoded is not None})
    fig.suptitle("Original source RGB: visual place review, not metric ground truth")
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return records


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", nargs=2, action="append", metavar=("LABEL", "PATH"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--graph-manifest", type=Path)
    parser.add_argument("--frame-stride", type=int, default=6)
    args = parser.parse_args(argv)
    if args.frame_stride < 1:
        parser.error("--frame-stride must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    output = args.output.resolve()
    ensure_fresh_output(output)
    paths, candidates = candidate_paths(args.trajectory)
    pose_hashes = {label: sha256_file(path) for label, path in paths.items()}
    rows, metric_rows, payloads, poses, hashes, database = evaluate(paths, args.graph_manifest)
    summaries = {candidate: matched_support(rows, candidate) for candidate in candidates}
    common_summary = all_stage_common_support(rows, candidates)
    aligned, gauge = display_align(poses)
    output.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=f".{output.name}.tmp-", dir=output.parent))
    try:
        figures = work / "figures"
        figures.mkdir()
        plot_matched_support(figures / "matched_support.png", summaries)
        plot_all_stage_common_support(figures / "common_support_all_stages.png", common_summary)
        plot_heights(figures / "trajectory_height_by_node.png", aligned,
                     sorted({n for pair in CRITICAL for n in pair}))
        for (a, b, kind, pa, pb), payload in payloads.items():
            missing = [label for label in poses if label not in payload[2]]
            plot_patch(figures / f"floor2_{a}_{b}_{kind}_{pa}_{pb}.png", *payload,
                       f"floor2 {a}/{b} {kind}: repeated historical conditional validation"
                       + (f"; missing endpoints: {', '.join(missing)}" if missing else ""))
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            points, ids, bands, sampling = point_sample_views(connection, aligned, args.frame_stride)
            render_views(figures, points, ids, bands, sampling)
            rgb = rgb_montage(connection, figures / "original_rgb_critical_pairs.png")
        finally:
            connection.close()
        after = sha256_file(database)
        if hashes["database"]["sha256"] != after:
            raise RuntimeError("original database changed during read-only evaluation")
        for label, path in paths.items():
            if sha256_file(path) != pose_hashes[label]:
                raise RuntimeError(f"trajectory changed during evaluation: {label}")
        sources = [Path(__file__), Path(__file__).with_name("evaluate_submap_candidates.py"),
                   Path(__file__).with_name("evaluate_map_surfaces.py"),
                   Path(__file__).with_name("optimize_session.py"),
                   Path(__file__).with_name("export_rtabmap_db.py"),
                   Path(__file__).with_name("refine_geometry_surfaces.py")]
        snapshot = work / "source_snapshot"
        snapshot.mkdir()
        for source in sources:
            shutil.copyfile(source, snapshot / source.name)
        report = {
            "schema_version": 1, "created_utc": utc_now(), "command": sys.argv,
            "status": "REPEATED_HISTORICAL_CONDITIONAL_DIAGNOSTIC", "accepted": False,
            "navigation": False, "original_database_unchanged": True,
            "inputs": hashes, "database_sha256_after": after,
            "pose_trajectories": {k: {"path": str(p), "sha256": pose_hashes[k], "pose_count": len(poses[k])}
                                  for k, p in paths.items()},
            "pose_trajectories_unchanged_during_evaluation": True,
            "candidate_to_graph_binding_verified": False,
            "source_sha256": {p.name: sha256_file(p) for p in sources},
            "fixed_patch_count": len(rows), "candidate_stages": candidates,
            "baseline_stages": list(BASELINES), "severe_regression_threshold_m": SEVERE_M,
            "matched_support_by_candidate": summaries, "comparisons": rows,
            "common_support_all_stages": common_summary,
            "display_gauge": gauge, "visual_sampling": sampling, "source_rgb": rgb,
            "maximum_historical_reproduction_error_m": max((r["historical_reproduction_error_m"] for r in rows), default=0.),
            "figures": [str(p.relative_to(work)) for p in sorted(figures.glob("*.png"))],
            "limitations": [
                "Historical loop-selected correspondences are repeatedly inspected, not independent truth or a fresh held-out test.",
                "Finite patches are reconstructed once from raw-oriented original samples and verified against historical app metrics.",
                "Exact-pair graph provenance does not establish independence from reused frames, physical places or earlier selection.",
                "Any supplied graph is descriptive provenance only; candidate-to-graph binding is not certified by this evaluator.",
                "Per-patch best uses only available baselines, whose availability is recorded per patch.",
                "Missing candidate endpoints are never imputed; all comparisons expose matched support.",
                "Original depth XY plots use the common sampled frame intersection, not the full cloud or a completeness assessment.",
                "Display uses one rigid gauge anchor; there is no nonrigid visual alignment or line across trajectory gaps.",
                "Camera-height bands and point-Z slices are exploratory, not surveyed floor labels or measured stair heights.",
                "Historical finite-plane separation is not wall thickness, door width, true pose error or navigation approval.",
            ],
        }
        (work / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        write_csv(work / "patch_metrics.csv", metric_rows)
        flat = []
        for row in rows:
            flat.append({"patch_id": patch_id(row), "graph_partition_status": row["graph_partition_status"],
                         **{name: value for name, value in row["values"].items()}})
        write_csv(work / "comparisons.csv", flat)
        write_csv(work / "common_support_all_stages.csv", [
            {"stage": stage, "matched_count": common_summary["matched_count"], **metrics,
             **{f"severe_vs_{baseline}": count for baseline, count in
                common_summary["severe_regressions_by_candidate"].get(stage, {}).items()}}
            for stage, metrics in common_summary["metrics"].items()])
        ensure_fresh_output(output)
        work.rename(output)
    except Exception:
        shutil.rmtree(work, ignore_errors=True)
        raise
    print(json.dumps({"output": str(output), "patches": len(rows), "candidates": candidates,
                      "figures": len(report["figures"]), "accepted": False}))


if __name__ == "__main__":
    main()
