#!/usr/bin/env python3
"""Compare expanded submap candidates on frozen historical finite patches.

The finite planes, their train/test split, and their correspondence IDs are
reproduced from the pre-experiment audit.  They are fitted once in the original
raw-oriented camera frames and are never re-fitted after a candidate pose is
loaded.  Consequently this program measures the effect of pose changes rather
than making every candidate its own easier evaluation target.

This is a conditional, read-only diagnostic.  A historical loop-derived patch
is not surveyed ground truth, and a spatially withheld depth sample from a
frame used by the graph is not an independent validation observation.
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evaluate_map_surfaces import Settings, fit_planes, pair_metrics
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now
from refine_geometry_surfaces import SPECS


ROOT = Path(__file__).resolve().parents[1]
EVALUATED_SESSIONS = ("outside", "lobby", "floor2")
CRITICAL_PAIRS = {
    "outside": {(69, 323), (113, 351)},
    "lobby": {(595, 650), (445, 495), (82, 290)},
    "floor2": {(644, 1035), (548, 981), (656, 1055), (608, 1023), (503, 961)},
}
PREVIOUS_PREVIEW = ROOT / "outputs/student_union_20261017_all_preview/bundle"
HISTORICAL_ROOT = ROOT / "outputs/student_union_20261017_revalidated"
SEVERE_REGRESSION_M = 0.05
BASELINE_STAGES = ("previous_preview", "selected_initial")


def ensure_fresh_output(path: Path) -> None:
    """Refuse an existing output before opening a database or writing a file."""
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {path}")


def resolve_path(value, base: Path = ROOT) -> Path:
    path = Path(value)
    candidates = (path, base / path, ROOT / path)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(value)


def resolve_directory(value, base: Path = ROOT) -> Path:
    path = Path(value)
    for candidate in (path, base / path, ROOT / path):
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(value)


def write_csv(path: Path, rows: list[dict]) -> None:
    fields = sorted({key for row in rows for key in row}) or ["no_rows"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fixed_patches(clouds: Clouds, raw_poses: dict[int, np.ndarray], node: int) -> dict:
    """Fit the deterministic historical patches once, before candidate use."""
    training, testing = clouds.get(node)
    rotation = raw_poses[node][:3, :3]
    settings = Settings()
    result = {}
    for kind, offset in (("horizontal", 0), ("wall", 100000)):
        planes, _ = fit_planes(
            training @ rotation.T,
            testing @ rotation.T,
            kind,
            settings,
            settings.seed + node + offset,
        )
        for plane in planes:
            # Return to the original camera coordinates.  Every later stage
            # receives these exact arrays and cannot fit a new plane.
            plane["normal"] = rotation.T @ plane["normal"]
            plane["center"] = rotation.T @ plane["center"]
            plane["training"] = plane["training"] @ rotation
            plane["testing"] = plane["testing"] @ rotation
            result[kind, plane["plane_index"]] = plane
    return result


def pair_graph_provenance(graph: dict, session: str, node_a: int, node_b: int) -> dict:
    """Classify a frame pair from the graph's explicit source provenance.

    ``graph_independent_historical`` means only that the pair is absent from
    the graph objective.  It deliberately does not claim independent sensing,
    place identity, or ground truth.
    """
    hits = []
    train_groups = {
        str(edge.get("region_group"))
        for edge in graph.get("edges", [])
        if edge.get("role") == "train"
    }
    wanted = {(session, int(node_a)), (session, int(node_b))}
    for edge in graph.get("edges", []):
        endpoints = edge.get("provenance", {}).get("frame_endpoints")
        if not isinstance(endpoints, list) or len(endpoints) != 4:
            continue
        observed = {(str(endpoints[0]), int(endpoints[1])),
                    (str(endpoints[2]), int(endpoints[3]))}
        if observed == wanted:
            hits.append(edge)
    train = [edge for edge in hits if edge.get("role") == "train"]
    heldout = [edge for edge in hits if edge.get("role") == "heldout"]
    if train:
        status = "graph_training"
        reason = "the exact frame pair contributes a training edge"
    elif heldout:
        status = "correlated_heldout"
        same_region = any(str(edge.get("region_group")) in train_groups for edge in heldout)
        reason = (
            "held-out graph edge shares a physical-region group with training"
            if same_region else
            "edge is excluded from fitting, but its historical patch/place selection and frame observations remain conditional"
        )
    else:
        status = "graph_independent_historical"
        reason = "no exact source frame pair appears as a graph edge"
    return {
        "graph_partition_status": status,
        "graph_edge_ids": [str(edge.get("id")) for edge in hits],
        "graph_edge_roles": sorted({str(edge.get("role")) for edge in hits}),
        "graph_region_groups": sorted({str(edge.get("region_group")) for edge in hits}),
        "graph_correlation_reason": reason,
        "independent_ground_truth": False,
    }


def make_comparison(base: dict, stage_values: dict[str, float | None],
                    candidate_stages: list[str],
                    severe_threshold_m: float = SEVERE_REGRESSION_M) -> dict:
    """Create one auditable patch comparison, including missing candidates."""
    previous = stage_values.get("previous_preview")
    initial = stage_values.get("selected_initial")
    if previous is None or initial is None:
        raise ValueError("both baseline stages are required for every fixed patch")
    reference = min(previous, initial)
    result = {**base, "stage_plane_abs_median_m": dict(stage_values),
              "best_baseline_m": float(reference), "candidate_deltas": {}}
    for stage in candidate_stages:
        value = stage_values.get(stage)
        if value is None:
            result["candidate_deltas"][stage] = {
                "available": False,
                "delta_vs_initial_m": None,
                "delta_vs_previous_m": None,
                "delta_vs_best_baseline_m": None,
                "severe_regression": None,
            }
            continue
        delta_initial = float(value - initial)
        delta_previous = float(value - previous)
        delta_best = float(value - reference)
        result["candidate_deltas"][stage] = {
            "available": True,
            "delta_vs_initial_m": delta_initial,
            "delta_vs_previous_m": delta_previous,
            "delta_vs_best_baseline_m": delta_best,
            "severe_regression": bool(delta_best > severe_threshold_m),
            "severe_threshold_m": float(severe_threshold_m),
        }
    return result


def severe_regressions(comparisons: list[dict], candidate_stages: list[str]) -> dict:
    result = {}
    for stage in candidate_stages:
        rows = [row for row in comparisons
                if row["candidate_deltas"][stage]["severe_regression"] is True]
        result[stage] = sorted(
            rows,
            key=lambda row: -row["candidate_deltas"][stage]["delta_vs_best_baseline_m"],
        )
    return result


def aggregate_metrics(comparisons: list[dict], stages: list[str]) -> dict:
    """Aggregate available fixed-patch medians and expose coverage explicitly."""
    result = {}
    total = len(comparisons)
    for stage in stages:
        values = np.asarray([
            row["stage_plane_abs_median_m"].get(stage)
            for row in comparisons
            if row["stage_plane_abs_median_m"].get(stage) is not None
        ], dtype=float)
        entry = {
            "total_fixed_patches": total,
            "available_count": int(len(values)),
            "missing_count": int(total - len(values)),
            "coverage_fraction": float(len(values) / total) if total else 0.,
        }
        if len(values):
            entry.update({
                "median_m": float(np.median(values)),
                "p90_m": float(np.percentile(values, 90)),
                "maximum_m": float(np.max(values)),
            })
        else:
            entry.update({"median_m": None, "p90_m": None, "maximum_m": None})
        result[stage] = entry
    return result


def grouped_aggregates(comparisons: list[dict], stages: list[str], field: str) -> dict:
    return {
        value: aggregate_metrics([row for row in comparisons if row[field] == value], stages)
        for value in sorted({row[field] for row in comparisons})
    }


def flatten_comparisons(comparisons: list[dict], stages: list[str],
                        candidate_stages: list[str]) -> list[dict]:
    rows = []
    for comparison in comparisons:
        row = {key: value for key, value in comparison.items()
               if key not in ("stage_plane_abs_median_m", "candidate_deltas")}
        for key in ("graph_edge_ids", "graph_edge_roles", "graph_region_groups"):
            if key in row:
                row[key] = ";".join(row[key])
        for stage in stages:
            row[f"{stage}_plane_abs_median_m"] = comparison[
                "stage_plane_abs_median_m"].get(stage)
        for stage in candidate_stages:
            for key, value in comparison["candidate_deltas"][stage].items():
                row[f"{stage}_{key}"] = value
        rows.append(row)
    return rows


def plot_patch(path: Path, patch_a: dict, patch_b: dict,
               transforms: dict[str, np.ndarray], metrics: dict[str, dict],
               title: str) -> None:
    """Render all available stages in one stable original-patch coordinate view."""
    normal = patch_a["normal"]
    _, _, vectors = np.linalg.svd(
        patch_a["training"] - patch_a["center"], full_matrices=False)
    basis = np.vstack((vectors[0], vectors[1], normal))
    target = (patch_a["testing"] - patch_a["center"]) @ basis.T
    moving = {
        label: (apply(patch_b["testing"], transform) - patch_a["center"]) @ basis.T
        for label, transform in transforms.items()
    }
    total = np.vstack([target, *moving.values()])
    lower, upper = np.min(total, axis=0), np.max(total, axis=0)
    padding = np.maximum(.03, .04 * (upper - lower))
    width = max(11., 3.8 * len(moving))
    figure, axes = plt.subplots(2, len(moving), figsize=(width, 7.4), squeeze=False)
    for column, (label, points) in enumerate(moving.items()):
        for row, tangent in enumerate((0, 1)):
            axis = axes[row, column]
            axis.scatter(target[:, tangent], target[:, 2], s=3, c="#147ba7",
                         alpha=.65, label="fixed visit A")
            axis.scatter(points[:, tangent], points[:, 2], s=3, c="#dc702c",
                         alpha=.65, label="visit B")
            axis.axhline(0., color="#606060", linewidth=.7)
            axis.set(
                xlim=(lower[tangent] - padding[tangent], upper[tangent] + padding[tangent]),
                ylim=(lower[2] - padding[2], upper[2] + padding[2]),
                xlabel=f"patch tangent {tangent + 1} (m)",
                ylabel="signed normal offset (m)",
            )
            axis.grid(alpha=.2)
        value = metrics[label]["symmetric_plane_abs_median_m"]
        axes[0, column].set_title(f"{label}\nmedian {value * 100:.1f} cm", fontsize=9)
    axes[0, 0].legend(fontsize=8)
    figure.suptitle(
        title + "\nidentical original finite-patch samples and axes; no candidate re-fit",
        fontsize=12,
    )
    figure.tight_layout(rect=(0., 0., 1., .91))
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_progression(path: Path, comparisons: list[dict], stages: list[str]) -> None:
    aggregates = aggregate_metrics(comparisons, stages)
    x = np.arange(len(stages))
    median = [100. * aggregates[stage]["median_m"] if aggregates[stage]["median_m"] is not None else np.nan
              for stage in stages]
    p90 = [100. * aggregates[stage]["p90_m"] if aggregates[stage]["p90_m"] is not None else np.nan
           for stage in stages]
    maximum = [100. * aggregates[stage]["maximum_m"] if aggregates[stage]["maximum_m"] is not None else np.nan
               for stage in stages]
    coverage = [100. * aggregates[stage]["coverage_fraction"] for stage in stages]
    figure, axes = plt.subplots(2, 1, figsize=(max(11., 2.1 * len(stages)), 9.5),
                                gridspec_kw={"height_ratios": [3, 1]})
    axes[0].plot(x, median, "o-", linewidth=2, label="median")
    axes[0].plot(x, p90, "s-", linewidth=2, label="P90")
    axes[0].plot(x, maximum, "^-", linewidth=1.5, label="maximum")
    axes[0].set(ylabel="fixed-patch separation (cm)", xticks=x, xticklabels=stages)
    axes[0].grid(alpha=.25)
    axes[0].legend()
    axes[1].bar(x, coverage, color="#5679a6")
    axes[1].axhline(100., color="#444444", linewidth=.8)
    axes[1].set(ylabel="coverage (%)", ylim=(0., 105.), xticks=x, xticklabels=stages)
    axes[1].grid(axis="y", alpha=.2)
    figure.suptitle(
        "Frozen historical finite patches | no after-map re-fit\n"
        "missing quality-filtered endpoints reduce coverage and are not imputed",
        fontsize=13,
    )
    figure.tight_layout(rect=(0., 0., 1., .93))
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_critical_heatmap(path: Path, comparisons: list[dict], stages: list[str],
                          candidate_stages: list[str], limit: int = 16) -> None:
    def priority(row):
        deltas = [row["candidate_deltas"][stage]["delta_vs_best_baseline_m"]
                  for stage in candidate_stages
                  if row["candidate_deltas"][stage]["available"]]
        return max(deltas, default=-np.inf)

    selected = [row for row in comparisons
                if (row["a"], row["b"]) in CRITICAL_PAIRS.get(row["session"], set())
                or any(row["candidate_deltas"][stage]["severe_regression"] is True
                       for stage in candidate_stages)]
    selected = sorted(selected, key=priority, reverse=True)[:limit]
    if not selected:
        selected = sorted(comparisons, key=priority, reverse=True)[:min(limit, len(comparisons))]
    if not selected:
        return
    values = np.asarray([
        [np.nan if row["stage_plane_abs_median_m"].get(stage) is None else
         100. * row["stage_plane_abs_median_m"][stage] for stage in stages]
        for row in selected
    ])
    height = max(5.5, .48 * len(selected) + 2.)
    figure, axis = plt.subplots(figsize=(max(10., 2.1 * len(stages)), height))
    image = axis.imshow(values, aspect="auto", cmap="magma_r")
    axis.set_xticks(np.arange(len(stages)), labels=stages)
    labels = [f"{row['session']} {row['a']}/{row['b']} {row['kind']} {row['plane_a']}/{row['plane_b']}"
              for row in selected]
    axis.set_yticks(np.arange(len(selected)), labels=labels)
    for row in range(values.shape[0]):
        for column in range(values.shape[1]):
            value = values[row, column]
            axis.text(column, row, "missing" if np.isnan(value) else f"{value:.1f}",
                      ha="center", va="center", fontsize=7,
                      color="black" if np.isnan(value) or value < np.nanmedian(values) else "white")
    figure.colorbar(image, ax=axis, label="median separation (cm)")
    axis.set_title("Critical and severe-regression fixed patches | unchanged samples")
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def session_inputs(graph: dict, expanded: dict, expanded_root: Path,
                   graph_manifest_path: Path) -> tuple[Path, dict, dict[str, dict[str, Path]]]:
    metadata = graph.get("metadata", {})
    if not metadata.get("initial_graph"):
        raise ValueError("graph manifest metadata has no initial_graph directory")
    initial_graph = resolve_directory(metadata["initial_graph"], graph_manifest_path.parent)
    initial_report_path = initial_graph / "report.json"
    initial_report = json.loads(initial_report_path.read_text(encoding="utf-8"))

    expected_graph_hash = expanded.get("inputs", {}).get("graph_manifest_sha256")
    actual_graph_hash = sha256_file(graph_manifest_path)
    if expected_graph_hash and expected_graph_hash != actual_graph_hash:
        raise ValueError("expanded poses were produced from a different graph manifest")

    candidates = sorted(expanded.get("variants", {}))
    if not candidates:
        raise ValueError("expanded manifest has no variants")
    result = {}
    for name in EVALUATED_SESSIONS:
        baseline = initial_report.get("baseline_selection", {}).get(name, {})
        sources = baseline.get("candidate_sources", {})
        database = resolve_path(baseline.get("database", ROOT / "data/raw/rtabmap" / SPECS[name][0]))
        raw = resolve_path(sources.get("raw", {}).get(
            "path", ROOT / "outputs" / SPECS[name][1] / name / "raw_main_trajectory.csv"))
        app = resolve_path(sources.get("app", {}).get(
            "path", ROOT / "outputs" / SPECS[name][1] / "audit" / name / "app_optimized_trajectory.csv"))
        stages = {
            "previous_preview": resolve_path(PREVIOUS_PREVIEW / f"{name}_trajectory.csv"),
            "selected_initial": resolve_path(initial_graph / name / "initial.csv"),
        }
        for variant in candidates:
            record = expanded["variants"][variant].get("sessions", {}).get(name)
            if not record:
                raise ValueError(f"expanded variant {variant} has no {name} session")
            trajectory = resolve_path(record["trajectory_csv"], expanded_root)
            expected = record.get("trajectory_sha256")
            if expected and expected != sha256_file(trajectory):
                raise ValueError(f"expanded trajectory hash mismatch: {variant}/{name}")
            stages[variant] = trajectory
        historical = resolve_path(
            HISTORICAL_ROOT / SPECS[name][2] / name / "plane_pair_metrics.csv")
        result[name] = {"database": database, "raw": raw, "app": app,
                        "historical": historical, "stages": stages}
    return initial_graph, initial_report, result


def evaluate(args) -> tuple[dict, list[dict], list[dict], dict]:
    expanded_root = args.expanded_root.resolve()
    graph_path = args.graph_manifest.resolve()
    expanded_path = expanded_root / "manifest.json"
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    expanded = json.loads(expanded_path.read_text(encoding="utf-8"))
    initial_graph, initial_report, inputs = session_inputs(
        graph, expanded, expanded_root, graph_path)
    candidate_stages = sorted(expanded["variants"])
    stages = [*BASELINE_STAGES, *candidate_stages]

    patch_rows = []
    comparisons = []
    plot_payloads = {}
    database_hashes = {}
    pose_hashes = {}
    reference_hashes = {}
    reproduction_errors = []
    for name in EVALUATED_SESSIONS:
        entry = inputs[name]
        database = entry["database"]
        database_hashes[name] = {"path": str(database), "before": sha256_file(database)}
        raw = load_pose_csv(entry["raw"])
        app = load_pose_csv(entry["app"])
        reference_hashes[name] = {
            "raw_pose_csv": {"path": str(entry["raw"]),
                             "sha256": sha256_file(entry["raw"])},
            "historical_app_pose_csv": {"path": str(entry["app"]),
                                        "sha256": sha256_file(entry["app"])},
            "historical_patch_metrics_csv": {"path": str(entry["historical"]),
                                             "sha256": sha256_file(entry["historical"])},
        }
        poses = {stage: load_pose_csv(path) for stage, path in entry["stages"].items()}
        pose_hashes[name] = {stage: {"path": str(path), "sha256": sha256_file(path)}
                             for stage, path in entry["stages"].items()}
        with entry["historical"].open(newline="", encoding="utf-8") as stream:
            historical = [row for row in csv.DictReader(stream) if row["trajectory"] == "app"]
        if args.scan_all_historical:
            pairs = sorted({(int(row["from_id"]), int(row["to_id"])) for row in historical})
        else:
            pairs = sorted(CRITICAL_PAIRS[name])

        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            clouds = Clouds(connection, None)
            cache = {}
            for node_a, node_b in pairs:
                matches = [row for row in historical
                           if int(row["from_id"]) == node_a and int(row["to_id"]) == node_b]
                if not matches:
                    raise ValueError(f"no historical fixed patch: {name} {node_a}/{node_b}")
                for node in (node_a, node_b):
                    if node not in raw:
                        raise ValueError(f"raw pose is missing historical node {name}/{node}")
                    if node not in cache:
                        cache[node] = fixed_patches(clouds, raw, node)
                provenance = pair_graph_provenance(graph, name, node_a, node_b)
                for historical_row in matches:
                    kind = historical_row["kind"]
                    plane_a = int(historical_row["from_plane"])
                    plane_b = int(historical_row["to_plane"])
                    patch_a = cache[node_a][kind, plane_a]
                    patch_b = cache[node_b][kind, plane_b]
                    if node_a not in app or node_b not in app:
                        raise ValueError(f"app reproduction pose missing {name} {node_a}/{node_b}")
                    reproduced = pair_metrics(patch_a, patch_b, app[node_a], app[node_b])
                    reproduction_error = abs(
                        reproduced["symmetric_plane_abs_median_m"]
                        - float(historical_row["symmetric_plane_abs_median_m"]))
                    reproduction_errors.append(reproduction_error)
                    if reproduction_error > 5e-5:
                        raise ValueError(
                            f"historical patch not reproduced: {name} {node_a}/{node_b} "
                            f"{kind} {plane_a}/{plane_b}; {reproduction_error:.9f} m")

                    base = {
                        "session": name, "a": node_a, "b": node_b, "kind": kind,
                        "plane_a": plane_a, "plane_b": plane_b,
                        "historical_reproduction_error_m": float(reproduction_error),
                        **provenance,
                    }
                    values = {}
                    metrics = {}
                    transforms = {}
                    for stage in stages:
                        trajectory = poses[stage]
                        if node_a not in trajectory or node_b not in trajectory:
                            values[stage] = None
                            continue
                        metric = pair_metrics(
                            patch_a, patch_b, trajectory[node_a], trajectory[node_b])
                        metrics[stage] = metric
                        values[stage] = float(metric["symmetric_plane_abs_median_m"])
                        transforms[stage] = inverse(trajectory[node_a]) @ trajectory[node_b]
                        patch_rows.append({**base, "stage": stage, "available": True, **metric})
                    for stage in stages:
                        if stage not in metrics:
                            patch_rows.append({**base, "stage": stage, "available": False,
                                               "missing_reason": "candidate trajectory omits one or both quality-filtered endpoints"})
                    comparison = make_comparison(base, values, candidate_stages,
                                                 args.severe_regression_m)
                    comparisons.append(comparison)
                    key = (name, node_a, node_b, kind, plane_a, plane_b)
                    plot_payloads[key] = (patch_a, patch_b, transforms, metrics, provenance)
        finally:
            connection.close()
        database_hashes[name]["after"] = sha256_file(database)
        database_hashes[name]["unchanged"] = (
            database_hashes[name]["before"] == database_hashes[name]["after"])
        if not database_hashes[name]["unchanged"]:
            raise RuntimeError(f"source database changed during evaluation: {database}")

    all_stages = list(stages)
    aggregates = {
        "overall": aggregate_metrics(comparisons, all_stages),
        "by_session": grouped_aggregates(comparisons, all_stages, "session"),
        "by_graph_partition": grouped_aggregates(
            comparisons, all_stages, "graph_partition_status"),
    }
    severe = severe_regressions(comparisons, candidate_stages)
    ranking = sorted(candidate_stages, key=lambda stage: (
        aggregates["overall"][stage]["missing_count"],
        len(severe[stage]),
        aggregates["overall"][stage]["p90_m"] is None,
        aggregates["overall"][stage]["p90_m"] if aggregates["overall"][stage]["p90_m"] is not None else np.inf,
        aggregates["overall"][stage]["maximum_m"] if aggregates["overall"][stage]["maximum_m"] is not None else np.inf,
    ))
    report = {
        "schema_version": 1,
        "created_utc": utc_now(),
        "status": "EXPERIMENTAL_CONDITIONAL_FIXED_PATCH_EVALUATION",
        "accepted": False,
        "navigation": False,
        "command": sys.argv,
        "inputs": {
            "expanded_manifest": str(expanded_path),
            "expanded_manifest_sha256": sha256_file(expanded_path),
            "graph_manifest": str(graph_path),
            "graph_manifest_sha256": sha256_file(graph_path),
            "initial_graph": str(initial_graph),
            "initial_graph_report_sha256": sha256_file(initial_graph / "report.json"),
            "pose_trajectories": pose_hashes,
            "fixed_patch_references": reference_hashes,
        },
        "source_sha256": sha256_file(Path(__file__)),
        "database_sha256": database_hashes,
        "original_databases_unchanged": all(row["unchanged"] for row in database_hashes.values()),
        "method": [
            "reproduce historical finite patch IDs and deterministic raw-oriented train/test extraction",
            "fit each finite plane once before loading candidate poses",
            "evaluate gauge-invariant pair-relative transforms with identical finite samples",
            "never refit a plane or correspondence on an optimized candidate",
            "record quality-filtered missing endpoints rather than imputing them",
        ],
        "validation_partition_definition": {
            "graph_training": "exact source frame pair contributes to the optimization objective",
            "correlated_heldout": "edge is excluded from fitting, but source frames/place selection or physical region remain conditional",
            "graph_independent_historical": "exact source pair is absent from this graph; still not surveyed or sensor-independent ground truth",
        },
        "limitations": [
            "Historical patch correspondences were established from earlier loop/place hypotheses and are conditional, not surveyed truth.",
            "Spatially withheld depth samples from the same RGB-D frames are not independent sensor observations.",
            "A pair absent from the new graph is graph-independent only; prior algorithmic selection can remain correlated.",
            "Quality filtering may remove a historical endpoint; missing patches are reported and never assigned a favorable value.",
            "No measured door width, stair rise, floor height, or total-station reference is available here.",
        ],
        "scan_all_historical": bool(args.scan_all_historical),
        "fixed_patch_count": len(comparisons),
        "maximum_historical_reproduction_error_m": float(max(reproduction_errors, default=0.)),
        "stages": all_stages,
        "candidate_stages": candidate_stages,
        "stage_aggregates": aggregates,
        "candidate_ranking_diagnostic_only": ranking,
        "candidate_ranking_policy": "ascending missing-patch count, severe-regression count, overall P90 and maximum; never grants approval",
        "severe_regression_threshold_m": float(args.severe_regression_m),
        "severe_regressions_by_candidate": severe,
        "comparisons": comparisons,
        "figures": [],
    }
    context = {"stages": all_stages, "candidate_stages": candidate_stages,
               "plot_payloads": plot_payloads, "initial_report": initial_report}
    return report, patch_rows, comparisons, context


def write_outputs(output: Path, report: dict, patch_rows: list[dict],
                  comparisons: list[dict], context: dict) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}.tmp-", dir=output.parent))
    try:
        stages = context["stages"]
        candidates = context["candidate_stages"]
        figures_dir = temporary / "figures"
        figures_dir.mkdir()
        progression = figures_dir / "finite_patch_progression.png"
        critical = figures_dir / "critical_regressions.png"
        plot_progression(progression, comparisons, stages)
        plot_critical_heatmap(critical, comparisons, stages, candidates)
        figure_paths = [str(progression.relative_to(temporary))]
        if critical.exists():
            figure_paths.append(str(critical.relative_to(temporary)))

        severe = severe_regressions(comparisons, candidates)
        severe_keys = {
            (row["session"], row["a"], row["b"], row["kind"], row["plane_a"], row["plane_b"])
            for rows in severe.values() for row in rows
        }
        critical_keys = {
            (row["session"], row["a"], row["b"], row["kind"], row["plane_a"], row["plane_b"])
            for row in comparisons
            if (row["a"], row["b"]) in CRITICAL_PAIRS.get(row["session"], set())
        }
        keys = sorted(critical_keys | severe_keys)
        # Cap the gallery deterministically; the summary heatmap still contains
        # the largest regressions even when many historical patches regress.
        if len(keys) > 24:
            priority = {}
            by_key = {(row["session"], row["a"], row["b"], row["kind"], row["plane_a"], row["plane_b"]): row
                      for row in comparisons}
            for key in keys:
                row = by_key[key]
                priority[key] = max((
                    row["candidate_deltas"][stage]["delta_vs_best_baseline_m"]
                    for stage in candidates
                    if row["candidate_deltas"][stage]["available"]), default=-np.inf)
            keep = set(critical_keys)
            for key in sorted(keys, key=lambda value: priority[value], reverse=True):
                if len(keep) >= 24:
                    break
                keep.add(key)
            keys = sorted(keep)[:24]
        for key in keys:
            patch_a, patch_b, transforms, metrics, provenance = context["plot_payloads"][key]
            session, node_a, node_b, kind, plane_a, plane_b = key
            filename = f"{session}_{node_a}_{node_b}_{kind}_{plane_a}_{plane_b}.png"
            path = figures_dir / filename
            plot_patch(path, patch_a, patch_b, transforms, metrics,
                       f"{session} {node_a}/{node_b} {kind}; {provenance['graph_partition_status']}")
            figure_paths.append(str(path.relative_to(temporary)))

        report["figures"] = figure_paths
        report["output"] = str(output)
        (temporary / "report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        write_csv(temporary / "patch_metrics.csv", patch_rows)
        write_csv(temporary / "comparisons.csv",
                  flatten_comparisons(comparisons, stages, candidates))
        aggregate_rows = []
        for scope, groups in (("overall", {"all": report["stage_aggregates"]["overall"]}),
                              ("session", report["stage_aggregates"]["by_session"]),
                              ("graph_partition", report["stage_aggregates"]["by_graph_partition"])):
            for group, values in groups.items():
                for stage, metrics in values.items():
                    aggregate_rows.append({"scope": scope, "group": group,
                                           "stage": stage, **metrics})
        write_csv(temporary / "stage_aggregates.csv", aggregate_rows)
        shutil.copyfile(Path(__file__), temporary / "source_snapshot.py")
        ensure_fresh_output(output)
        temporary.rename(output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expanded-root", type=Path, required=True,
                        help="directory containing the expanded-pose manifest.json")
    parser.add_argument("--graph-manifest", type=Path, required=True,
                        help="exact submap graph manifest used for the backend")
    parser.add_argument("--output", type=Path, required=True,
                        help="fresh output directory; existing paths are refused")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--scan-all-historical", action="store_true",
                      help="evaluate every historical accepted finite-patch correspondence")
    mode.add_argument("--critical-only", dest="scan_all_historical", action="store_false",
                      help="evaluate only the predeclared critical wall/floor examples")
    parser.set_defaults(scan_all_historical=True)
    parser.add_argument("--severe-regression-m", type=float,
                        default=SEVERE_REGRESSION_M)
    args = parser.parse_args(argv)
    if args.severe_regression_m <= 0:
        parser.error("--severe-regression-m must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    args.output = args.output.resolve()
    ensure_fresh_output(args.output)
    if not (args.expanded_root / "manifest.json").is_file():
        raise FileNotFoundError(args.expanded_root / "manifest.json")
    if not args.graph_manifest.is_file():
        raise FileNotFoundError(args.graph_manifest)
    report, patch_rows, comparisons, context = evaluate(args)
    write_outputs(args.output, report, patch_rows, comparisons, context)
    print(json.dumps({
        "output": str(args.output),
        "fixed_patches": len(comparisons),
        "candidates": context["candidate_stages"],
        "accepted": False,
        "navigation": False,
    }))


if __name__ == "__main__":
    main()
