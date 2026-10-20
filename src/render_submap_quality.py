#!/usr/bin/env python3
"""Render audit figures from the immutable frame-quality/submap artifacts.

The renderer reads only CSV/JSON artifacts produced by ``build_quality_submaps``.
It does not decode sensor blobs, change a pose, optimize a graph, or edit an
RTAB-Map database.  Every plotted point is a row from ``frames.csv`` and every
submap anchor is a record from ``submaps.json``.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


GRADE_COLORS = {"A": "#238b45", "B": "#e69123", "C": "#c63d3d"}
SESSION_COLORS = {
    "outside": "#309c54",
    "lobby": "#e99731",
    "bridge_out1f": "#d358a4",
    "floor2": "#1594c7",
    "bridge_12_sub": "#8c59c6",
}
DEFAULT_DIAGNOSTIC_THRESHOLDS = {
    "score_a_min": 75.0,
    "score_b_min": 45.0,
    "intrinsic_a_max": 0.5,
    "intrinsic_b_max": 2.0,
    "intrinsic_hard": 3.0,
    "linear_a_max": 0.35,
    "linear_b_max": 0.75,
    "linear_hard": 1.0,
    "angular_a_max": 12.0,
    "angular_b_max": 25.0,
    "angular_c_gate": 35.0,
    "angular_hard": 75.0,
}
REQUIRED_FRAME_COLUMNS = {
    "node_id",
    "pose_x_m",
    "pose_y_m",
    "pose_z_m",
    "session_path_m",
    "linear_step_m",
    "angular_step_deg",
    "intrinsic_jump_depth_px",
    "quality_score",
    "quality_grade",
    "hard_boundary",
    "correlation_group_id",
    "primary_submap_id",
}
FLOAT_COLUMNS = {
    "pose_x_m",
    "pose_y_m",
    "pose_z_m",
    "session_path_m",
    "linear_step_m",
    "angular_step_deg",
    "intrinsic_jump_depth_px",
    "quality_score",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_optional_float(value: str | None) -> float:
    if value is None or not value.strip():
        return float("nan")
    return float(value)


def parse_bool(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value or "").strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no", ""}:
        return False
    raise ValueError(f"Unsupported boolean value: {value!r}")


def read_frames(path: Path) -> list[dict[str, Any]]:
    """Read and type the exact per-frame audit rows used by the plots."""
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_FRAME_COLUMNS - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path} is missing required columns: {sorted(missing)}")
        rows: list[dict[str, Any]] = []
        for raw in reader:
            row: dict[str, Any] = dict(raw)
            row["node_id"] = int(raw["node_id"])
            for column in FLOAT_COLUMNS:
                row[column] = parse_optional_float(raw.get(column))
            row["hard_boundary"] = parse_bool(raw.get("hard_boundary"))
            if row["quality_grade"] not in GRADE_COLORS:
                raise ValueError(
                    f"Unsupported quality grade {row['quality_grade']!r} in {path}"
                )
            rows.append(row)
    if not rows:
        raise ValueError(f"No frame rows in {path}")
    node_ids = [row["node_id"] for row in rows]
    if len(node_ids) != len(set(node_ids)):
        raise ValueError(f"Duplicate node_id values in {path}")
    return rows


def resolve_artifact(value: str, input_root: Path, session: str, fallback: str) -> Path:
    path = Path(value)
    if path.exists():
        return path.resolve()
    if not path.is_absolute() and (input_root / path).exists():
        return (input_root / path).resolve()
    portable = input_root / session / fallback
    if portable.exists():
        return portable.resolve()
    raise FileNotFoundError(f"Artifact not found: {value} (fallback {portable})")


def load_quality_inputs(manifest_path: Path) -> tuple[dict, dict[str, dict[str, Any]], dict[str, Path]]:
    """Load manifest-declared frame and submap artifacts without touching DB data."""
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text())
    if not isinstance(manifest.get("sessions"), dict) or not manifest["sessions"]:
        raise ValueError("Input manifest has no sessions")
    input_root = manifest_path.parent
    files: dict[str, Path] = {"quality_manifest": manifest_path}
    sessions: dict[str, dict[str, Any]] = {}
    for name, summary in manifest["sessions"].items():
        artifacts = summary.get("artifacts", {})
        frames_path = resolve_artifact(
            artifacts.get("frames_csv", ""), input_root, name, "frames.csv"
        )
        submaps_path = resolve_artifact(
            artifacts.get("submaps_json", ""), input_root, name, "submaps.json"
        )
        frames = read_frames(frames_path)
        submap_document = json.loads(submaps_path.read_text())
        submaps = submap_document.get("submaps")
        if not isinstance(submaps, list):
            raise ValueError(f"No submap list in {submaps_path}")
        if int(summary.get("frame_count", len(frames))) != len(frames):
            raise ValueError(f"Frame count disagrees with manifest for {name}")
        if int(summary.get("submap_count", len(submaps))) != len(submaps):
            raise ValueError(f"Submap count disagrees with manifest for {name}")
        files[f"{name}:frames_csv"] = frames_path
        files[f"{name}:submaps_json"] = submaps_path
        submaps_csv = artifacts.get("submaps_csv")
        if submaps_csv:
            files[f"{name}:submaps_csv"] = resolve_artifact(
                submaps_csv, input_root, name, "submaps.csv"
            )
        session_manifest = input_root / name / "session_manifest.json"
        if session_manifest.exists():
            files[f"{name}:session_manifest"] = session_manifest.resolve()
        sessions[name] = {
            "summary": summary,
            "frames": frames,
            "submaps": submaps,
            "frames_path": frames_path,
            "submaps_path": submaps_path,
        }
    return manifest, sessions, files


def finite_xy_rows(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        row
        for row in rows
        if np.isfinite(row["pose_x_m"]) and np.isfinite(row["pose_y_m"])
    ]


def draw_broken_trajectory(ax: plt.Axes, rows: list[dict[str, Any]], x_key: str, y_key: str) -> int:
    """Draw pose order while refusing to bridge an explicit hard boundary."""
    segment: list[dict[str, Any]] = []
    segment_count = 0

    def flush() -> None:
        nonlocal segment_count
        if len(segment) >= 2:
            ax.plot(
                [r[x_key] for r in segment],
                [r[y_key] for r in segment],
                color="#8a929b",
                linewidth=0.65,
                alpha=0.55,
                zorder=1,
            )
            segment_count += 1

    for row in rows:
        finite = np.isfinite(row[x_key]) and np.isfinite(row[y_key])
        if not finite or row["hard_boundary"]:
            flush()
            segment = []
        if finite:
            segment.append(row)
    flush()
    return segment_count


def grade_scatter(ax: plt.Axes, rows: list[dict[str, Any]], x_key: str, y_key: str) -> None:
    for grade in ("A", "B", "C"):
        selected = [
            row
            for row in rows
            if row["quality_grade"] == grade
            and np.isfinite(row[x_key])
            and np.isfinite(row[y_key])
        ]
        if selected:
            ax.scatter(
                [row[x_key] for row in selected],
                [row[y_key] for row in selected],
                s=10,
                color=GRADE_COLORS[grade],
                alpha=0.82,
                linewidths=0,
                zorder=2,
            )


def anchor_rows(frames: list[dict[str, Any]], submaps: list[dict[str, Any]]) -> tuple[list[dict], list[dict]]:
    by_node = {row["node_id"]: row for row in frames}
    nominal: list[dict] = []
    short: list[dict] = []
    for submap in submaps:
        row = by_node.get(int(submap["anchor_node_id"]))
        if row is None:
            continue
        (nominal if submap.get("within_nominal_2_to_5m") else short).append(row)
    return nominal, short


def draw_event_markers(
    ax: plt.Axes,
    rows: list[dict[str, Any]],
    submaps: list[dict[str, Any]],
    x_key: str,
    y_key: str,
) -> tuple[int, int, int]:
    hard = [
        row
        for row in rows
        if row["hard_boundary"] and np.isfinite(row[x_key]) and np.isfinite(row[y_key])
    ]
    nominal, short = anchor_rows(rows, submaps)
    nominal = [r for r in nominal if np.isfinite(r[x_key]) and np.isfinite(r[y_key])]
    short = [r for r in short if np.isfinite(r[x_key]) and np.isfinite(r[y_key])]
    if nominal:
        ax.scatter(
            [r[x_key] for r in nominal],
            [r[y_key] for r in nominal],
            s=30,
            facecolors="none",
            edgecolors="#17212b",
            linewidths=0.65,
            zorder=3,
        )
    if short:
        ax.scatter(
            [r[x_key] for r in short],
            [r[y_key] for r in short],
            marker="s",
            s=28,
            facecolors="none",
            edgecolors="#7a3f91",
            linewidths=0.8,
            zorder=4,
        )
    if hard:
        ax.scatter(
            [r[x_key] for r in hard],
            [r[y_key] for r in hard],
            marker="x",
            s=55,
            color="#111111",
            linewidths=1.15,
            zorder=5,
        )
    return len(hard), len(nominal), len(short)


def render_session_trajectory(
    session: str,
    frames: list[dict[str, Any]],
    submaps: list[dict[str, Any]],
    path: Path,
) -> dict[str, Any]:
    """Render XY trajectory and sequential camera-Z profile from frozen poses."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5))
    # Reserve separate title and legend bands.  ``constrained_layout`` can put
    # a figure-level legend on top of the suptitle on some Matplotlib versions.
    fig.subplots_adjust(top=0.80, bottom=0.10, left=0.06, right=0.985, wspace=0.17)
    xy_rows = finite_xy_rows(frames)
    segments = draw_broken_trajectory(axes[0], xy_rows, "pose_x_m", "pose_y_m")
    grade_scatter(axes[0], xy_rows, "pose_x_m", "pose_y_m")
    hard_count, nominal_count, short_count = draw_event_markers(
        axes[0], xy_rows, submaps, "pose_x_m", "pose_y_m"
    )
    axes[0].set(
        title="Frozen baseline trajectory (XY)",
        xlabel="X (m)",
        ylabel="Y (m)",
    )
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].grid(alpha=0.2)

    draw_broken_trajectory(axes[1], frames, "session_path_m", "pose_z_m")
    grade_scatter(axes[1], frames, "session_path_m", "pose_z_m")
    draw_event_markers(axes[1], frames, submaps, "session_path_m", "pose_z_m")
    axes[1].set(
        title="Sequential camera Z (not surveyed floor height)",
        xlabel="Cumulative frozen-pose path (m)",
        ylabel="Camera Z (m)",
    )
    axes[1].grid(alpha=0.2)

    legend = [
        Line2D([0], [0], marker="o", color="none", markerfacecolor=GRADE_COLORS[g],
               markeredgecolor="none", markersize=6, label=f"Grade {g}")
        for g in ("A", "B", "C")
    ]
    legend.extend(
        [
            Line2D([0], [0], marker="o", color="none", markerfacecolor="none",
                   markeredgecolor="#17212b", markersize=7, label="Nominal submap anchor"),
            Line2D([0], [0], marker="s", color="none", markerfacecolor="none",
                   markeredgecolor="#7a3f91", markersize=7, label="Short submap anchor"),
            Line2D([0], [0], marker="x", color="#111111", linestyle="none",
                   markersize=7, label="Hard boundary"),
        ]
    )
    fig.legend(
        handles=legend,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.905),
        ncol=6,
        frameon=False,
        fontsize=9,
    )
    fig.suptitle(
        f"{session} | measured frame-quality audit; no pose optimization or map acceptance",
        fontsize=14,
        y=0.985,
    )
    fig.savefig(path, dpi=190, metadata={"Title": f"{session} frame quality and submaps"})
    plt.close(fig)
    return {
        "session": session,
        "frame_count": len(frames),
        "grade_counts": dict(Counter(row["quality_grade"] for row in frames)),
        "hard_boundary_count_plotted": hard_count,
        "nominal_anchor_count_plotted": nominal_count,
        "short_anchor_count_plotted": short_count,
        "trajectory_segment_count": segments,
        "projections": ["frozen_pose_xy", "session_path_vs_camera_z"],
    }


def render_summary(sessions: dict[str, dict[str, Any]], path: Path) -> dict[str, Any]:
    names = list(sessions)
    x = np.arange(len(names))
    fig, axes = plt.subplots(3, 1, figsize=(15, 14), constrained_layout=True)

    grade_totals = {grade: [] for grade in ("A", "B", "C")}
    for name in names:
        counts = Counter(row["quality_grade"] for row in sessions[name]["frames"])
        for grade in grade_totals:
            grade_totals[grade].append(counts[grade])
    bottom = np.zeros(len(names))
    for grade in ("A", "B", "C"):
        values = np.asarray(grade_totals[grade])
        axes[0].bar(x, values, bottom=bottom, color=GRADE_COLORS[grade], label=f"Grade {grade}")
        bottom += values
    axes[0].set(title="Frame grades", ylabel="Frames", xticks=x, xticklabels=names)
    axes[0].legend(ncol=3, frameon=False)

    width = 0.25
    usable = [int(sessions[n]["summary"]["usable_frame_count"]) for n in names]
    groups = [int(sessions[n]["summary"]["correlation_group_count"]) for n in names]
    primary_groups = [
        sum(int(submap.get("primary_evidence_groups", 0)) for submap in sessions[n]["submaps"])
        for n in names
    ]
    axes[1].bar(x - width, usable, width, label="Usable frames", color="#7894b6")
    axes[1].bar(x, groups, width, label="Correlation groups", color="#416c9a")
    axes[1].bar(x + width, primary_groups, width, label="Primary submap evidence groups", color="#233f63")
    axes[1].set(
        title="Correlation-aware evidence accounting",
        ylabel="Count",
        xticks=x,
        xticklabels=names,
    )
    axes[1].legend(ncol=3, frameon=False)

    nominal = [
        sum(bool(submap.get("within_nominal_2_to_5m")) for submap in sessions[n]["submaps"])
        for n in names
    ]
    short = [len(sessions[n]["submaps"]) - nominal[i] for i, n in enumerate(names)]
    axes[2].bar(x, nominal, color="#3979a8", label="Nominal 2–5 m")
    axes[2].bar(x, short, bottom=nominal, color="#a477b4", label="Short boundary fragment")
    axes[2].set(title="Proposed overlapping submaps", ylabel="Submaps", xticks=x, xticklabels=names)
    axes[2].legend(ncol=2, frameon=False)
    for ax in axes:
        ax.grid(axis="y", alpha=0.22)
        ax.set_axisbelow(True)
        ax.tick_params(axis="x", rotation=12)
    fig.suptitle(
        "Actual quality-audit inventory | proposal only, not optimized and not navigation-approved",
        fontsize=15,
    )
    fig.savefig(path, dpi=190, metadata={"Title": "Quality groups and submap summary"})
    plt.close(fig)
    return {
        "sessions": names,
        "grade_totals": grade_totals,
        "usable_frames": dict(zip(names, usable)),
        "correlation_groups": dict(zip(names, groups)),
        "primary_submap_evidence_groups": dict(zip(names, primary_groups)),
        "nominal_submaps": dict(zip(names, nominal)),
        "short_submaps": dict(zip(names, short)),
    }


def _plot_metric(
    ax: plt.Axes,
    frames: list[dict[str, Any]],
    metric: str,
    title: str,
    ylabel: str,
    thresholds: list[tuple[float, str, str]],
) -> None:
    valid = [
        row
        for row in frames
        if np.isfinite(row["session_path_m"]) and np.isfinite(row[metric])
    ]
    if valid:
        ax.plot(
            [row["session_path_m"] for row in valid],
            [row[metric] for row in valid],
            color="#7b858f",
            linewidth=0.65,
            alpha=0.7,
            zorder=1,
        )
        grade_scatter(ax, valid, "session_path_m", metric)
    for value, label, color in thresholds:
        ax.axhline(value, color=color, linewidth=0.75, linestyle="--", alpha=0.8, label=label)
    for row in frames:
        if row["hard_boundary"] and np.isfinite(row["session_path_m"]):
            ax.axvline(row["session_path_m"], color="#111111", linewidth=0.35, alpha=0.2)
    ax.set(title=title, ylabel=ylabel)
    ax.grid(alpha=0.18)
    if thresholds:
        ax.legend(loc="upper right", ncol=min(4, len(thresholds)), fontsize=7, frameon=False)


def recorded_diagnostic_thresholds(input_thresholds: dict | None) -> dict[str, float]:
    """Extract plot lines from the audit manifest, with legacy defaults."""
    result = dict(DEFAULT_DIAGNOSTIC_THRESHOLDS)
    source = input_thresholds or {}
    mappings = {
        "score_a_min": ("grade_score", "A_min"),
        "score_b_min": ("grade_score", "B_min"),
        "intrinsic_a_max": ("intrinsic_jump_depth_pixels", "A_max"),
        "intrinsic_b_max": ("intrinsic_jump_depth_pixels", "B_max"),
        "intrinsic_hard": ("hard_boundary", "intrinsic_jump_depth_pixels"),
        "linear_a_max": ("linear_pose_increment_m", "A_max"),
        "linear_b_max": ("linear_pose_increment_m", "B_max"),
        "linear_hard": ("hard_boundary", "linear_pose_increment_m"),
        "angular_a_max": ("angular_pose_increment_deg", "A_max"),
        "angular_b_max": ("angular_pose_increment_deg", "B_max"),
        "angular_c_gate": ("angular_pose_increment_deg", "maximum"),
        "angular_hard": ("hard_boundary", "angular_pose_increment_deg"),
    }
    for output_key, (section, key) in mappings.items():
        value = source.get(section, {}).get(key)
        if value is not None:
            result[output_key] = float(value)
    return result


def render_session_diagnostics(
    session: str,
    frames: list[dict[str, Any]],
    path: Path,
    input_thresholds: dict | None = None,
) -> dict[str, Any]:
    thresholds = recorded_diagnostic_thresholds(input_thresholds)
    fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True, constrained_layout=True)
    _plot_metric(
        axes[0], frames, "quality_score", "Deterministic quality score", "Score / 100",
        [(thresholds["score_a_min"], "A minimum", GRADE_COLORS["A"]),
         (thresholds["score_b_min"], "B minimum", GRADE_COLORS["B"])],
    )
    _plot_metric(
        axes[1], frames, "intrinsic_jump_depth_px", "Per-frame intrinsics jump", "Depth pixels",
        [(thresholds["intrinsic_a_max"], "A max", "#238b45"),
         (thresholds["intrinsic_b_max"], "B max", "#e69123"),
         (thresholds["intrinsic_hard"], "Hard boundary", "#c63d3d")],
    )
    _plot_metric(
        axes[2], frames, "linear_step_m", "Frozen-pose translation increment", "Metres / frame",
        [(thresholds["linear_a_max"], "A max", "#238b45"),
         (thresholds["linear_b_max"], "B max", "#e69123"),
         (thresholds["linear_hard"], "Hard boundary", "#c63d3d")],
    )
    _plot_metric(
        axes[3], frames, "angular_step_deg", "Frozen-pose rotation increment", "Degrees / frame",
        [(thresholds["angular_a_max"], "A max", "#238b45"),
         (thresholds["angular_b_max"], "B max", "#e69123"),
         (thresholds["angular_c_gate"], "C gate", "#c06b42"),
         (thresholds["angular_hard"], "Hard boundary", "#8b1f1f")],
    )
    axes[-1].set_xlabel("Cumulative frozen-pose path (m)")
    fig.suptitle(
        f"{session} | actual per-frame audit versus path distance; vertical lines are hard boundaries",
        fontsize=14,
    )
    fig.savefig(path, dpi=190, metadata={"Title": f"{session} path-distance diagnostics"})
    plt.close(fig)
    maxima = {}
    for metric in ("quality_score", "intrinsic_jump_depth_px", "linear_step_m", "angular_step_deg"):
        values = np.asarray([row[metric] for row in frames], dtype=float)
        values = values[np.isfinite(values)]
        maxima[metric] = float(np.max(values)) if len(values) else None
    return {
        "session": session,
        "frame_count": len(frames),
        "hard_boundary_count": sum(row["hard_boundary"] for row in frames),
        "metrics": list(maxima),
        "observed_maxima": maxima,
        "x_axis": "session_path_m from the frozen baseline poses",
        "thresholds_from_input_manifest": thresholds,
    }


def create_output(path: Path) -> None:
    """Create a new evidence directory, never replacing an earlier render."""
    path.mkdir(parents=True, exist_ok=False)


def verify_inventory(output: Path, figures: dict[str, dict[str, Any]]) -> dict[str, str]:
    actual = {path.name for path in output.glob("*.png")}
    expected = set(figures)
    if actual != expected:
        raise ValueError(
            f"Rendered PNG inventory differs from manifest entries: actual={sorted(actual)}, "
            f"expected={sorted(expected)}"
        )
    return {name: sha256_file(output / name) for name in sorted(actual)}


def render(manifest_path: Path, output: Path) -> dict[str, Any]:
    input_manifest, sessions, input_files = load_quality_inputs(manifest_path)
    script_path = Path(__file__).resolve()
    source_hashes = {str(script_path): sha256_file(script_path)}
    upstream_source_record: dict[str, Any] | None = None
    upstream_source_value = input_manifest.get("source")
    if upstream_source_value:
        upstream_source = Path(upstream_source_value)
        if upstream_source.exists():
            upstream_source = upstream_source.resolve()
            current_digest = sha256_file(upstream_source)
            source_hashes[str(upstream_source)] = current_digest
            declared_digest = input_manifest.get("source_sha256")
            upstream_source_record = {
                "path": str(upstream_source),
                "declared_sha256": declared_digest,
                "current_sha256": current_digest,
                "matches_declared": declared_digest == current_digest,
            }
    input_hashes_before = {label: sha256_file(path) for label, path in input_files.items()}
    database_paths = {
        name: Path(data["summary"]["source_database"]).resolve()
        for name, data in sessions.items()
    }
    database_hashes_before = {name: sha256_file(path) for name, path in database_paths.items()}
    for name, digest in database_hashes_before.items():
        declared = sessions[name]["summary"].get("database_sha256_before")
        if declared and declared != digest:
            raise ValueError(f"Source database hash differs from quality manifest for {name}")

    create_output(output)
    figure_records: dict[str, dict[str, Any]] = {}
    summary_name = "01_quality_group_submap_summary.png"
    figure_records[summary_name] = {
        "caption": (
            "Direct summary of frame grades, correlation-aware evidence groups, and proposed "
            "overlapping submaps from the immutable quality-audit CSV/JSON artifacts."
        ),
        **render_summary(sessions, output / summary_name),
    }
    for index, (name, data) in enumerate(sessions.items(), start=2):
        trajectory_name = f"{index:02d}_{name}_trajectory_quality_submaps.png"
        figure_records[trajectory_name] = {
            "caption": (
                f"{name}: frozen baseline camera trajectory colored by actual A/B/C frame grade; "
                "black crosses are declared hard boundaries, rings are nominal anchors, and purple "
                "squares are short-fragment anchors. Camera Z is not a surveyed floor elevation."
            ),
            **render_session_trajectory(
                name, data["frames"], data["submaps"], output / trajectory_name
            ),
        }
    diagnostics_start = 2 + len(sessions)
    for offset, (name, data) in enumerate(sessions.items()):
        diagnostics_name = f"{diagnostics_start + offset:02d}_{name}_path_diagnostics.png"
        figure_records[diagnostics_name] = {
            "caption": (
                f"{name}: actual path-distance traces for deterministic frame score, intrinsics "
                "jump, translation step, and angular step. Thresholds are the recorded audit "
                "thresholds; this is diagnostic evidence, not optimized geometry."
            ),
            **render_session_diagnostics(
                name,
                data["frames"],
                output / diagnostics_name,
                input_manifest.get("thresholds"),
            ),
        }

    input_hashes_after = {label: sha256_file(path) for label, path in input_files.items()}
    database_hashes_after = {name: sha256_file(path) for name, path in database_paths.items()}
    if input_hashes_before != input_hashes_after:
        raise RuntimeError("A quality input artifact changed while rendering")
    if database_hashes_before != database_hashes_after:
        raise RuntimeError("A source database changed while rendering")

    figure_hashes = verify_inventory(output, figure_records)
    result = {
        "schema_version": 1,
        "created_utc": utc_now(),
        "status": "ACTUAL_DATA_QUALITY_VISUALIZATION_NOT_OPTIMIZED_NOT_ACCEPTED",
        "accepted": False,
        "navigation": False,
        "actual_data_only": True,
        "synthetic_or_ai_generated_imagery": False,
        "argv": sys.argv,
        "input_quality_manifest": {
            "path": str(manifest_path.resolve()),
            "sha256": input_hashes_before["quality_manifest"],
            "declared_status": input_manifest.get("status"),
        },
        "source_hashes": source_hashes,
        "upstream_quality_builder_source": upstream_source_record,
        "input_hashes": {
            label: {"path": str(input_files[label]), "sha256": digest}
            for label, digest in sorted(input_hashes_before.items())
        },
        "source_database_hashes_before": database_hashes_before,
        "source_database_hashes_after": database_hashes_after,
        "source_databases_unchanged": database_hashes_before == database_hashes_after,
        "quality_inputs_unchanged": input_hashes_before == input_hashes_after,
        "rendering": {
            "trajectory": (
                "plot frozen-pose XY and path-vs-camera-Z rows; break lines at declared hard "
                "boundaries; overlay A/B/C grades and declared submap anchors"
            ),
            "summary": "exact counts from frames.csv, session summaries, and submaps.json",
            "diagnostics": (
                "plot recorded score/intrinsics/translation/rotation columns against recorded "
                "cumulative path with audit thresholds"
            ),
            "dpi": 190,
        },
        "figures": figure_records,
        "figure_hashes": figure_hashes,
        "limitations": [
            "Frozen baseline poses are audit coordinates, not an optimized or surveyed trajectory.",
            "Camera Z is not a measured floor elevation and must not be used as a stair-height survey.",
            "Overlapping submaps and frames sharing a correlation group are not independent evidence.",
            "These plots do not establish cross-session alignment or navigation safety.",
        ],
    }
    (output / "manifest.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("outputs/student_union_20261017_submap_quality/manifest.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/student_union_20261017_submap_robust/figures_quality"),
    )
    args = parser.parse_args()
    result = render(args.manifest, args.output)
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "figures": len(result["figures"]),
                "accepted": result["accepted"],
                "navigation": result["navigation"],
            }
        )
    )


if __name__ == "__main__":
    main()
