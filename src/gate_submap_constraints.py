#!/usr/bin/env python3
"""Freeze an initial-residual DCS gate before robust pose-graph fitting.

Ordinary IRLS DCS can re-enable a bad loop after the graph deforms to satisfy
it.  This diagnostic policy computes compatibility once at the frozen initial
poses and moves low-weight loop/cross-session constraints to a
selection-conditioned diagnostic role.  They remain visible as ``heldout`` but
must never be interpreted as independent validation because their exclusion
used their own residual.  Odometry and structural continuity assumptions are
never gated here.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import sys

from optimize_robust_submaps import _edge_chi2, parse_manifest, sha256_file, utc_now


def dcs_information_weight(chi2, phi):
    if not math.isfinite(chi2) or chi2 < 0 or not math.isfinite(phi) or phi <= 0:
        raise ValueError('DCS chi2 must be finite/nonnegative and phi finite/positive')
    scale = min(1., 2. * phi / (phi + chi2))
    return scale * scale


def gate_manifest(manifest, threshold=.1, phi=5.991):
    if not math.isfinite(threshold) or not 0 < threshold <= 1 or not math.isfinite(phi) or phi <= 0:
        raise ValueError("threshold must be in (0,1] and phi must be positive")
    nodes, edges = parse_manifest(manifest)
    initial = {node.id: node.initial for node in nodes}
    output = copy.deepcopy(manifest)
    by_id = {edge["id"]: edge for edge in output["edges"]}
    records = []
    for edge in edges:
        if not edge.used_for_fit or edge.edge_type not in {"loop", "cross_session"}:
            continue
        _error, chi2 = _edge_chi2(edge, initial)
        weight = dcs_information_weight(chi2, phi)
        gated = weight < threshold
        record = {
            "edge_id": edge.id, "type": edge.edge_type,
            "region_group": edge.region_group, "initial_chi2": chi2,
            "initial_dcs_information_weight": weight, "gated_from_fit": gated,
        }
        records.append(record)
        if gated:
            row = by_id[edge.id]
            row["role"] = "heldout"
            row.setdefault("provenance", {}).update({
                "selection_conditioned_diagnostic": True,
                "independent_holdout": False,
                "original_role": edge.role,
                "selection_policy": "frozen-initial DCS gate; not independent heldout evidence",
                "initial_chi2": chi2,
                "initial_dcs_information_weight": weight,
                "gate_threshold": threshold,
                "dcs_phi": phi,
            })
    output.setdefault("metadata", {})["frozen_initial_dcs_gate"] = {
        "threshold": threshold, "dcs_phi": phi,
        "eligible_types": ["loop", "cross_session"],
        "odometry_gated": False, "structural_transition_gated": False,
        "selection_conditioned_diagnostics_are_independent_holdouts": False,
        "initial_pose_compatibility_is_not_ground_truth": True,
        "chi2_information_policy": "raw edge information before physical-region 1/N normalization",
        "records": records,
        "gated_edge_ids": [row["edge_id"] for row in records if row["gated_from_fit"]],
    }
    output["metadata"]["accepted"] = False
    output["metadata"]["navigation"] = False
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threshold", type=float, default=.1)
    parser.add_argument("--dcs-phi", type=float, default=5.991)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    payload = json.loads(args.manifest.read_text())
    gated = gate_manifest(payload, args.threshold, args.dcs_phi)
    gated["metadata"]["frozen_initial_dcs_gate"].update({
        "created_utc": utc_now(), "source_manifest": str(args.manifest.resolve()),
        "source_manifest_sha256": sha256_file(args.manifest),
        "source_sha256": sha256_file(Path(__file__)),
    })
    args.output.mkdir(parents=True)
    shutil.copyfile(Path(__file__), args.output / "source_snapshot.py")
    (args.output / "manifest.json").write_text(json.dumps(gated, indent=2) + "\n")
    report = {
        "created_utc": utc_now(), "argv": sys.argv if argv is None else argv,
        "manifest_sha256": sha256_file(args.output / "manifest.json"),
        "gated_edge_ids": gated["metadata"]["frozen_initial_dcs_gate"]["gated_edge_ids"],
        "accepted": False, "navigation": False,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), **report}))


if __name__ == "__main__":
    main()
