#!/usr/bin/env python3
"""Reserve complete correlation-connected regions before a new fit.

This is a stricter diagnostic partition of historical data, not fresh ground
truth. Odometry remains in the objective, so sensor independence is not claimed.
No residual, optimized pose, or threshold outcome is used to select the split.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

from evaluate_region_holdouts import canonical_sha256, shared_evidence_regions


def reserve(graph, declaration, validation_graph):
    identity=validation_graph.get("metadata",{}).get("region_validation_view",{})
    if identity.get("source_canonical_manifest_sha256") != canonical_sha256(graph):
        raise ValueError("Region declaration view does not belong to this graph")
    regions={r["region_id"]:r for r in declaration["regions"]}
    shared=shared_evidence_regions(regions)
    locked=set()
    for key,row in regions.items():
        if row["locked_holdout"]: locked.update(shared[key])
    reserve_ids={str(e["id"]) for e in validation_graph["edges"]
                 if e["type"] != "odometry" and e["region_group"] in locked}
    result=copy.deepcopy(graph);changed=[]
    for edge in result["edges"]:
        if edge["id"] in reserve_ids and edge["role"] == "train":
            edge["role"]="heldout";changed.append(edge["id"])
            edge.setdefault("provenance",{})["strict_region_reservation"]={
                "previous_role":"train","selection":"preexisting heldout region or shared sensor correlation"}
    result["metadata"]["strict_region_reservation"]={
        "source_canonical_manifest_sha256":canonical_sha256(graph),
        "changed_edge_ids":sorted(changed),"reserved_edge_ids":sorted(reserve_ids),
        "locked_region_ids":sorted(locked),"uses_candidate_residuals":False,
        "accepted":False,"navigation":False,
        "limitation":"Historical repeatedly inspected observations; odometry retained; not sensor-independent ground truth."}
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest",type=Path,required=True)
    p.add_argument("--regions",type=Path,required=True)
    p.add_argument("--region-graph",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    graph=reserve(json.loads(a.manifest.read_text()),json.loads(a.regions.read_text()),
                  json.loads(a.region_graph.read_text()))
    graph["metadata"]["created_utc"]=datetime.now(timezone.utc).isoformat()
    a.output.mkdir(parents=True)
    (a.output/"manifest.json").write_text(json.dumps(graph,indent=2)+"\n")
    detail=graph["metadata"]["strict_region_reservation"]
    report={"created_utc":graph["metadata"]["created_utc"],"argv":sys.argv,
            "source_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "input_sha256":{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in [a.manifest,a.regions,a.region_graph]},**detail}
    (a.output/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    shutil.copyfile(__file__,a.output/"source_snapshot.py")
    print(json.dumps({"output":str(a.output),"moved_train_to_heldout":len(detail["changed_edge_ids"]),
                      "reserved_edges":len(detail["reserved_edge_ids"]),"accepted":False}))


if __name__ == "__main__":main()
