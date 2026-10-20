#!/usr/bin/env python3
"""Directional/sequence inventory, never a panorama or walkability approval."""
import argparse
import json
from pathlib import Path
import numpy as np


def directional_union(nodes,bins=72):
    azimuth=(np.arange(bins)+.5)*360/bins;seen=np.zeros(bins,bool)
    for node in nodes:
        heading=node['heading_in_map_deg'];half=node['horizontal_fov_deg']/2
        delta=(azimuth-heading+180)%360-180
        seen|=abs(delta)<half
    return float(seen.mean())


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('pack',type=Path);a=p.parse_args()
    road=json.loads((a.pack/'roadview.json').read_text());results={}
    for session in sorted({n['session'] for n in road['nodes']}):
        nodes=[n for n in road['nodes'] if n['session']==session];cells={}
        for n in nodes:
            key=tuple(np.floor(np.asarray(n['position_map_m'])/[2.,2.,.75]).astype(int))
            cells.setdefault(key,[]).append(n)
        coverage=[directional_union(v) for v in cells.values()]
        edges=[e for e in road['edges'] if e['from'].startswith(session+':')]
        distances=[e['distance_3d_m'] for e in edges]
        results[session]={'photos':len(nodes),'capture_edges':len(edges),'spatial_cells':len(cells),
            'horizontal_direction_union_median':float(np.median(coverage)),
            'cells_below_50pct_horizontal_directions':int(np.sum(np.array(coverage)<.5)),
            'capture_step_p95_m':float(np.percentile(distances,95)) if distances else None,
            'capture_step_max_m':float(max(distances)) if distances else None}
    report={'sessions':results,'perspective_not_360':True,'surveyed_navigation':False,'privacy_review_complete':False,
        'OSM_placement_surveyed':False,'automatic_cross_session_transitions':0,
        'directional_coverage_limits':'Approximate horizontal direction union within 2m XY / 0.75m Z bins; not spherical coverage, visibility, same camera centre or panorama stitchability.',
        'release_blockers':['No true 360 captures; keep perspective viewer','Review cross-session/floor transitions against photographs',
            'Redact faces and plates, manually review privacy','Confirm coarse GPS placement with surveyed/known control points',
            'Validate walkable routes independently of capture order','Unobserved geometry must remain unknown for navigation']}
    (a.pack/'coverage_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
