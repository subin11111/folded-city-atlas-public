#!/usr/bin/env python3
"""Keep colour correction separate from unverified hole caps; freeze diagnostics."""
import argparse
import json
from pathlib import Path
import numpy as np

from optimize_session import sha256_file,utc_now
from repair_surface_fragmentation import export_rgb


def graph_components(count,edges):
    parent=list(range(count));degree=np.zeros(count,int)
    def find(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for edge in edges:
        i,j=edge['i'],edge['j'];parent[find(i)]=find(j);degree[i]+=1;degree[j]+=1
    return len({find(i) for i in range(count)}),int(np.sum(degree==0))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);a=p.parse_args()
    import open3d as o3d
    original=json.loads((a.bundle/'manifest.json').read_text());report={}
    for n,s in original['sessions'].items():
        folder=a.bundle/n;path=folder/'balanced_only.glb'
        if path.exists():raise FileExistsError(path)
        mesh=o3d.io.read_triangle_mesh(str(folder/'appearance_fixed.ply'));flag=np.load(folder/'inferred_faces.npy')
        mesh.remove_triangles_by_mask(flag);mesh.compute_vertex_normals();export_rgb(mesh,path)
        count=len(s['gains']);components,isolated=graph_components(count,s['exposure'].get('pair_constraints',[]))
        report[n]={'default_display':'balanced_only.glb','default_faces_added':0,'original_vertices_preserved':True,
            'optional_hole_candidate':'appearance_fixed.glb','inferred_marked_candidate':'inferred_marked.glb',
            'exposure_frames':count,'exposure_graph_components':components,'unconstrained_exposure_frames':isolated,
            'colour_coverage_fraction':s['vertex_colour_coverage'],'small_hole_trial':{k:v for k,v in s['hole_repair'].items() if k!='groups'},
            'reserved_before':s['exposure']['reserved_before'],'reserved_after':s['exposure']['reserved_after'],
            'sha256':sha256_file(path)}
        print(json.dumps({'finalized':n,'exposure_graph_components':components,'unconstrained_frames':isolated}),flush=True)
    (a.bundle/'review.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':report,
        'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'selected_map_changed':False,
        'policy':'Default changes colour only. Hole caps remain explicit optional inferred geometry, not approved navigation or a complete hole repair.',
        'not_solved':['Large/unobserved gaps','Inconsistent/doubled geometry','Physical shadows and reflections','High-resolution texture seams','Cross-session exposure consistency']},indent=2)+'\n')


if __name__=='__main__':main()
