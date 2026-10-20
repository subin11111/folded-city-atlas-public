#!/usr/bin/env python3
"""Optional viewing derivative hiding only the newly modeled ceiling sheets."""
import argparse,json
from pathlib import Path
import numpy as np
from clean_structural_fragments import filter_glb
from optimize_session import sha256_file


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());results={}
    for name,s in run['sessions'].items():
        folder=a.bundle/name;mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'));planes=[v for v in s['flat_sheets'] if v['accepted']]
        keep=np.ones(len(mesh.triangles),bool);cursor=len(keep)-sum(v['triangles'] for v in planes)
        for plane in planes:
            if plane['kind']=='ceiling':keep[cursor:cursor+plane['triangles']]=False
            cursor+=plane['triangles']
        filter_glb(folder/'clean_textured.glb',folder/'cutaway_textured.glb',keep);mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();o3d.io.write_triangle_mesh(str(folder/'cutaway_structural.ply'),mesh)
        results[name]={'hidden_ceiling_faces':int((~keep).sum()),'glb_sha256':sha256_file(folder/'cutaway_textured.glb'),'ply_sha256':sha256_file(folder/'cutaway_structural.ply')}
    (a.bundle/'cutaway.json').write_text(json.dumps({'sessions':results,'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'scope':'Visibility derivative only; full floors/ceilings remain in clean_textured.glb.'},indent=2)+'\n')


if __name__=='__main__':main()
