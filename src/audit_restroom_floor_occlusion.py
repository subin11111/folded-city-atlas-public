#!/usr/bin/env python3
"""Find modeled-floor cells hiding an opaque fascia in two original photographs."""
import argparse,json
from pathlib import Path
import numpy as np
from audit_structural_delivery import glb_triangle_positions
from complete_verified_upper_walls import inside_polygon
from optimize_session import sha256_file
from surface_model_utils import project

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--floor-bundle',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());cfg=json.loads((ROOT/'config/verified_upper_wall_completion.json').read_text());base=a.floor_bundle or ROOT/cfg['source_bundle'];prior=json.loads((base/'manifest.json').read_text())
    import open3d as o3d
    scene=o3d.t.geometry.RaycastingScene();lookup={}
    for name,s in prior['sessions'].items():
        xyz=glb_triangle_positions(base/name/'clean_textured.glb');cursor=s['retained_previous_prefix_faces']
        for plane in s['flat_sheets']:
            if not plane.get('accepted'):continue
            count=plane['triangles'];points=xyz[cursor:cursor+count];indices=np.arange(cursor,cursor+count);cursor+=count
            mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(points.reshape(-1,3));mesh.triangles=o3d.utility.Vector3iVector(np.arange(count*3).reshape(-1,3));gid=scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
            pack=np.load(base/name/plane['mask_file']);cells=np.floor((points.mean(1)[:,:2]-pack['low'])/float(pack['cell'])).astype(int);lookup[gid]=(name,plane,indices,cells)
    wall=run['sessions']['lobby'];xyz=glb_triangle_positions(a.bundle/'lobby/clean_textured.glb');start=wall['original_source_faces'];wall_points=[]
    for region in wall['upper_wall_review']:
        count=region.get('added_triangles',0) if region.get('accepted') else 0
        if region['kind']=='upper_wall' and count:wall_points.append(xyz[start:start+count].mean(1))
        start+=count
    points=np.concatenate(wall_points)
    road=json.loads((ROOT/'outputs/student_union_roadview_v1/roadview.json').read_text())['nodes'];view_polygons={713:cfg['wall_regions'][0]['polygon'],167:[[0,0],[719,0],[719,65],[610,80],[480,103],[340,142],[0,184]]};votes={};view_counts={}
    for node,polygon in view_polygons.items():
        v=next(v for v in road if v['session']=='lobby' and v['node']==node);pose=np.asarray(v['world_optical']);uv,z=project(points,np.asarray(v['K']),pose);valid=(z>.25)&inside_polygon(uv,polygon);q=points[valid];vector=q-pose[:3,3];distance=np.linalg.norm(vector,axis=1);rays=np.c_[np.broadcast_to(pose[:3,3],q.shape),vector/distance[:,None]].astype(np.float32);hit=scene.cast_rays(o3d.core.Tensor(rays));t=hit['t_hit'].numpy();bad=np.flatnonzero(np.isfinite(t)&(t<distance-.20));view_counts[node]=len(bad)
        gids=hit['geometry_ids'].numpy();faces=hit['primitive_ids'].numpy();seen=set()
        for i in bad:
            name,plane,indices,cells=lookup[int(gids[i])];ij=cells[faces[i]];key=(name,plane['id'],int(ij[0]),int(ij[1]));seen.add(key)
        for key in seen:votes.setdefault(key,[]).append(node)
    approved=[{'session':k[0],'plane':k[1],'x':k[2],'y':k[3],'original_photo_nodes':v} for k,v in votes.items() if len(v)>=2]
    result={'view_conflict_rays':view_counts,'two_view_contradicted_modeled_floor_cells':approved,'count':len(approved),
        'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'floor_manifest_sha256':sha256_file(base/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),
        'scope':'Two original opaque-wall photographs + photo-bounded wall display hypothesis, not independent surveyed free-space or a raw-depth-only test.'}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='two_view_contradicted_modeled_floor_cells'},flush=True)


if __name__=='__main__':main()
