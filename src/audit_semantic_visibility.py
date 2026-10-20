#!/usr/bin/env python3
"""First-hit visibility of appended photo surfaces, not just their existence."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects'];results=[]
    for name in sorted(set(o['session'] for o in objects)):
        chosen=[o for o in objects if o['session']==name]
        mesh=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        scene=o3d.t.geometry.RaycastingScene(nthreads=2);scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        start=len(mesh.triangles)-sum(o.get('geometry',{}).get('triangles',0) for o in chosen if o.get('surface_added'))
        for o in chosen:
            if not o.get('surface_added'):continue
            g=o['geometry'];stop=start+g['triangles'];k=np.asarray(o['K_rgb']);pose=np.asarray(o['world_optical']);poly=np.asarray(o['polygon'],np.float32)
            low=poly.min(0).astype(int);high=poly.max(0).astype(int);y,x=np.mgrid[low[1]:high[1]+1:3,low[0]:high[0]+1:3]
            uv=np.c_[x.ravel(),y.ravel()];inside=np.array([cv2.pointPolygonTest(poly,tuple(p),True)>2 for p in uv.astype(np.float32)]);uv=uv[inside]
            direction=np.c_[(uv[:,0]-k[0,2])/k[0,0],(uv[:,1]-k[1,2])/k[1,1],np.ones(len(uv))]@pose[:3,:3].T
            origin=np.broadcast_to(pose[:3,3],direction.shape);rays=np.c_[origin,direction].astype(np.float32)
            hit=scene.cast_rays(o3d.core.Tensor(rays));ids=hit['primitive_ids'].numpy();t=hit['t_hit'].numpy()
            n=np.asarray(g['plane_normal']);c=np.asarray(g['plane_centre']);toward=np.sign((pose[:3,3]-c)@n);c=c+n*toward*g['display_offset_m']
            expected=((c-pose[:3,3])@n)/(direction@n);visible=(ids>=start)&(ids<stop)
            delta=(t-expected)[np.isfinite(t)]
            results.append({'id':o['id'],'session':name,'class':o['class'],'sampled_polygon_rays':len(uv),
                'own_surface_first_hit_fraction':float(visible.mean()) if len(uv) else None,
                'finite_render_fraction':float(np.isfinite(t).mean()) if len(uv) else None,
                'first_hit_minus_photo_plane_m_quantiles':np.quantile(delta,[.1,.5,.9]).tolist() if len(delta) else None,
                'actual_semantic_face_start':start,'actual_semantic_face_stop':stop})
            start=stop
        print('Checked first-hit semantic visibility: '+name,flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'created_utc':utc_now(),'objects':results,
        'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),
        'scope':'Whether each appended surface is the first geometry hit in its own reference view; not measured pose accuracy.'},indent=2)+'\n')


if __name__=='__main__':main()
