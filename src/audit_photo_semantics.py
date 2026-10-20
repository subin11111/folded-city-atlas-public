#!/usr/bin/env python3
"""Independently verify original RGB, plane anchors, and actual semantic mesh faces."""
import argparse,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
from export_rtabmap_db import load_pose_csv
from surface_model_utils import decode_frame,unproject,project
from optimize_session import apply,sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects'];results=[];rejected=[]
    run=json.loads((a.bundle/'manifest.json').read_text())
    for name,s in selected['sessions'].items():
        chosen=[o for o in objects if o['session']==name]
        if not chosen:continue
        poses=load_pose_csv(mp.parent/s['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        cloud=o3d.io.read_point_cloud(str(mp.parent/s['cloud']));tree=cKDTree(np.asarray(cloud.points))
        mesh=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));vertices=np.asarray(mesh.vertices);triangles=np.asarray(mesh.triangles)
        source_mesh=o3d.io.read_triangle_mesh(run['sessions'][name]['source_mesh']);source_mesh.compute_triangle_normals()
        source_centres=np.asarray(source_mesh.vertices)[np.asarray(source_mesh.triangles)].mean(1)
        cursor=len(triangles)-sum(o.get('geometry',{}).get('triangles',0) for o in chosen if o.get('surface_added'))
        for o in chosen:
            if not o.get('geometry',{}).get('accepted'):
                rejected.append({'id':o['id'],'reason':o.get('geometry',o['plane_fit'])});continue
            frame=decode_frame(con,o['node'],s['preprocessing'],poses[o['node']])
            png=a.bundle/name/o['source_rgb'];actual=np.asarray(Image.open(png).convert('RGB'))
            if not np.array_equal(actual,frame['rgb']) or sha256_file(png)!=o['source_rgb_sha256']:raise ValueError('Original RGB mismatch: '+o['id'])
            if not np.allclose(frame['world_optical'],o['world_optical'],atol=1e-10):raise ValueError('Source pose mismatch')
            g=o['geometry'];boundary=np.asarray(g['world_boundary'])
            # The DB calibration rotation is float32 and very slightly
            # non-orthonormal. Use the full matrix inverse for this algebraic
            # reprojection check instead of assuming R^-1 == R.T.
            optical=apply(boundary,np.linalg.inv(frame['world_optical']));pixel_h=optical@frame['k_rgb'].T
            uv=pixel_h[:,:2]/pixel_h[:,2:3]
            reprojection=float(np.linalg.norm(uv-np.asarray(o['polygon']),axis=1).max())
            if reprojection>1e-5:raise ValueError('Semantic polygon ray-plane mismatch')
            normal=np.asarray(g['plane_normal']);centre=np.asarray(g['plane_centre'])
            support_kind=o['plane_fit']['support_source']
            mapped_support=support_kind in ['existing_local_map_horizontal_faces','existing_local_map_vertical_faces']
            if mapped_support:
                ids=np.asarray(o['plane_fit']['map_support_face_indices'],int)
                horizontal=support_kind=='existing_local_map_horizontal_faces'
                if len(ids)<(30 if horizontal else 6) or np.any(ids<0) or np.any(ids>=len(source_centres)):
                    raise ValueError('Invalid local map plane support: '+o['id'])
                supported=source_centres[ids]
                if horizontal and (o['class'] not in ['floor','ceiling'] or np.any(abs(np.asarray(source_mesh.triangle_normals)[ids,2])<.9) or abs(normal[2])<.95):
                    raise ValueError('Non-horizontal support for floor/ceiling')
                if not horizontal and (o['class']!='sign' or np.any(abs(np.asarray(source_mesh.triangle_normals)[ids,2])>.5) or abs(normal[2])>.5):
                    raise ValueError('Invalid existing display wall decal support')
                if np.percentile(abs((supported-centre)@normal),90)>(.06 if horizontal else .03):
                    raise ValueError('Mapped plane residual too large')
                # These are reloaded existing-map faces, not claimed sensor
                # samples in this particular photo (the ceiling has no depth).
                uv_support,depth_support=project(supported,frame['k_rgb'],frame['world_optical'])
                padding=o['plane_fit'].get('context_padding_rgb_pixels',0);poly=np.asarray(o['polygon'],np.float32)
                distance=np.array([cv2.pointPolygonTest(poly,tuple(p),True) for p in uv_support.astype(np.float32)])
                if np.any(depth_support<.25) or np.any(distance < -padding*np.sqrt(2)-1.5):
                    raise ValueError('Mapped plane support outside reviewed local context')
            else:
                mask=np.zeros(frame['rgb'].shape[:2],np.uint8)
                for poly in o.get('plane_support_polygons',[o['polygon']]):cv2.fillPoly(mask,[np.asarray(poly,np.int32)],1)
                for poly in o.get('colour_exclusions',[]):cv2.fillPoly(mask,[np.asarray(poly,np.int32)],0)
                h,w=frame['depth'].shape;small=cv2.resize(mask,(w,h),interpolation=cv2.INTER_NEAREST)>0
                if 'nearby' in support_kind:small=cv2.dilate(small.astype(np.uint8),np.ones((11,11),np.uint8))>0
                points=apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical'])
                valid=(frame['depth'].ravel()>0)&small.ravel()&(abs((points-centre)@normal)<.08)
                supported=points[valid]
            distances=tree.query(supported[::max(1,len(supported)//250)],workers=2)[0]
            nn=float(np.median(distances)) if len(distances) else None
            if len(supported)<(6 if support_kind=='existing_local_map_vertical_faces' else 12) or nn is None or (nn>.15 and support_kind!='existing_local_map_vertical_faces'):
                raise ValueError('Semantic anchor lacks mapped support: '+o['id']+' / '+str(nn))
            face_count=0
            if o['surface_added']:
                face_count=g['triangles'];xyz=vertices[triangles[cursor:cursor+face_count]];cursor+=face_count
                normals=np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]);facing=np.sum(normals*(frame['world_optical'][:3,3]-xyz.mean(1)),axis=1)
                if np.any(facing<-1e-10):raise ValueError('Semantic face points away from its source camera')
                area=np.linalg.norm(normals,axis=1).sum()/2
                if not np.isclose(area,g['area_m2'],rtol=1e-5):raise ValueError('Semantic face range disagreement')
            if o.get('state','').startswith('open_') and o['surface_added']:raise ValueError('Open aperture was sealed')
            results.append({'id':o['id'],'class':o['class'],'state':o.get('state'),'surface_added':o['surface_added'],
                'source_RGB_pixels_equal_DB':True,'source_rgb_sha256':sha256_file(png),'maximum_boundary_reprojection_px':reprojection,
                'support_kind':support_kind,'independent_reloaded_support_points':len(supported),
                'independent_reloaded_RGBD_support_points':len(supported) if not mapped_support else 0,
                'selected_map_nn_median_m':nn,'actual_mesh_faces':face_count})
        if cursor!=len(triangles):raise ValueError('Semantic triangle count/order mismatch')
        con.close();print('Verified RGB, mapped anchors and semantic faces: '+name,flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'created_utc':utc_now(),'objects':results,'rejected_candidates':rejected,
        'mapped_objects':len(results),'added_surfaces':sum(o['surface_added'] for o in results),
        'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'auditor_sha256':sha256_file(Path(__file__)),
        'scope':'Construction and provenance checks on visually reviewed photo annotations; no global semantic or surveyed accuracy claim.'},indent=2)+'\n')


if __name__=='__main__':main()
