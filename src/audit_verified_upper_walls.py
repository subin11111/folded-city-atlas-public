#!/usr/bin/env python3
"""Audit exact source preservation and the bounded new upper-wall/junction geometry."""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from audit_structural_delivery import glb_triangle_positions
from complete_verified_upper_walls import inside_polygon
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from surface_model_utils import decode_frame,project

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());results={}
    import open3d as o3d
    for name,s in run['sessions'].items():
        before=o3d.io.read_triangle_mesh(s['source_mesh']);after=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));old=np.asarray(before.vertices)[np.asarray(before.triangles)].astype(np.float32);xyz=np.asarray(after.vertices)[np.asarray(after.triangles)].astype(np.float32);cursor=s['original_source_faces'];checks=[]
        source_keep=np.ones(len(old),bool)
        if run.get('modeled_floor_occlusion_trim'):
            source_keep=np.load(a.bundle/name/'retained_pre_occlusion_faces.npy')[:len(old)]
        if cursor!=source_keep.sum() or not np.array_equal(xyz[:cursor],old[source_keep]):raise ValueError('Existing nonwithdrawn wall/floor/stair coordinates changed')
        if not np.array_equal(xyz,glb_triangle_positions(a.bundle/name/'clean_textured.glb')):raise ValueError('Active GLB/PLY mismatch')
        poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        for region in s['upper_wall_review']:
            if not region.get('accepted'):checks.append({'id':region['id'],'accepted':False,'reason':region['reason']});continue
            count=region['added_triangles'];added=xyz[cursor:cursor+count];cursor+=count
            if not count:continue
            f=decode_frame(con,region['node'],selected['sessions'][name]['preprocessing'],poses[region['node']]);centre=added.mean(1);uv,z=project(centre,f['k_rgb'],f['world_optical'])
            if not inside_polygon(uv,region['polygon']).all():raise ValueError('New face exceeds the reviewed photo region')
            normal=np.cross(added[:,1]-added[:,0],added[:,2]-added[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
            check={'id':region['id'],'accepted':True,'actual_triangles':count,'area_m2':region['area_m2'],'all_centroids_inside_reviewed_photo':True,'height_min_m':float(added[:,:,2].min()),'height_max_m':float(added[:,:,2].max())}
            if region['kind']=='verified_local_ceiling':
                if min(abs(normal[:,2]))<.98:raise ValueError('Ceiling is not the measured near-horizontal plane')
                if max(region['wall_top_junction_disagreement_m'])>.002:raise ValueError('New ceiling does not meet the measured wall-top line')
                check['joint_residual_construction_m']=region['wall_top_junction_disagreement_m'];check['scope']='Shared line is imposed by construction; not independent alignment accuracy.'
            else:
                if np.percentile(abs(normal[:,2]),95)>.3:raise ValueError('A wall patch became a horizontal cap')
                if 'maximum_opaque_z_m' in region and added[:,:,2].max()>region['maximum_opaque_z_m']+.003:raise ValueError('Opaque wall crosses measured glazing boundary')
            checks.append(check)
        con.close()
        if cursor!=len(xyz):raise ValueError('Undocumented new faces')
        results[name]={'all_nonwithdrawn_original_coordinates_order_retained':True,'original_faces':len(old),'withdrawn_modeled_floor_faces':int((~source_keep).sum()),'all_original_wall_and_stair_coordinates_retained':True,'actual_GLB_PLY_equal':True,'pose_changes':0,'regions':checks}
        print('Verified source geometry + photo-bounded upper walls: '+name,flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Original geometry preservation and bounded display model; no surveyed accuracy, whole-building completion or floor-height correction claimed.'},indent=2)+'\n')


if __name__=='__main__':main()
