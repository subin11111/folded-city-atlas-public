#!/usr/bin/env python3
"""Verify actual flat sheet triangles, source preservation, stairs and semantics."""
import argparse,json
from pathlib import Path
import numpy as np
from audit_structural_delivery import glb_triangle_positions
from audit_continuous_planes import face_keys
from flatten_horizontal_surfaces import protected_points
from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());results={};cleanup={};objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    base_mp=Path(run['base_display_manifest']);base=json.loads(base_mp.read_text())
    for name,s in run['sessions'].items():
        folder=a.bundle/name;before=o3d.io.read_triangle_mesh(s['source_mesh']);before.compute_triangle_normals();mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'))
        actual=glb_triangle_positions(folder/'clean_textured.glb');xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
        if not np.array_equal(actual,xyz):raise ValueError('Actual GLB/PLY disagree: '+name)
        source_actual=glb_triangle_positions(Path(s['source_texture']));source_xyz=np.asarray(before.vertices)[np.asarray(before.triangles)].astype(np.float32)
        if not np.array_equal(source_actual,source_xyz):raise ValueError('Selected baseline GLB/PLY disagree: '+name)
        remove=np.load(folder/'removed_source_faces.npy');protected=np.load(folder/'protected_source_faces.npy')
        if len(remove)!=len(before.triangles) or (remove&protected).any():raise ValueError('Protected source faces were changed')
        retained=source_xyz[~remove];cursor=len(retained)
        if not np.array_equal(retained,xyz[:cursor]):raise ValueError('Unselected geometry/order changed: '+name)
        structures=np.load(folder/'vertical_structure_source_faces.npy')
        if (remove&structures).any():raise ValueError('Standing source wall/furniture geometry was removed')
        planes=[]
        for plane in s['flat_sheets']:
            if not plane['accepted']:continue
            count=plane['triangles'];added=xyz[cursor:cursor+count];cursor+=count
            z=added[:,:,2].astype(float);span=float(np.ptp(z));max_deviation=float(abs(z-plane['height_m']).max())
            if span>1e-6 or max_deviation>1e-6:raise ValueError('New sheet is not exactly horizontal')
            normal=np.cross(added[:,1]-added[:,0],added[:,2]-added[:,0]);direction=1 if plane['kind']=='floor' else -1
            if np.any(normal[:,2]*direction<=0):raise ValueError('Floor/ceiling face orientation incorrect')
            guard=s['flat_stair_guard']
            if protected_points(added.reshape(-1,3),np.asarray(guard['centres']),np.asarray(guard['half'])).any():raise ValueError('New sheet crosses protected stair corridor')
            pack=np.load(folder/plane['mask_file']);mask=pack['mask']
            if (mask&pack['forbidden']).any():raise ValueError('Sheet seals a protected opening')
            area=float(np.linalg.norm(normal,axis=1).sum()/2)
            if not np.isclose(area,plane['area_xy_m2'],rtol=1e-5):raise ValueError('Mask and actual mesh area disagree')
            planes.append({'id':plane['id'],'kind':plane['kind'],'height_m':plane['height_m'],'actual_triangle_height_span_m':span,
                'maximum_deviation_from_declared_plane_m':max_deviation,'area_xy_m2':area,'model_inferred_area_m2':plane['model_inferred_area_m2'],
                'triangles':count,'free_space_violations':0,'stair_guard_violations':0,'photo_fraction_on_sheet':plane['photo_fraction_on_sheet']})
        if cursor!=len(xyz):raise ValueError('Extra undocumented triangles')
        historical_stairs=None;stair_mask=base_mp.parent/name/'protected_stair_source_faces.npy'
        if stair_mask.exists():
            stair_source=o3d.io.read_triangle_mesh(base['sessions'][name]['source_mesh']);stair_ids=np.flatnonzero(np.load(stair_mask))
            if not np.isin(face_keys(stair_source,stair_ids),face_keys(mesh)).all():raise ValueError('Previously protected stair triangles were lost')
            historical_stairs={'triangles':len(stair_ids),'all_coordinates_retained':True}
        for obj in [o for o in objects if o['session']==name and o['class'] not in ['floor','ceiling'] and o.get('surface_added')]:
            ids=np.asarray(obj['current_triangle_indices']);g=obj['geometry']
            if len(ids)!=g['triangles'] or np.any(ids<0):raise ValueError('Semantic sign/door lost triangles')
        for obj in [o for o in objects if o['session']==name]:
            if sha256_file(folder/obj['source_rgb'])!=obj['source_rgb_sha256']:raise ValueError('Original semantic photograph changed')
        cleanup[name]={'removed_faces':int(remove.sum()),'before_faces':len(before.triangles),'after_faces':len(mesh.triangles),
            'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb'),
            'reason':'Replaced horizontal sheets only; source walls, semantic faces and stair guards retained.'}
        results[name]={'actual_GLB_PLY_equal_float32':True,'all_unselected_source_coordinates_retained':True,'protected_source_triangles':int(protected.sum()),
            'all_protected_source_coordinates_retained':True,'removed_source_faces':int(remove.sum()),'planes':planes,
            'historical_protected_main_stairs':historical_stairs,
            'standing_vertical_structure_faces_retained':int(structures.sum()),
            'steep_short_horizontal_layer_shards_removed':int(np.sum(remove&(abs(np.asarray(before.triangle_normals)[:,2])<=.30))),
            'outside_geometry_exactly_preserved':not remove.any() if name=='outside' else None}
        print('Verified flat sheets and retained source geometry: '+name,flush=True)
    (a.bundle/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,
        'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),
        'pose_changes':0,'navigation_ready':False,'scope':'Display-sheet planarity, area, guard and exact-coordinate preservation; not surveyed accuracy.'},indent=2)+'\n')


if __name__=='__main__':main()
