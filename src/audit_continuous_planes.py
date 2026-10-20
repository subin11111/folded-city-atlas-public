#!/usr/bin/env python3
"""Check continuous-plane deliverables, opening masks and retained stair/floor faces."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
from scipy import ndimage
from optimize_session import sha256_file,utc_now
from audit_structural_delivery import glb_triangle_positions
from surface_model_utils import project


def face_keys(mesh,ids=None):
    xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles) if ids is None else np.asarray(mesh.triangles)[ids]].astype('<f4')
    return np.ascontiguousarray(xyz).reshape(-1,9).view(np.dtype((np.void,36))).ravel()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--finalize',action='store_true')
    p.add_argument('--archive-reconstruction-source',action='store_true')
    p.add_argument('--allow-reviewed-horizontal-replacement',action='store_true');a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());results={};cleanup={}
    for n,s in run['sessions'].items():
        folder=a.bundle/n;ply=folder/'structural.ply';glb=folder/'structural_textured.glb'
        mesh=o3d.io.read_triangle_mesh(str(ply));actual=glb_triangle_positions(glb)
        xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
        if not np.array_equal(actual,xyz):raise ValueError('GLB/PLY triangle order disagreement: '+n)
        before=o3d.io.read_triangle_mesh(str(a.source/n/'clean_structural.ply'));before.compute_triangle_normals()
        horizontal=np.flatnonzero(abs(np.asarray(before.triangle_normals)[:,2])>.90)
        present=np.isin(face_keys(before,horizontal),face_keys(mesh))
        reviewed_horizontal=0
        if not present.all():
            if not a.allow_reviewed_horizontal_replacement:raise ValueError('Horizontal stair/floor/ceiling triangles lost: '+n)
            missing=horizontal[~present];tri=np.asarray(before.triangles)[missing];centres=np.asarray(before.vertices)[tri].mean(1)
            covered=np.zeros(len(missing),bool)
            for obj in s.get('semantic_objects',[]):
                if obj['class'] not in ['floor','ceiling'] or not obj.get('surface_added'):continue
                geometry=obj['geometry'];normal=np.asarray(geometry['plane_normal']);centre=np.asarray(geometry['plane_centre'])
                uv,z=project(centres,np.asarray(obj['K_rgb']),np.asarray(obj['world_optical']))
                import cv2
                poly=np.asarray(obj['polygon'],np.float32)
                inside=np.array([cv2.pointPolygonTest(poly,tuple(p),True)>=-1.5 for p in uv.astype(np.float32)])
                covered|=inside&(z>.2)&(abs((centres-centre)@normal)<.101)
            if not covered.all():raise ValueError('Horizontal changes outside reviewed photo surfaces: '+n)
            reviewed_horizontal=len(missing)
        removal_path=folder/'removed_source_faces.npy'
        retained=None
        stair_protection=None
        if (folder/'protected_stair_source_faces.npy').exists():
            stair_ids=np.flatnonzero(np.load(folder/'protected_stair_source_faces.npy'))
            if not np.isin(face_keys(before,stair_ids),face_keys(mesh)).all():raise ValueError('Protected stair corridor changed')
            stair_protection={'source_triangles':len(stair_ids),'all_coordinates_retained':True}
            # Also check the previous stage's complete saved stair mask. A
            # fresh spatial selection alone must not silently narrow it.
            historical_mask=a.source/n/'protected_stair_source_faces.npy'
            historical_summary=a.source/n/'summary.json'
            if historical_mask.exists() and historical_summary.exists():
                summary=json.loads(historical_summary.read_text());historical_source=o3d.io.read_triangle_mesh(summary['source_mesh'])
                historical_ids=np.flatnonzero(np.load(historical_mask))
                if not np.isin(face_keys(historical_source,historical_ids),face_keys(mesh)).all():raise ValueError('Historical protected stair coordinates changed')
                stair_protection['historical_source_triangles']=len(historical_ids)
                stair_protection['all_historical_coordinates_retained']=True
        if removal_path.exists():
            removed=np.load(removal_path)
            if len(removed)!=len(before.triangles):raise ValueError('Source face mask length mismatch')
            retained=bool(np.isin(face_keys(before,np.flatnonzero(~removed)),face_keys(mesh)).all())
            if not retained:raise ValueError('Unselected source geometry changed: '+n)
            actual_horizontal_removed=int(np.sum(removed & (abs(np.asarray(before.triangle_normals)[:,2])>.985)))
            if a.finalize and s.get('horizontal_faces_removed')!=actual_horizontal_removed:
                s['creation_stage_horizontal_faces_removed_report']=s.get('horizontal_faces_removed')
                s['horizontal_faces_removed']=actual_horizontal_removed
                s['metric_correction']='Recomputed from saved source face mask and reloaded source normals; creation stage held a mutable Open3D array view.'
        masks=[]
        for path in sorted(folder.glob('mask_*.npz')):
            m=np.load(path);mask=m['mask'];support=m['support'];free=m['free'];obs=m['observed']
            forbidden=(free>=2)&(free>=1.5*(support+1))
            added=mask&~obs
            if np.any(mask&forbidden) or np.any(added&(support<2)):raise ValueError('Opening/gap support guard failed: '+str(path))
            if 'stair_guard' in m.files and np.any(mask&m['stair_guard']):raise ValueError('New wall intrudes into stair corridor')
            masks.append({'file':path.name,'sha256':sha256_file(path),'cells':int(mask.sum()),'added_supported_cells':int(added.sum()),'free_space_violations':0,
                'observed_raster_components':ndimage.label(obs)[1],'rebuilt_raster_components':ndimage.label(mask)[1],
                'area_m2':float(mask.sum()*.06**2),'added_supported_area_m2':float(added.sum()*.06**2)})
        results[n]={'triangles':len(mesh.triangles),'GLB_PLY_equal_float32':True,'original_horizontal_triangles':len(horizontal),
            'all_original_horizontal_triangle_coordinates_retained':bool(present.all()),'horizontal_normal_abs_z_minimum':.90,
            'horizontal_faces_replaced_inside_reviewed_photo_surfaces':reviewed_horizontal,
            'horizontal_geometry_retained_except_reviewed_replacements':True,
            'reviewed_photo_polygon_raster_tolerance_px':1.5,
            'all_unselected_source_triangles_retained':retained,'plane_masks':masks,
            'protected_stair_corridor':stair_protection,
            'semantic_door_window_labels':bool(run.get('semantic_display')),
            'opening_preservation_scope':('Base wall masks retain measured free space; added photo semantic surfaces explicitly record the reference door/glazing state.' if run.get('semantic_display') else 'Measured free-space and unknown unsupported gaps; not a semantic/survey guarantee.')}
        if a.finalize:
            for src,dst in [(ply,folder/'clean_structural.ply'),(glb,folder/'clean_textured.glb')]:
                if dst.exists():
                    if sha256_file(src)!=sha256_file(dst):raise ValueError('Existing final differs')
                else:shutil.copyfile(src,dst)
            cleanup[n]={'removed_faces':0,'before_faces':len(mesh.triangles),'after_faces':len(mesh.triangles),
                'reason':'Source was already cleaned; no global tiny-face removal to preserve retained stairs/details.',
                'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb')}
        print('Preserved horizontal geometry and validated plane masks: '+n,flush=True)
    if a.finalize:
        if a.archive_reconstruction_source:
            source=Path(__file__).resolve().parent/'reconstruct_continuous_planes.py'
            snapshot=a.bundle.resolve()/'reconstruction_source_snapshot.py'
            if str(source) in run['inputs_sha256']:
                if sha256_file(source)!=run['inputs_sha256'][str(source)]:raise ValueError('Runtime source changed before archiving')
                shutil.copyfile(source,snapshot)
                run['inputs_sha256'][str(snapshot)]=run['inputs_sha256'].pop(str(source))
                run.setdefault('historical_source_snapshots',{})[str(source)]=str(snapshot)
        for n,s in run['sessions'].items():(a.bundle/n/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
        (a.bundle/'manifest.json').write_text(json.dumps(run,indent=2)+'\n')
        (a.bundle/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'audit_source_sha256':sha256_file(Path(__file__)),'pose_changes':0,'navigation_ready':False},indent=2)+'\n')


if __name__=='__main__':main()
