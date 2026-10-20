#!/usr/bin/env python3
"""Check no invented ceiling vertices, unchanged floors and existing wall heights."""
import argparse,json
from pathlib import Path
import numpy as np
from audit_continuous_planes import face_keys
from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file,utc_now
from withdraw_inferred_ceilings import retained_without_modeled_ceilings


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['base_display_manifest']).parent;prior=json.loads((base/'manifest.json').read_text());results={}
    if not run.get('inferred_ceilings_disabled'):raise ValueError('Inferred ceilings are not disabled')
    for name,s in run['sessions'].items():
        before=o3d.io.read_triangle_mesh(s['source_mesh']);after=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));raw=o3d.io.read_triangle_mesh(s['observed_ceiling_source_ply'])
        old=np.asarray(before.vertices)[np.asarray(before.triangles)].astype(np.float32);xyz=np.asarray(after.vertices)[np.asarray(after.triangles)].astype(np.float32)
        keep=retained_without_modeled_ceilings(len(old),s['retained_previous_prefix_faces'],prior['sessions'][name]['flat_sheets']);restore=np.load(a.bundle/name/'restored_observed_faces.npy')
        original=np.asarray(raw.vertices)[np.asarray(raw.triangles)].astype(np.float32)
        if not np.array_equal(xyz[:keep.sum()],old[keep]) or not np.array_equal(xyz[keep.sum():],original[restore]):raise ValueError('Wall/floor height changed or a ceiling vertex was invented')
        if not np.array_equal(xyz,glb_triangle_positions(a.bundle/name/'clean_textured.glb')):raise ValueError('GLB does not match audited PLY')
        if any(p0.get('accepted') and p0['kind']=='ceiling' for p0 in s['flat_sheets']):raise ValueError('Modeled ceiling remains accepted')
        # Trace even the residual original-prefix ceiling-band faces, rather
        # than only checking the newly restored geometry.
        before.compute_triangle_normals();pre=s['retained_previous_prefix_faces'];centres=old[:pre].mean(1);candidate=np.zeros(pre,bool)
        for p0 in prior['sessions'][name]['flat_sheets']:
            if p0['kind']=='ceiling':candidate|=(centres[:,2]>=p0['height_band_m'][0])&(centres[:,2]<p0['height_band_m'][1])
        candidate&=abs(np.asarray(before.triangle_normals)[:pre,2])>.65
        present=np.isin(face_keys(before,np.flatnonzero(candidate)),face_keys(raw))
        if not present.all():raise ValueError('An older modeled ceiling remains in the retained prefix')
        results[name]={'all_existing_floor_wall_stair_coordinates_unchanged':True,'actual_GLB_PLY_equal':True,
            'all_residual_ceiling_band_horizontal_faces_are_original_TSDF_subset':True,'residual_observed_ceiling_band_faces':int(candidate.sum()),
            'restored_original_TSDF_faces':int(restore.sum()),'removed_modeled_ceiling_faces':int((~keep).sum()),
            'ceiling_height_estimates_created':0,'restoration_vertex_displacement_m':0.,'new_inferred_ceiling_triangles':0}
        print('Verified no inferred ceilings or wall/floor height changes: '+name,flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,
        'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),
        'scope':'Exact existing depth-based TSDF triangle subsets only, not surveyed truth; no arbitrary ceiling completion or height.'},indent=2)+'\n')


if __name__=='__main__':main()
