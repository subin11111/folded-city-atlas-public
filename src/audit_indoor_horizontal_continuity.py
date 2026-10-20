#!/usr/bin/env python3
"""Check actual continuity geometry, original retention and fixed-source rays."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage
from audit_structural_delivery import glb_triangle_positions
from audit_unified_presentation import embedded_images
from complete_service_slabs import mask_lookup
from flatten_horizontal_surfaces import protected_points
from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['base_display_manifest']).parent;prior=json.loads((base/'manifest.json').read_text());roof=np.load(a.bundle/'roof_mask.npz');mask=roof['mask'];observed=roof['observed'];assert not (mask&~ndimage.binary_fill_holes(observed)).any(),'Outward ceiling growth';cfg=run['horizontal_continuity_config'];model=LevelPresentation(ROOT,run['presentation_config']);guard_source=json.loads((ROOT/'outputs/student_union_verified_wall_delivery_v2/manifest.json').read_text());results={}
    import open3d as o3d
    before=o3d.t.geometry.RaycastingScene();after=o3d.t.geometry.RaycastingScene();floor_reports=[]
    for name,s in run['sessions'].items():
        old=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));new=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));old.compute_triangle_normals();old_xyz=np.asarray(old.vertices)[np.asarray(old.triangles)].astype(np.float32);xyz=np.asarray(new.vertices)[np.asarray(new.triangles)].astype(np.float32);keep=np.load(a.bundle/name/'retained_source_faces.npy');count=s['retained_source_triangles'];assert np.array_equal(xyz[:count],old_xyz[keep]);assert np.array_equal(xyz,glb_triangle_positions(a.bundle/name/'clean_textured.glb'));images=embedded_images(base/name/'clean_textured.glb');assert images==embedded_images(a.bundle/name/'clean_textured.glb')[:len(images)];categories=np.load(a.bundle/name/'removal_categories.npz');assert np.array_equal(~keep,categories['floor']|categories['roof']);normal=np.asarray(old.triangle_normals);assert (abs(normal[~keep,2])>.5).all(),'Standing vertical structure removed';centres=old_xyz.mean(1)
        if name in ['lobby','floor2','bridge_12_sub']:
            guard=guard_source['sessions'][name]['flat_stair_guard'];c=np.asarray(guard['centres']);half=np.asarray(guard['half']);protected=np.zeros(len(keep),bool)
            if len(c):protected=protected_points(old_xyz.reshape(-1,3),model.warp(c,name),half+[.1,.1,.5]).reshape(-1,3).any(1)
            tread_band=(centres[:,2]<model.upper+.25)&(centres[:,2]>model.lower-.25);assert keep[protected&tread_band].all(),'Stair/landing geometry changed'
        cursor=count
        for layer in s['horizontal_continuity']:
            triangles=xyz[cursor:cursor+layer['triangles']];deviation=float(abs(triangles[:,:,2]-layer['height_m']).max(initial=0));assert deviation<2e-6
            if layer['kind']=='ceiling':
                assert len(triangles)==int(mask.sum())*2
                indices=np.floor((triangles.mean(1)[:,:2]-roof['low'])/roof['cell']).astype(int);coverage=np.zeros(mask.shape,int);np.add.at(coverage,(indices[:,1],indices[:,0]),1);assert np.all(coverage[mask]==2) and not coverage[~mask].any()
            floor_reports.append({'id':layer['id'],'kind':layer['kind'],'height_m':layer['height_m'],'height_deviation_m':deviation,'triangles':len(triangles)});cursor+=layer['triangles']
        if name=='lobby':
            heading=next(o for o in prior['sessions']['lobby']['semantic_objects'] if o['id']=='lobby_shinhan_heading');start=prior['sessions']['lobby']['retained_source_triangles']+sum(s['triangles'] for s in prior['sessions']['lobby']['service_slabs']);assert keep[start:start+heading['geometry']['triangles']].all(),'Bank heading changed'
        before.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(old));after.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(new));results[name]={'retained_source_XYZ_exact':True,'actual_GLB_PLY_equal':True,'original_embedded_image_streams_preserved':len(images),'removed_floor_overlap_triangles':int(categories['floor'].sum()),'removed_roof_fragment_triangles':int(categories['roof'].sum()),'standing_vertical_structure_retained':True,'bank_heading_retained':True if name=='lobby' else None,'ply_sha256':sha256_file(a.bundle/name/'clean_structural.ply'),'glb_sha256':sha256_file(a.bundle/name/'clean_textured.glb')};print('Verified continuous horizontal surfaces: '+name,flush=True)
    y,x=np.where(mask);rng=np.random.default_rng(124);ids=rng.choice(len(x),min(len(x),8000),replace=False);xy=roof['low']+np.c_[x[ids]+.43,y[ids]+.57]*roof['cell'];origin=np.c_[xy,np.full(len(xy),float(roof['height'])-.7)];direction=np.tile([0,0,1.],(len(xy),1));rays=o3d.core.Tensor(np.c_[origin,direction].astype(np.float32));diagnostic={}
    for name,scene in [('before',before),('after',after)]:
        distance=scene.cast_rays(rays)['t_hit'].numpy();hit=np.isfinite(distance)&(distance<1.5);delta=abs(distance[hit]-.7);diagnostic[name]={'same_selected_XY_rays':len(xy),'surface_hit_in_1p5m_fraction':float(hit.mean()),'first_hit_within_2cm_of_display_roof_fraction':float((np.isfinite(distance)&(abs(distance-.7)<.02)).mean()),'first_hit_height_gap_p90_m':float(np.percentile(delta,90)) if len(delta) else None}
    floor_masks={}
    for layer in cfg['floor_layers']:
        d=np.load(a.bundle/(layer['id']+'_floor_mask.npz'));assert not (d['holes']&(~d['allowed']|d['forbidden'])).any();assert not (d['holes']&~ndimage.binary_fill_holes(d['coverage'])).any();floor_masks[layer['id']]={'added_area_m2':float(d['holes'].sum()*d['cell']**2),'no_outward_growth_or_protected_opening_fill':True}
    for f,h in run['inputs_sha256'].items():assert sha256_file(Path(f))==h,'Input binding changed: '+f
    ptr=json.loads((ROOT/'config/current_map.json').read_text());assert sha256_file(ROOT/ptr['manifest'])==run['selected_manifest_sha256'];selected=json.loads((ROOT/ptr['manifest']).read_text());database_hashes={n:sha256_file(Path(v['database'])) for n,v in selected['database_integrity'].items()};assert all(database_hashes[n]==v['sha256'] for n,v in selected['database_integrity'].items());result={'created_utc':utc_now(),'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'selected_map_unchanged':True,'source_database_hashes_unchanged':database_hashes,'input_bindings_valid':True,'sessions':results,'new_plane_geometry':floor_reports,'ceiling_mask':{'area_m2':float(mask.sum()*roof['cell']**2),'bounded_hole_fill_m2':float((mask&~observed).sum()*roof['cell']**2),'outside_photographed_outer_envelope_cells':0,'every_ceiling_cell_has_two_actual_triangles':True,'connected_components':int(ndimage.label(mask)[1])},'floor_masks':floor_masks,'fixed_upward_ray_diagnostic':diagnostic,'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Actual surface self-consistency and retention checks on the selected observation area. Not surveyed ceiling accuracy, independent SLAM evaluation or navigation approval.'};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
