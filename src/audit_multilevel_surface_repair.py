#!/usr/bin/env python3
"""Audit actual topology-fixed triangles, shared heights and frozen source geometry."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage
from audit_structural_delivery import glb_triangle_positions
from audit_multilevel_topology import stats
from flatten_horizontal_surfaces import protected_points
from reconcile_multilevel_surfaces import sample_lattice,remap_mask
from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['previous_flat_manifest']).parent;cfg=run['topology_config']
    results={};cleanup={};layers={};objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    for name,s in run['sessions'].items():
        folder=a.bundle/name;before=o3d.io.read_triangle_mesh(s['source_mesh']);mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'))
        xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32);actual=glb_triangle_positions(folder/'clean_textured.glb')
        if not np.array_equal(actual,xyz):raise ValueError('GLB/PLY mismatch: '+name)
        source_xyz=np.asarray(before.vertices)[np.asarray(before.triangles)].astype(np.float32);keep=np.load(folder/'retained_previous_faces.npy');prefix=int(s['retained_previous_prefix_faces'])
        if not keep[:prefix].all() or keep[prefix:].any() or not np.array_equal(source_xyz[keep],xyz[:prefix]):raise ValueError('Original retained geometry changed')
        cursor=prefix;planes=[]
        for plane in s['flat_sheets']:
            if not plane.get('accepted'):continue
            count=plane['triangles'];added=xyz[cursor:cursor+count];cursor+=count;pack=np.load(folder/plane['mask_file']);mask=pack['mask'];cell=float(pack['cell'])
            lattice=dict(np.load(a.bundle/plane['anchor_file']));expected=sample_lattice(lattice,added[:,:,:2].reshape(-1,2)).reshape(-1,3)
            error=float(abs(added[:,:,2]-expected).max())
            if error>2e-6:raise ValueError('Mesh is detached from shared anchors: '+plane['id'])
            normal=np.cross(added[:,1]-added[:,0],added[:,2]-added[:,0]);direction=1 if plane['kind']=='floor' else -1
            if np.any(normal[:,2]*direction<=0):raise ValueError('Incorrect sheet orientation')
            slope=np.rad2deg(np.arctan2(np.linalg.norm(normal[:,:2],axis=1),abs(normal[:,2])))
            if slope.max()>cfg['local_plane_slope_limit_deg']+.005:raise ValueError('Actual mesh slope exceeds bound')
            g=s['flat_stair_guard']
            if protected_points(added.reshape(-1,3),np.asarray(g['centres']),np.asarray(g['half'])).any():raise ValueError('New mesh crosses stair guard')
            if (mask&pack['forbidden']).any() or (mask&pack['atrium_guard']).any():raise ValueError('New sheet seals a protected void')
            if plane['id'] in cfg['low_ceiling_layers'] and np.any(mask&(ndimage.distance_transform_edt(~pack['observed'])*cell>cfg['low_ceiling_maximum_gap_fill_m']+1e-9)):
                raise ValueError('Lower ceiling inherited unsupported floor footprint')
            area=float(abs(normal[:,2]).sum()/2)
            if not np.isclose(area,mask.sum()*cell*cell,rtol=1e-5):raise ValueError('Mask / actual XY area mismatch')
            centres=added.mean(1);ij=np.floor((centres[:,:2]-pack['low'])/cell).astype(int)
            if not mask[ij[:,1],ij[:,0]].all():raise ValueError('Triangle outside selected mask')
            planes.append({'id':plane['id'],'triangles':count,'height_span_m':float(np.ptp(added[:,:,2])),
                'maximum_slope_deg':float(slope.max()),'anchor_error_max_m':error,'area_xy_m2':area,
                'atrium_covered_cells':0,'protected_stair_vertices':0,'source_fit_p90_m':plane['source_anchor_residual_p90_m'],
                'scope':'Fit residual, NOT held-out or surveyed accuracy.'})
            layers[plane['id']]={'pack':{k:pack[k] for k in pack.files},'lattice':lattice,'plane':plane}
        if cursor!=len(xyz):raise ValueError('Undocumented triangles')
        for obj in [o for o in objects if o['session']==name]:
            if sha256_file(folder/obj['source_rgb'])!=obj['source_rgb_sha256']:raise ValueError('Original semantic RGB changed')
            if obj.get('surface_added') and obj['class'] not in ['floor','ceiling']:
                ids=np.asarray(obj['current_triangle_indices'])
                if ids.max()>=prefix or not np.array_equal(xyz[ids],source_xyz[ids]):raise ValueError('Mapped door/window/sign changed')
        cleanup[name]={'removed_faces':int((~keep).sum()),'before_faces':len(keep),'after_faces':len(xyz),
            'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb'),
            'reason':'Replaced only prior modeled horizontal sheets; original retained walls/stairs/semantic faces exact.'}
        results[name]={'GLB_PLY_equal_float32':True,'retained_original_prefix_faces':prefix,'retained_prefix_coordinates_exact':True,
            'all_mapped_vertical_semantics_and_original_RGB_retained':True,'planes':planes,'outside_exactly_unchanged':not (~keep).any() if name=='outside' else None}
        print('Verified multilevel topology and retained source: '+name,flush=True)
    first,second=layers['floor2_floor_2f'],layers['sub_floor_2f'];pa=first['pack'];pb=second['pack'];yy,xx=np.nonzero(pa['mask']);xy=pa['low']+np.c_[xx+.5,yy+.5]*float(pa['cell'])
    ij=np.floor((xy-pb['low'])/float(pb['cell'])).astype(int);x,y=ij.T;h,w=pb['mask'].shape;ok=(x>=0)&(x<w)&(y>=0)&(y<h);ii=np.flatnonzero(ok);ii=ii[pb['mask'][y[ii],x[ii]]];xy=xy[ii]
    gap=sample_lattice(second['lattice'],xy)-sample_lattice(first['lattice'],xy)
    if not len(gap) or abs(gap).max()>1e-9:raise ValueError('Shared upper-floor sheets disagree at the same XY')
    old=json.loads((base/'manifest.json').read_text());old_levels={p['id']:p['height_m'] for s in old['sessions'].values() for p in s['flat_sheets'] if p.get('accepted')}
    comparisons={'upper_floor_main_vs_sub':{'shared_model_gap_at_same_XY':stats(gap),'maximum_absolute_model_gap_m':float(abs(gap).max()),
        'previous_independent_constant_gap_m':old_levels['sub_floor_2f']-old_levels['floor2_floor_2f'],
        'scope':'Equality is enforced by construction at common XY; NOT independent accuracy or a new pose registration result.'}}
    (a.bundle/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'comparisons':comparisons,
        'atrium_removed_from_lower_ceilings_m2_sum_overlapping_sessions':sum(p.get('atrium_removed_area_m2',0) for s in run['sessions'].values() for p in s['flat_sheets'] if p['id'] in cfg['low_ceiling_layers']),
        'pose_changes':0,'navigation_ready':False,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),
        'scope':'Modeled display topology and exact source preservation; no automatic surveyed building model.'},indent=2)+'\n')


if __name__=='__main__':main()
