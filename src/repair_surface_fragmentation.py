#!/usr/bin/env python3
"""Independent display-only continuity candidates with measured-support guards.

Never modifies selected poses, source maps or original RGB-D databases. Poisson
interpolates geometry: reduced fragment counts are NOT improved registration.
"""
import argparse
import copy
import gc
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
from scipy.spatial import cKDTree

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, sha256_file, utc_now
from reconstruct_surface_portfolio import clean_mesh, mesh_stats, texture_mesh
from refine_surface_display import srgb_to_linear, standardize_glb
from surface_model_utils import decode_frame, project, write_glb

ROOT=Path(__file__).resolve().parents[1]


def support_mask(vertices, triangles, tree, distance=.10, maximum_edge=.30):
    """All vertices AND face centres must lie near measured reference points."""
    v=np.asarray(vertices);t=np.asarray(triangles)
    vertex_distance=tree.query(v,workers=2)[0]
    centre_distance=tree.query(v[t].mean(axis=1),workers=2)[0]
    edges=np.max(np.stack([np.linalg.norm(v[t[:,i]]-v[t[:,j]],axis=1)
                          for i,j in [(0,1),(1,2),(2,0)]]),axis=0)
    return (vertex_distance[t].max(axis=1)<=distance)&(centre_distance<=distance)&(edges<=maximum_edge)


def free_space_reject(free_votes, visible_votes, minimum=2, fraction=.6):
    free=np.asarray(free_votes);visible=np.asarray(visible_votes)
    return (free>=minimum)&(free>=fraction*np.maximum(visible,1))


def components(mesh):
    labels,counts,areas=mesh.cluster_connected_triangles()
    counts=np.asarray(counts);areas=np.asarray(areas)
    return np.asarray(labels),counts,areas


def topology(mesh):
    labels,counts,areas=components(mesh)
    return {'components':len(counts),'triangles':len(mesh.triangles),
        'surface_area_m2':float(areas.sum()),'components_smaller_than_002m2':int(np.sum(areas<.02)),
        'small_component_area_fraction':float(areas[areas<.02].sum()/max(areas.sum(),1e-12)),
        'largest_component_area_fraction':float(areas.max()/areas.sum()) if len(areas) else 0.}


def prune(mesh,minimum_area):
    labels,counts,areas=components(mesh)
    keep=areas>=minimum_area
    if not np.any(keep):keep[np.argmax(areas)]=True
    report={'removed_components':int(np.sum(~keep)),'removed_triangles':int(counts[~keep].sum()),
        'removed_area_m2':float(areas[~keep].sum()),'minimum_component_area_m2':minimum_area}
    mesh.remove_triangles_by_mask(~keep[labels]);mesh.remove_unreferenced_vertices()
    return clean_mesh(mesh),report


def display_copy(mesh,budget):
    if len(mesh.triangles)>budget:
        mesh=mesh.simplify_quadric_decimation(budget,boundary_weight=10.)
    return clean_mesh(mesh)


def free_space_guard(mesh,frames,tolerance=.12,relative=.02):
    centres=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].mean(axis=1)
    votes=np.zeros(len(centres),np.int32);visible=np.zeros(len(centres),np.int32)
    for frame in frames.values():
        depth=frame['depth'];h,w=depth.shape;uv,z=project(centres,frame['k_depth'],frame['world_optical'])
        xy=np.rint(np.clip(uv,-1e5,1e5)).astype(int);x,y=xy.T
        ids=np.flatnonzero((z>.25)&(x>=0)&(x<w)&(y>=0)&(y<h))
        measured=depth[y[ids],x[ids]];tol=tolerance+relative*z[ids]
        # Occluded surfaces are not counted as visible evidence.
        known=(measured>0)&(z[ids]<=measured+tol)
        visible[ids[known]]+=1
        free=known&(z[ids]<measured-tol);votes[ids[free]]+=1
    reject=free_space_reject(votes,visible)
    report={'centroid_faces_checked':len(centres),'rejected_faces':int(reject.sum()),
        'minimum_free_space_votes':2,'minimum_free_fraction':.6,'absolute_tolerance_m':tolerance,
        'relative_tolerance':relative,'scope':'Same-capture face-centre guard; not proof that openings/stair clearance are correct'}
    mesh.remove_triangles_by_mask(reject);mesh.remove_unreferenced_vertices()
    return clean_mesh(mesh),report


def export_rgb(mesh,path):
    return write_glb(path,np.asarray(mesh.vertices),np.asarray(mesh.triangles),
        np.asarray(mesh.vertex_normals),srgb_to_linear(np.asarray(mesh.vertex_colors)))


def camera_metrics(o3d,mesh,frame):
    scene=o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    depth=frame['depth'];k=frame['k_depth'];yy,xx=np.indices(depth.shape)
    directions=np.stack([(xx-k[0,2])/k[0,0],(yy-k[1,2])/k[1,1],np.ones_like(xx)],axis=-1)
    directions=directions@frame['world_optical'][:3,:3].T
    origins=np.broadcast_to(frame['world_optical'][:3,3],directions.shape)
    rays=np.concatenate([origins,directions],axis=-1).astype(np.float32)
    predicted=scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
    valid=depth>.25;hit=np.isfinite(predicted)&(predicted>.25);both=valid&hit
    delta=predicted[both]-depth[both];tol=.12+.02*depth[both]
    return {'camera_node':frame['id'],'valid_depth_pixels':int(valid.sum()),
        'surface_coverage_on_valid_depth':float(np.sum(both)/max(valid.sum(),1)),
        'median_absolute_depth_error_m':float(np.median(abs(delta))) if len(delta) else None,
        'p90_absolute_depth_error_m':float(np.percentile(abs(delta),90)) if len(delta) else None,
        'free_space_conflict_on_hits':float(np.mean(delta < -tol)) if len(delta) else None,
        'scope':'Same capture/poses; coverage can improve by inventing surfaces. Compare error and support too.'}


def build(o3d,name,selected,mp,a,hashes,cfg):
    started=time.monotonic();out=a.output/name;out.mkdir()
    source=a.run/name;record=selected['sessions'][name]
    source_summary=json.loads((source/'summary.json').read_text())
    paths=[source/'tsdf_3cm.ply',source/'tsdf_5cm.ply',source/'tsdf_3cm_display.ply',
        mp.parent/record['cloud'],mp.parent/record['trajectory'],Path(selected['database_integrity'][name]['database'])]
    for path in paths:hashes[str(path.resolve())]=sha256_file(path)
    if sha256_file(paths[-1])!=selected['database_integrity'][name]['sha256']:raise ValueError('DB changed')
    poses=load_pose_csv(paths[-2]);reference=o3d.io.read_point_cloud(str(paths[-3]));tree=cKDTree(np.asarray(reference.points))
    db=sqlite3.connect(f'file:{paths[-1]}?mode=ro',uri=True)
    frames={n:decode_frame(db,n,record['preprocessing'],poses[n]) for n in source_summary['texture']['camera_frames']}
    primary=cfg['representative_frames'][name][0]
    unused=sorted(set(poses)-set(frames));audit=min(unused,key=lambda n:abs(n-primary)) if unused else primary
    probes={label:decode_frame(db,n,record['preprocessing'],poses[n]) for label,n in [('source',primary),('audit',audit)]}
    db.close()
    baseline=o3d.io.read_triangle_mesh(str(source/'tsdf_3cm_display.ply'))
    results={'baseline_180k':{'topology':topology(baseline),
        'camera_probes':{label:camera_metrics(o3d,baseline,f) for label,f in probes.items()}}}
    del baseline
    print(name+': component cleanup and larger display budget',flush=True)
    fine=o3d.io.read_triangle_mesh(str(source/'tsdf_3cm.ply'))
    results['original_full_topology']=topology(fine)
    fine,pruned=prune(fine,.02)
    fine=display_copy(fine,a.display_budget)
    o3d.io.write_triangle_mesh(str(out/'clean_high_budget.ply'),fine)
    export_rgb(fine,out/'clean_high_budget.glb')
    results['clean_high_budget']={'pruning':pruned,'topology':topology(fine),
        'camera_probes':{label:camera_metrics(o3d,fine,f) for label,f in probes.items()},
        'metrics':mesh_stats(o3d,fine,reference,20261017)}
    del fine;gc.collect()
    print(name+': support-constrained Poisson',flush=True)
    coarse=o3d.io.read_triangle_mesh(str(source/'tsdf_5cm.ply'));coarse,_=prune(coarse,.02)
    coarse.compute_vertex_normals();pc=o3d.geometry.PointCloud()
    pc.points=coarse.vertices;pc.normals=coarse.vertex_normals;pc.colors=coarse.vertex_colors
    pc=pc.voxel_down_sample(.06)
    if len(pc.points)>300000:
        indices=np.linspace(0,len(pc.points)-1,300000).astype(int);pc=pc.select_by_index(indices)
    input_count=len(pc.points);del coarse;gc.collect()
    mesh,density=o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc,depth=10,scale=1.03,
        linear_fit=True,n_threads=2)
    initial_faces=len(mesh.triangles);density=np.asarray(density)
    mesh.remove_vertices_by_mask(density<np.quantile(density,.01));mesh=clean_mesh(mesh)
    keep=support_mask(np.asarray(mesh.vertices),np.asarray(mesh.triangles),tree)
    unsupported=int((~keep).sum());mesh.remove_triangles_by_mask(~keep);mesh=clean_mesh(mesh)
    mesh,guard=free_space_guard(mesh,frames);mesh,pruned=prune(mesh,.02)
    vertices=np.asarray(mesh.vertices);_,nearest=tree.query(vertices,workers=2)
    mesh.vertex_colors=o3d.utility.Vector3dVector(np.asarray(reference.colors)[nearest])
    o3d.io.write_triangle_mesh(str(out/'supported_poisson_full.ply'),mesh)
    results['supported_poisson_full']={'topology':topology(mesh),'metrics':mesh_stats(o3d,mesh,reference,20261017)}
    mesh=display_copy(mesh,a.display_budget)
    # Decimation can bridge a boundary, so repeat measured-support/free-space checks.
    keep=support_mask(np.asarray(mesh.vertices),np.asarray(mesh.triangles),tree)
    mesh.remove_triangles_by_mask(~keep);mesh=clean_mesh(mesh)
    mesh,display_guard=free_space_guard(mesh,frames);mesh,display_pruned=prune(mesh,.02)
    o3d.io.write_triangle_mesh(str(out/'supported_poisson.ply'),mesh);export_rgb(mesh,out/'supported_poisson.glb')
    results['supported_poisson']={'input_oriented_points':input_count,'initial_poisson_faces':initial_faces,
        'density_bottom_quantile_removed':.01,'unsupported_faces_removed_before_decimation':unsupported,
        'maximum_support_distance_m':.10,'maximum_triangle_edge_m':.30,
        'free_space_guard':guard,'after_decimation_guard':display_guard,'pruning':pruned,
        'display_pruning':display_pruned,'topology':topology(mesh),
        'camera_probes':{label:camera_metrics(o3d,mesh,f) for label,f in probes.items()},
        'metrics':mesh_stats(o3d,mesh,reference,20261017)}
    print(name+': depth-checked native photo texture',flush=True)
    texture=texture_mesh(mesh,frames,cfg['texture'],out);assignment=texture.pop('camera_assignment')
    np.save(out/'texture_frame_assignment.npy',np.asarray(assignment,np.int32))
    standardize_glb(out/'photo_textured.glb',out/'supported_poisson_textured.glb')
    results['texture']=texture
    results.update(session=name,pose_changes=0,source_map_replaced=False,navigation_ready=False,
        display_budget=a.display_budget,seconds=time.monotonic()-started,
        camera_nodes={label:f['id'] for label,f in probes.items()},
        outputs={p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in out.iterdir() if p.is_file()})
    (out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps({'complete':name,'baseline_components':results['baseline_180k']['topology']['components'],
        'clean_components':results['clean_high_budget']['topology']['components'],
        'poisson_components':results['supported_poisson']['topology']['components']}),flush=True)
    return results


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=ROOT/'outputs/student_union_surface_portfolio_v1')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--sessions',nargs='+')
    p.add_argument('--display-budget',type=int,default=600000)
    p.add_argument('--resume',action='store_true');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(exist_ok=True,parents=True)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());cfg=json.loads((ROOT/'config/student_union_surface_portfolio.json').read_text())
    manifest_path=a.output/'manifest.json'
    previous=json.loads(manifest_path.read_text()) if a.resume and manifest_path.exists() else {}
    hashes=previous.get('inputs_sha256',{})
    for path in [pointer_path,mp,Path(__file__),ROOT/'src/surface_model_utils.py',
                 ROOT/'src/reconstruct_surface_portfolio.py',ROOT/'src/refine_surface_display.py']:
        h=sha256_file(path)
        if str(path.resolve()) in hashes and hashes[str(path.resolve())]!=h:raise ValueError('Source changed during resume')
        hashes[str(path.resolve())]=h
    results=previous.get('sessions',{})
    for name in a.sessions or selected['sessions']:
        if name in results:continue
        results[name]=build(o3d,name,selected,mp,a,hashes,cfg)
        for path,h in hashes.items():
            if sha256_file(Path(path))!=h:raise ValueError('Original input changed: '+path)
        manifest={'created_utc':utc_now(),'sessions':results,'inputs_sha256':hashes,
            'all_inputs_unchanged':True,'source_map_replaced':False,'navigation_ready':False,
            'limitations':['Poisson interpolation is new display geometry, not measured truth.',
                'Small component removal may remove real isolated thin signs. Original map is retained.',
                'Face-centre free-space checks are incomplete; door/stair clearance needs separate validation.',
                'No pose optimization and no Gaussian retraining in this repair experiment.']}
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()
