#!/usr/bin/env python3
"""Display-only small-hole repair and robust photometric exposure compensation.

Added faces are explicitly inferred, not navigation geometry. Camera/SLAM poses
and original databases remain unchanged. Texture colours are baked continuously
per vertex; native photographs remain the detail-preserving roadview source.
"""
import argparse
import gc
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np

from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from refine_surface_display import srgb_to_linear
from repair_surface_fragmentation import export_rgb,topology
from surface_model_utils import decode_frame,project

ROOT=Path(__file__).resolve().parents[1]


def linear_to_srgb(value):
    value=np.clip(value,0,1)
    return np.where(value<=.0031308,12.92*value,1.055*value**(1/2.4)-.055)


def visible_samples(points,normals,frame):
    uv,z=project(points,frame['k_depth'],frame['world_optical']);h,w=frame['depth'].shape
    x,y=np.rint(np.clip(uv,-1e5,1e5)).astype(int).T
    ids=np.flatnonzero((z>.25)&(z<4.2)&(x>=0)&(x<w)&(y>=0)&(y<h))
    measured=frame['depth'][y[ids],x[ids]]
    direction=frame['world_optical'][:3,3]-points[ids]
    cosine=np.abs(np.sum(normals[ids]*direction,axis=1))/np.maximum(np.linalg.norm(direction,axis=1),1e-9)
    gate=(measured>0)&(abs(z[ids]-measured)<.08+.02*z[ids])&(cosine>.3)
    ids=ids[gate];cosine=cosine[gate]
    uv_rgb,_=project(points[ids],frame['k_rgb'],frame['world_optical'])
    ih,iw=frame['rgb'].shape[:2];xx,yy=np.rint(np.clip(uv_rgb,-1e5,1e5)).astype(int).T
    valid=(xx>=0)&(xx<iw)&(yy>=0)&(yy<ih);ids=ids[valid];cosine=cosine[valid]
    colours=frame['rgb'][yy[valid],xx[valid]]/255.
    weights=cosine**3/np.maximum(z[ids],.5)**2
    return ids,colours,weights


def solve_log_gains(observations,training_mask):
    n,_,_=observations.shape;rows=[];ratios=[];weights=[];records=[]
    valid=np.isfinite(observations).all(2)&(observations.min(2)>.015)&(observations.max(2)<.85)
    for i in range(n):
        for j in range(i+1,n):
            a=observations[i];b=observations[j]
            good=valid[i]&valid[j]&training_mask
            if good.sum()<40:continue
            logs=np.log(b[good]) - np.log(a[good]);target=np.median(logs,axis=0)
            row=np.zeros(n);row[i]=1;row[j]=-1
            rows.append(row);ratios.append(target);weights.append(min(good.sum()/100,10.))
            records.append({'i':i,'j':j,'training_points':int(good.sum())})
    if not rows:return np.ones((n,3)),{'edges':0,'gains_identifiable':False}
    a=np.asarray(rows);b=np.asarray(ratios);base=np.asarray(weights);g=np.zeros((n,3))
    # Zero-centred prior preserves the session's exposure level; no frame is
    # globally brightened merely to make an image look more complete.
    prior=np.eye(n)*.12
    for channel in range(3):
        x=np.zeros(n)
        for _ in range(12):
            residual=a@x-b[:,channel];w=base*np.minimum(1.,.12/np.maximum(abs(residual),1e-8))
            aa=np.concatenate([a*np.sqrt(w[:,None]),prior]);bb=np.r_[b[:,channel]*np.sqrt(w),np.zeros(n)]
            x=np.linalg.lstsq(aa,bb,rcond=None)[0];x=np.clip(x,np.log(.65),np.log(1.55))
        g[:,channel]=x
    return np.exp(g),{'edges':len(rows),'gains_identifiable':True,'pair_constraints':records,
        'gain_bounds':[.65,1.55],'model':'per-frame linear-RGB multiplicative gains; robust IRLS; fixed camera poses'}


def photometric_error(observations,gains,mask):
    errors=[]
    valid=np.isfinite(observations).all(2)&(observations.min(2)>.015)&(observations.max(2)<.85)
    for i in range(len(gains)):
        for j in range(i+1,len(gains)):
            a=observations[i];b=observations[j]
            good=mask&valid[i]&valid[j]
            if good.sum()<12:continue
            delta=np.abs(np.log(a[good]*gains[i])-np.log(b[good]*gains[j]))
            errors.extend(np.median(delta,axis=1).tolist())
    return {'sample_pairs':len(errors),'median_absolute_log_rgb':float(np.median(errors)) if errors else None,
        'p90_absolute_log_rgb':float(np.percentile(errors,90)) if errors else None}


def fill_small_supported_holes(o3d,mesh,frames):
    # VTK-backed triangulation proposes small boundary closures. Every retained
    # original vertex/triangle is preserved, with float32 export precision.
    tensor=o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    filled=tensor.fill_holes(hole_size=.25).to_legacy()
    v0=np.asarray(mesh.vertices);t0=np.asarray(mesh.triangles);v=np.asarray(filled.vertices);tt=np.asarray(filled.triangles)
    if len(v)!=len(v0) or not np.array_equal(tt[:len(t0)],t0) or not np.allclose(v,v0,atol=2e-6):
        raise ValueError('Hole filler altered existing topology or moved original geometry')
    added=tt[len(t0):];stats={'proposed_faces':len(added),'maximum_boundary_hole_radius_m':.25}
    if not len(added):return mesh,np.zeros(len(t0),bool),stats
    proposal=o3d.geometry.TriangleMesh();proposal.vertices=filled.vertices;proposal.triangles=o3d.utility.Vector3iVector(added)
    labels,counts,areas=proposal.cluster_connected_triangles();labels=np.asarray(labels);areas=np.asarray(areas)
    keep_group=np.zeros(len(counts),bool);group_details=[]
    for group in range(len(counts)):
        ids=np.unique(added[labels==group]);points=v[ids];centred=points-points.mean(0)
        _,_,vt=np.linalg.svd(centred,full_matrices=False);planar=float(np.max(abs(centred@vt[-1])))
        span=float(np.linalg.norm(np.ptp(points,axis=0)));normal=abs(vt[-1,2])
        keep_group[group]=areas[group]<=.15 and planar<=.018 and span<=.6 and (normal<.3 or normal>.92)
        group_details.append({'group':group,'area_m2':float(areas[group]),'planar_residual_m':planar,
            'span_m':span,'geometric_gate':bool(keep_group[group])})
    centres=v[added].mean(1);free=np.zeros(len(added),int);support=np.zeros(len(added),int)
    for frame in frames.values():
        uv,z=project(centres,frame['k_depth'],frame['world_optical']);h,w=frame['depth'].shape
        x,y=np.rint(np.clip(uv,-1e5,1e5)).astype(int).T
        ids=np.flatnonzero((z>.25)&(z<4.2)&(x>=0)&(x<w)&(y>=0)&(y<h))
        d=frame['depth'][y[ids],x[ids]];tol=.08+.02*z[ids]
        support[ids[(d>0)&(abs(d-z[ids])<=tol)]]+=1
        free[ids[(d>0)&(z[ids]<d-tol)]]+=1
    # Unsupported but planar micro-holes may be displayed as INFERRED geometry;
    # disagreement in two views vetoes even these. Never a navigation surface.
    inferred_micro=areas[labels]<.04
    keep=keep_group[labels]&(free<2)&((support>=3)|inferred_micro)
    accepted=added[keep];out=o3d.geometry.TriangleMesh(mesh)
    out.triangles=o3d.utility.Vector3iVector(np.concatenate([t0,accepted]));out.compute_vertex_normals()
    flag=np.r_[np.zeros(len(t0),bool),np.ones(len(accepted),bool)]
    stats.update(accepted_faces=len(accepted),rejected_faces=int((~keep).sum()),
        supported_faces=int(np.sum(keep&(support>=3))),inferred_micro_faces=int(np.sum(keep&(support<3))),
        all_added_faces_inferred=True,accepted_area_m2=float(np.sum(np.linalg.norm(np.cross(v[accepted[:,1]]-v[accepted[:,0]],v[accepted[:,2]]-v[accepted[:,0]]),axis=1))*.5),
        groups=group_details,policy='Display only; planar <=0.15m2, radius<=0.25m; unknown micro <=0.04m2 flagged, free-space veto >=2 views')
    return out,flag,stats


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());source=ROOT/'outputs/student_union_preserved_display_v1'
    source_manifest=json.loads((source/'manifest.json').read_text())
    previous=json.loads((a.output/'manifest.json').read_text()) if a.resume else {}
    results=previous.get('sessions',{});hashes=previous.get('inputs_sha256',{})
    for path in [pointer_path,mp,Path(__file__),ROOT/'src/surface_model_utils.py']:
        h=sha256_file(path)
        if str(path) in hashes and hashes[str(path)]!=h:raise ValueError('Changed source during resume')
        hashes[str(path)]=h
    for name in a.sessions or selected['sessions']:
        if name in results:continue
        print(name+': hole and exposure audit',flush=True);dest=a.output/name;dest.mkdir();record=selected['sessions'][name]
        paths=[source/name/'preserved_geometry.ply',mp.parent/record['trajectory'],Path(selected['database_integrity'][name]['database'])]
        for path in paths:hashes[str(path)]=sha256_file(path)
        poses=load_pose_csv(paths[1]);con=sqlite3.connect(f'file:{paths[2]}?mode=ro',uri=True)
        ids=source_manifest['sessions'][name]['texture']['camera_frames']
        frames={n:decode_frame(con,n,record['preprocessing'],poses[n]) for n in ids};con.close()
        mesh=o3d.io.read_triangle_mesh(str(paths[0]));mesh.compute_vertex_normals();before=topology(mesh)
        mesh,inferred,hole_stats=fill_small_supported_holes(o3d,mesh,frames)
        np.save(dest/'inferred_faces.npy',inferred)
        vertices=np.asarray(mesh.vertices);normals=np.asarray(mesh.vertex_normals)
        rng=np.random.default_rng(20261018);sample_ids=np.sort(rng.choice(len(vertices),min(200000,len(vertices)),replace=False))
        observations=np.full((len(ids),len(sample_ids),3),np.nan,np.float32)
        for i,node in enumerate(ids):
            ii,c,w=visible_samples(vertices[sample_ids],normals[sample_ids],frames[node]);observations[i,ii]=srgb_to_linear(c)
        train=np.arange(len(sample_ids))%5!=0;gains,gain_report=solve_log_gains(observations,train)
        gain_report['sampled_world_vertices']=len(sample_ids)
        gain_report['visible_world_samples_per_frame']=np.isfinite(observations).all(2).sum(1).tolist()
        gain_report['training_before']=photometric_error(observations,np.ones_like(gains),train)
        gain_report['training_after']=photometric_error(observations,gains,train)
        gain_report['reserved_before']=photometric_error(observations,np.ones_like(gains),~train)
        gain_report['reserved_after']=photometric_error(observations,gains,~train)
        gain_report['reserved_scope']='Reserved world-point samples in the same capture; not independent lighting/sensor validation'
        print(name+': continuous vertex colour baking',flush=True)
        sums=np.zeros((len(vertices),3));weights=np.zeros(len(vertices));best=np.zeros(len(vertices));best_colours=np.zeros((len(vertices),3))
        for i,node in enumerate(ids):
            ii,c,w=visible_samples(vertices,normals,frames[node]);linear=srgb_to_linear(c)*gains[i]
            sums[ii]+=linear*w[:,None];weights[ii]+=w
            better=w>best[ii];best[ii[better]]=w[better];best_colours[ii[better]]=linear[better]
        covered=weights>0;linear=srgb_to_linear(np.asarray(mesh.vertex_colors))
        # Equal blend and the strongest view: softer exposure seams without
        # claiming that 5cm vertex colour can preserve small text like a photo.
        linear[covered]=.65*sums[covered]/weights[covered,None]+.35*best_colours[covered]
        mesh.vertex_colors=o3d.utility.Vector3dVector(linear_to_srgb(linear))
        o3d.io.write_triangle_mesh(str(dest/'appearance_fixed.ply'),mesh);export_rgb(mesh,dest/'appearance_fixed.glb')
        marked=o3d.geometry.TriangleMesh(mesh);colours=np.asarray(marked.vertex_colors).copy()
        if inferred.any():colours[np.unique(np.asarray(marked.triangles)[inferred])]=[1.,.15,.65]
        marked.vertex_colors=o3d.utility.Vector3dVector(colours);export_rgb(marked,dest/'inferred_marked.glb')
        gain_rows={str(node):gains[i].tolist() for i,node in enumerate(ids)}
        (dest/'exposure_gains.json').write_text(json.dumps(gain_rows,indent=2)+'\n')
        results[name]={'session':name,'hole_repair':hole_stats,'exposure':gain_report,'gains':gain_rows,
            'original_vertex_positions_unchanged':True,'new_faces_are_inferred':True,'navigation_ready':False,
            'before_topology':before,'after_topology':topology(mesh),'vertex_colour_coverage':float(covered.mean()),
            'output_colour_model':'Fixed-camera globally gain-corrected continuous vertex RGB; not high-resolution text texture',
            'display_glb':'appearance_fixed.glb','marked_glb':'inferred_marked.glb',
            'outputs':{p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in dest.iterdir() if p.is_file()}}
        (dest/'summary.json').write_text(json.dumps(results[name],indent=2)+'\n')
        for path,h in hashes.items():
            if sha256_file(Path(path))!=h:raise ValueError('Original input changed')
        (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':results,
            'inputs_sha256':hashes,'selected_manifest_sha256':pointer['manifest_sha256'],
            'source_map_replaced':False,'all_original_inputs_unchanged':True,'navigation_ready':False,
            'limitations':['Only small bounded planar holes are proposed; large/unobserved openings remain.',
                'Every added face is marked inferred, not navigation geometry.',
                'Gain compensation cannot undo moving shadows, reflections or saturation.',
                'Continuous vertex colour trades text sharpness for fewer hard texture seams; roadview keeps native photos.']},indent=2)+'\n')
        print(json.dumps({'complete':name,'holes':{k:v for k,v in hole_stats.items() if k!='groups'},
            'reserved_photo_before':gain_report['reserved_before'],'reserved_photo_after':gain_report['reserved_after']}),flush=True)
        del mesh,marked,frames,observations;gc.collect()


if __name__=='__main__':main()
