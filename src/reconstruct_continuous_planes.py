#!/usr/bin/env python3
"""Continuous finite planes, source-mesh replacement, and original-photo charts.

Display derivative only. No poses or source DBs are written. Unknown openings
are NOT closed just because a polygon has a hole. Positive depth support is
required for large gap completion; repeated free-space evidence vetoes faces.
"""
import argparse
import gc
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from PIL import Image

from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file, utc_now
from surface_model_utils import decode_frame, project, write_glb
from rebuild_structural_display import world_grid, free_space_votes
from repair_visual_appearance import linear_to_srgb
from refine_surface_display import srgb_to_linear

ROOT=Path(__file__).resolve().parents[1]
CELL=.06


def patch_corners(patch):
    h,w=patch['shape'];uv=np.array([[0,0],[w,0],[0,h],[w,h]])*CELL+patch['low']
    return np.asarray(patch['centre'])+uv@np.asarray(patch['basis'])[:,:2].T


def compatible(a,b,angle_deg=5.,offset=.12,gap=.60):
    na=np.asarray(a['basis'])[:,2];nb=np.asarray(b['basis'])[:,2]
    if abs(na@nb)<np.cos(np.deg2rad(angle_deg)):return False
    ca=np.asarray(a['centre']);cb=np.asarray(b['centre'])
    if max(abs((ca-cb)@na),abs((ca-cb)@nb))>offset:return False
    basis=np.asarray(a['basis'])[:,:2]
    qa=(patch_corners(a)-ca)@basis;qb=(patch_corners(b)-ca)@basis
    separation=np.maximum(np.maximum(qa.min(0)-qb.max(0),qb.min(0)-qa.max(0)),0)
    return np.linalg.norm(separation)<=gap


def group_patches(patches):
    """Spatially connected, complete-link plane compatibility prevents chain drift."""
    groups=[]
    for i in sorted(range(len(patches)),key=lambda i:-patches[i]['cells']):
        for group in groups:
            if all(compatible(patches[i],patches[j],gap=1e6) for j in group) and any(compatible(patches[i],patches[j]) for j in group):
                group.append(i);break
        else:groups.append([i])
    return groups


def continuous_mask(observed,support,free,cell=CELL):
    """Fill only supported gaps; preserve unknown holes and observed free space."""
    pad=18;closed=ndimage.binary_closing(np.pad(observed,pad),iterations=16)[pad:-pad,pad:-pad]
    envelope=ndimage.binary_fill_holes(closed|observed)
    veto=(free>=2)&(free>=1.5*(support+1))
    veto=ndimage.binary_dilation(veto,iterations=1)
    # Distinct selected camera positions provide support, not missing depth.
    added=envelope&~observed&(support>=2)&~veto
    mask=(observed|added)&~veto
    labels,_=ndimage.label(mask);sizes=np.bincount(labels.ravel());keep=sizes*cell*cell>=.15;keep[0]=False
    mask&=keep[labels]
    return mask,{'new_supported_cells':int(np.sum(mask&~observed)),
        'vetoed_observed_cells':int(np.sum(observed&veto)),
        'unknown_gap_cells_preserved':int(np.sum(envelope&~observed&~added&~veto)),
        'free_space_cells':int(veto.sum())}


def make_plane(patches,indices,points,normals):
    anchor=patches[indices[0]];basis=np.asarray(anchor['basis']).copy();centre=np.asarray(anchor['centre'])
    bounds=np.concatenate([patch_corners(patches[i]) for i in indices])
    q=(points-centre)@basis;uv_bounds=(bounds-centre)@basis[:,:2]
    low=np.floor(uv_bounds.min(0)/CELL)*CELL;high=np.ceil(uv_bounds.max(0)/CELL)*CELL
    inside=(abs(q[:,2])<.20)&(abs(normals@basis[:,2])>.85)&np.all((q[:,:2]>=low)&(q[:,:2]<=high),axis=1)
    # Union of local footprints prevents merging across a different corridor.
    footprint=np.zeros(len(points),bool)
    for i in indices:
        p=patches[i];local=(points-np.asarray(p['centre']))@np.asarray(p['basis'])
        h,w=p['shape'];lo=np.asarray(p['low'])
        footprint|=np.all((local[:,:2]>=lo-.12)&(local[:,:2]<=lo+[w*CELL,h*CELL]+.12),axis=1)
    ids=np.flatnonzero(inside&footprint)
    if len(ids)<150:return None
    # Robust offset only. Keep the verified local orientation rather than
    # rotating the plane to fit an entire building or swallowing a nearby wall.
    delta=float(np.median(q[ids,2]));centre=centre+basis[:,2]*delta
    shape=(np.ceil((high-low)/CELL).astype(int)+2)[::-1]
    if np.prod(shape)>120000:return None
    ij=np.floor(((points[ids]-centre)@basis[:,:2]-low)/CELL).astype(int)
    observed=np.zeros(shape,bool);observed[ij[:,1],ij[:,0]]=True
    # Raster splats account for sampling spacing without extrapolating a wall.
    observed=ndimage.binary_closing(np.pad(observed,2),iterations=1)[2:-2,2:-2]|observed
    return {'centre':centre,'basis':basis,'low':low,'observed':observed,'members':indices,
        'fit_p90_m':float(np.percentile(abs((points[ids]-centre)@basis[:,2]),90))}


def sample_view(frame,points,normal,tolerance=.18):
    uv,z=project(points,frame['k_depth'],frame['world_optical']);h,w=frame['depth'].shape
    x,y=np.rint(np.clip(uv,-1e6,1e6)).astype(int).T
    visible=(z>.35)&(z<4.3)&(x>=1)&(x<w-1)&(y>=1)&(y<h-1)
    d=np.zeros(len(z));ii=np.flatnonzero(visible);d[ii]=frame['depth'][y[ii],x[ii]]
    direction=frame['world_optical'][:3,3]-points
    cosine=abs(direction@normal)/np.maximum(np.linalg.norm(direction,axis=1),1e-8)
    valid=visible&(d>0)&(abs(d-z)<tolerance+.02*z)&(cosine>.45)
    weight=np.where(valid,cosine**4/np.maximum(z,.5)**2,0.)
    return valid,weight


def select_views(plane,frames,maximum=32):
    h,w=plane['observed'].shape;grid=world_grid(plane['centre'],plane['basis'],plane['low'],(h,w),centres=True)
    ids=np.flatnonzero(ndimage.binary_fill_holes(plane['observed']).ravel())
    ids=ids[np.linspace(0,len(ids)-1,min(500,len(ids))).astype(int)];points=grid[ids]
    proposals=[];normal=plane['basis'][:,2]
    for node,f in frames.items():
        # Coarse culling followed by measured depth validation at actual cells.
        if abs((f['world_optical'][:3,3]-plane['centre'])@normal)>4.5:continue
        valid,weight=sample_view(f,points,normal)
        if valid.sum()>=4:proposals.append((node,valid,weight))
    covered=np.zeros(len(points),int);chosen=[];positions=[]
    for _ in range(maximum):
        best=None;best_score=0
        for node,valid,weight in proposals:
            if node in chosen:continue
            pos=frames[node]['world_optical'][:3,3]
            if positions and min(np.linalg.norm(pos-p) for p in positions)<.20:continue
            score=np.sum(np.minimum(weight,4.)/(1+covered*3))
            if score>best_score:best=(node,valid);best_score=score
        if best is None:break
        node,valid=best;chosen.append(node);positions.append(frames[node]['world_optical'][:3,3]);covered+=valid
    return chosen,{'candidate_frames_with_depth_support':len(proposals),'selected_photo_nodes':chosen,
        'sampled_coverage_fraction':float(np.mean(covered>0)),'all_frame_candidates':len(frames)}


def solve_log_gains(images,weights):
    """Relative robust overlap gains, anchored to the most-covered photograph."""
    n=len(images)
    if not n:return np.empty((0,3))
    rows=[];values=[]
    for i in range(n):
        for j in range(i):
            valid=(weights[i]>0)&(weights[j]>0)&np.all((images[i]>.02)&(images[i]<.95)&(images[j]>.02)&(images[j]<.95),axis=2)
            if valid.sum()<30:continue
            sample=np.log(np.maximum(images[j][valid],.01))-np.log(np.maximum(images[i][valid],.01))
            row=np.zeros(n);row[i]=1;row[j]=-1;rows.append(row);values.append(np.median(sample,axis=0))
    rows.extend(np.eye(n)*.05);values.extend(np.zeros((n,3)))
    anchor=int(np.argmax([np.count_nonzero(w) for w in weights]));row=np.zeros(n);row[anchor]=10;rows.append(row);values.append(np.zeros(3))
    log=np.linalg.lstsq(np.asarray(rows),np.asarray(values),rcond=None)[0]
    return np.exp(np.clip(log,np.log(.5),np.log(2.)))


def photo_chart(plane,mask,frames,nodes,reference,dest,index):
    """1cm photo atlas; overlap gains + low-frequency feathering, dominant detail."""
    h,w=mask.shape;tw=min(2048,max(2,int(w*CELL/.01)));th=min(2048,max(2,int(h*CELL/.01)))
    yy,xx=np.indices((th,tw));uv=np.c_[(xx.ravel()+.5)*w*CELL/tw,(yy.ravel()+.5)*h*CELL/th]+plane['low']
    points=plane['centre']+uv@plane['basis'][:,:2].T
    # Warp only the selected photo candidates. All poses were considered above.
    images=[];weights=[];coverage=[]
    for node in nodes:
        f=frames[node];valid,score=sample_view(f,points,plane['basis'][:,2])
        pixels,_=project(points,f['k_rgb'],f['world_optical']);maps=pixels.reshape(th,tw,2).astype(np.float32)
        rgb=cv2.remap(f['rgb'],maps[:,:,0],maps[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)/255.
        images.append(srgb_to_linear(rgb).astype(np.float32));weights.append(score.reshape(th,tw).astype(np.float32));coverage.append(valid.reshape(th,tw))
    # Solve exposure on a reduced grid (actual overlaps, not point-cloud RGB).
    step=max(1,int(max(tw,th)/256));gains=solve_log_gains([v[::step,::step] for v in images],[v[::step,::step] for v in weights])
    tree=cKDTree(np.asarray(reference.points));_,near=tree.query(points,workers=2)
    fallback=srgb_to_linear(np.asarray(reference.colors)[near]).reshape(th,tw,3).astype(np.float32)
    if images:
        stack=np.asarray(weights);winner=np.argmax(stack,axis=0);covered=stack.max(0)>0
        total=np.zeros((th,tw),np.float32);low=np.zeros((th,tw,3),np.float32);detail=np.zeros_like(low)
        overlap_before=[];overlap_after=[]
        anchor=int(np.argmax([np.count_nonzero(v) for v in weights]))
        for i,(im,weight,gain) in enumerate(zip(images,weights,gains)):
            fixed=im*gain
            # Normalize the low frequencies across source transitions; retain
            # high-frequency detail from ONE image to avoid ghosted lettering.
            smooth=cv2.GaussianBlur(fixed,(0,0),4)
            feather=cv2.GaussianBlur(weight,(0,0),2)*(weight>0)
            low+=smooth*feather[:,:,None];total+=feather
            chosen=(winner==i)&covered;detail[chosen]=(fixed-smooth)[chosen]
            if i!=anchor:
                overlap=(weights[anchor]>0)&(weight>0)
                if overlap.any():
                    overlap_before.append(float(np.median(abs(images[anchor][overlap]-im[overlap]))))
                    overlap_after.append(float(np.median(abs(images[anchor][overlap]*gains[anchor]-fixed[overlap]))))
        result=fallback.copy();result[covered]=(low/np.maximum(total[:,:,None],1e-8)+detail)[covered]
    else:
        result=fallback;covered=np.zeros((th,tw),bool);overlap_before=[];overlap_after=[]
    mask_tex=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0
    path=dest/f'continuous_{index:03d}.png';Image.fromarray(np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5)).save(path)
    # Unknown/fallback cells are saved for honest coverage visualization.
    coverage_path=dest/f'coverage_{index:03d}.png';Image.fromarray(np.uint8(np.where(mask_tex,np.where(covered,255,80),0))).save(coverage_path)
    return path,{'photo_coverage_on_surface_fraction':float(np.mean(covered[mask_tex])) if mask_tex.any() else 0.,
        'texture_shape':[tw,th],'gains':{str(n):g.tolist() for n,g in zip(nodes,gains)},
        'overlap_linear_rgb_before_median':float(np.median(overlap_before)) if overlap_before else None,
        'overlap_linear_rgb_after_median':float(np.median(overlap_after)) if overlap_after else None,
        'fallback':'selected_cloud_RGB_only_at_uncovered_texels','coverage_image':coverage_path.name}


def build_mesh(o3d,plane,mask):
    h,w=mask.shape;vertices=world_grid(plane['centre'],plane['basis'],plane['low'],(h+1,w+1))
    ids=np.arange((h+1)*(w+1)).reshape(h+1,w+1);a=ids[:-1,:-1][mask];b=a+1;c=a+w+1;d=c+1
    t=np.concatenate([np.c_[a,b,c],np.c_[b,d,c]])
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(vertices);mesh.triangles=o3d.utility.Vector3iVector(t);mesh.compute_vertex_normals()
    q=(vertices-plane['centre'])@plane['basis'][:,:2]-plane['low'];uv=q/[w*CELL,h*CELL]
    return mesh,uv[t]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source',type=Path,default=ROOT/'outputs/student_union_structural_strong_floors_v1')
    p.add_argument('--sessions',nargs='+',default=['floor2','lobby']);p.add_argument('--resume',action='store_true');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    if sha256_file(mp)!=ptr['manifest_sha256']:raise ValueError('Map changed')
    source=json.loads((a.source/'manifest.json').read_text());prior=a.output/'manifest.json'
    run=json.loads(prior.read_text()) if prior.exists() else {'sessions':{},'inputs_sha256':{}}
    def track(path):
        path=Path(path).resolve();h=sha256_file(path)
        if str(path) in run['inputs_sha256'] and run['inputs_sha256'][str(path)]!=h:raise ValueError('Input changed '+str(path))
        run['inputs_sha256'][str(path)]=h
    for path in [Path(__file__),mp,a.source/'manifest.json',ROOT/'src/surface_model_utils.py',ROOT/'src/append_plane_glb.py',ROOT/'src/coherent_photo_charts.py',ROOT/'src/rebuild_structural_display.py',ROOT/'src/clean_structural_fragments.py']:track(path)
    for name in a.sessions:
        if name in run['sessions']:continue
        dest=a.output/name;dest.mkdir(exist_ok=False);s=selected['sessions'][name]
        db=Path(selected['database_integrity'][name]['database']);track(db);track(mp.parent/s['cloud']);track(mp.parent/s['trajectory'])
        poses=load_pose_csv(mp.parent/s['trajectory']);con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        frames={}
        print(name+': considering all '+str(len(poses))+' original RGB-D frames',flush=True)
        for i,(node,pose) in enumerate(poses.items()):
            frames[node]=decode_frame(con,node,s['preprocessing'],pose)
            if i%150==0:print(name+': decoded '+str(i+1),flush=True)
        con.close()
        reference=o3d.io.read_point_cloud(str(mp.parent/s['cloud']));pc=reference.voxel_down_sample(.05)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.25,max_nn=40));points=np.asarray(pc.points);normals=np.asarray(pc.normals)
        # Only walls are rebuilt here. Existing floor/ceiling heightfields and
        # stair treads/risers stay byte-coordinate identical outside wall masks.
        patches=[v for v in source['sessions'][name]['patches'] if abs(np.asarray(v['basis'])[2,2])<.12]
        groups=group_patches(patches);print(name+': '+str(len(patches))+' wall patches -> '+str(len(groups))+' groups',flush=True)
        base=a.source/name/'clean_structural.ply';track(base)
        old=o3d.io.read_triangle_mesh(str(base));old.compute_triangle_normals();v=np.asarray(old.vertices);t=np.asarray(old.triangles)
        centre_faces=v[t].mean(1);oldnorm=np.asarray(old.triangle_normals);remove=np.zeros(len(t),bool)
        meshes=[];textures=[];uvs=[];reports=[];already=[]
        for index,group in enumerate(groups):
            plane=make_plane(patches,group,points,normals)
            if plane is None:continue
            nodes,view_report=select_views(plane,frames)
            if len(nodes)<2:continue
            grid=world_grid(plane['centre'],plane['basis'],plane['low'],plane['observed'].shape,centres=True)
            support,free=free_space_votes(grid,{n:frames[n] for n in nodes},tolerance=.18)
            mask,mask_report=continuous_mask(plane['observed'],support.reshape(plane['observed'].shape),free.reshape(plane['observed'].shape))
            # Remove overlap from later co-planar charts, even if patches were
            # not merged spatially. Never fuse opposite/distant parallel walls.
            for older in already:
                if abs(plane['basis'][:,2]@older['basis'][:,2])<np.cos(np.deg2rad(5)):continue
                q=(grid-older['centre'])@older['basis'];ij=np.floor((q[:,:2]-older['low'])/CELL).astype(int);x,y=ij.T;oh,ow=older['mask'].shape
                ii=np.flatnonzero((abs(q[:,2])<.15)&(x>=0)&(x<ow)&(y>=0)&(y<oh))
                mask.ravel()[ii[older['mask'][y[ii],x[ii]]]]=False
            if mask.sum()*CELL*CELL<1.5:continue
            mesh,uv=build_mesh(o3d,plane,mask)
            from coherent_photo_charts import texture_plane
            texture,photo_report=texture_plane(plane,mask,frames,nodes,reference,dest,index)
            q=(centre_faces-plane['centre'])@plane['basis'];ij=np.floor((q[:,:2]-plane['low'])/CELL).astype(int);x,y=ij.T;h,w=mask.shape
            # Remove only wall-oriented faces; this excludes stair treads and
            # floor/ceiling surfaces. Protruding normal-incompatible details stay.
            ii=np.flatnonzero((abs(q[:,2])<.32)&(abs(oldnorm@plane['basis'][:,2])>.65)&(x>=0)&(x<w)&(y>=0)&(y<h))
            remove[ii[mask[y[ii],x[ii]]]]=True
            _,near=cKDTree(np.asarray(reference.points)).query(np.asarray(mesh.vertices),workers=2)
            mesh.vertex_colors=o3d.utility.Vector3dVector(np.asarray(reference.colors)[near]);meshes.append(mesh);uvs.append(uv);textures.append(texture)
            already.append({**plane,'mask':mask})
            report={'centre':plane['centre'].tolist(),'basis':plane['basis'].tolist(),'low':plane['low'].tolist(),'shape':list(mask.shape),
                'cells':int(mask.sum()),'source_patch_members':group,'fit_p90_m':plane['fit_p90_m'],**view_report,**mask_report,**photo_report}
            reports.append(report);print(json.dumps({'session':name,'group':index,'members':len(group),'area_m2':round(mask.sum()*CELL*CELL,2),'photo_coverage':round(photo_report['photo_coverage_on_surface_fraction'],3)}),flush=True)
            np.savez_compressed(dest/f'mask_{index:03d}.npz',mask=mask,observed=plane['observed'],support=support.reshape(mask.shape),free=free.reshape(mask.shape),centre=plane['centre'],basis=plane['basis'],low=plane['low'])
        horizontal_faces_removed=int(np.sum(remove&(abs(oldnorm[:,2])>.985)))
        np.save(dest/'removed_source_faces.npy',remove)
        old.remove_triangles_by_mask(remove);old.remove_unreferenced_vertices();old.compute_vertex_normals()
        # Preserve the original photo-textured GLB of residual geometry exactly
        # (index filtering), then append continuous charts without recoloring it.
        from clean_structural_fragments import filter_glb
        from append_plane_glb import append_planes
        residual=dest/'residual.glb';source_glb=a.source/name/'clean_textured.glb';track(source_glb)
        filter_glb(source_glb,residual,~remove)
        combined=o3d.geometry.TriangleMesh(old)
        for mesh in meshes:combined+=mesh
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'structural.ply'),combined)
        append_planes(residual,dest/'structural_textured.glb',meshes,uvs,textures)
        # For audit ordering, GLB must match residual + chart triangle order.
        run['sessions'][name]={**source['sessions'][name],'patches':reports,'continuous_planes':reports,
            'original_wall_patch_count':len(patches),'merged_group_count':len(groups),'rebuilt_groups':len(reports),
            'replaced_original_faces':int(remove.sum()),'retained_previous_faces':len(old.triangles),
            'horizontal_faces_removed':horizontal_faces_removed,
            'source_database_frames_considered':len(frames),'pose_changes':0,'navigation_ready':False,
            'source_mesh':str(base.resolve()),'removed_source_faces_file':'removed_source_faces.npy',
            'texture_method':'Original photographs; robust overlap gains; coherent Potts ICM labels; bounded luminance leveling'}
        (dest/'summary.json').write_text(json.dumps(run['sessions'][name],indent=2)+'\n')
        run.update(created_utc=utc_now(),selected_manifest_sha256=ptr['manifest_sha256'],selected_map_changed=False,navigation_ready=False,
            scope='Continuous wall display planes, supported hole completion, original-photo charts; existing stairs/floors retained.')
        prior.write_text(json.dumps(run,indent=2)+'\n');del frames,pc,reference,old,combined;gc.collect()
    # Unmodified sessions are available for one five-session native comparison.
    for name in selected['sessions']:
        if name in run['sessions']:continue
        dest=a.output/name;dest.mkdir(exist_ok=True)
        for filename in ['structural.ply','structural_textured.glb']:
            input_path=a.source/name/('clean_structural.ply' if filename.endswith('ply') else 'clean_textured.glb')
            track(input_path);shutil.copyfile(input_path,dest/filename)
        run['sessions'][name]={**source['sessions'][name],'continuous_planes':[],'unchanged_from_previous':True}
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input modified '+path)
    prior.write_text(json.dumps(run,indent=2)+'\n')


if __name__=='__main__':main()
