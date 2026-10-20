#!/usr/bin/env python3
"""Finite structural patches replace torn display surfaces; never edits SLAM poses.

Robust planar detection + finite occupancy morphology + free-space veto + smooth
world-space colour charts. Doors/large gaps stay open; other geometry stays local.
All replacement patches are modeled display geometry, not surveyed navigation.
"""
import argparse
import gc
import json
from pathlib import Path
import sqlite3
import shutil

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from repair_surface_fragmentation import export_rgb,topology
from surface_model_utils import decode_frame,project,write_glb
from structural_prior_profiles import strong_wall_mask

ROOT=Path(__file__).resolve().parents[1]
CELL=.06


def bounded_mask(occupied,radius=3,maximum_hole_area=.24,cell=CELL):
    """Small local slits only; large enclosed holes and open boundaries survive."""
    padded=np.pad(occupied,radius+1)
    closed=ndimage.binary_closing(padded,iterations=radius)[radius+1:-radius-1,radius+1:-radius-1]
    proposals=(closed|ndimage.binary_fill_holes(occupied))&~occupied
    labels,count=ndimage.label(proposals)
    sizes=np.bincount(labels.ravel())*cell*cell;keep=sizes<=maximum_hole_area;keep[0]=False
    return occupied|keep[labels]


def world_grid(centre,basis,low,shape,cell=CELL,centres=False):
    y,x=np.indices(shape);uv=np.c_[x.ravel(),y.ravel()]*cell+low
    if centres:uv+=cell/2
    return centre+uv@basis[:,:2].T


def free_space_votes(points,frames,tolerance=.10):
    support=np.zeros(len(points),int);free=np.zeros(len(points),int)
    for f in frames.values():
        uv,z=project(points,f['k_depth'],f['world_optical']);h,w=f['depth'].shape
        x,y=np.rint(np.clip(uv,-1e5,1e5)).astype(int).T
        ii=np.flatnonzero((z>.25)&(z<4.3)&(x>=0)&(x<w)&(y>=0)&(y<h))
        d=f['depth'][y[ii],x[ii]];tol=tolerance+.02*z[ii]
        support[ii[(d>0)&(abs(d-z[ii])<=tol)]]+=1
        free[ii[(d>0)&(z[ii]<d-tol)]]+=1
    return support,free


def discover(o3d,cloud):
    pc=cloud.voxel_down_sample(.075)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.25,max_nn=40))
    patches=pc.detect_planar_patches(normal_variance_threshold_deg=35,coplanarity_deg=80,
        outlier_ratio=.5,min_plane_edge_length=.8,min_num_points=40,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=40))
    p=np.asarray(pc.points);normals=np.asarray(pc.normals);result=[]
    for box in sorted(patches,key=lambda b:-b.extent[0]*b.extent[1]):
        basis=np.asarray(box.R).copy();normal=basis[:,2]
        if not(abs(normal[2])<.12 or abs(normal[2])>.985):continue
        q=(p-box.center)@basis
        inside=(abs(q[:,0])<box.extent[0]/2+.12)&(abs(q[:,1])<box.extent[1]/2+.12)&(abs(q[:,2])<.15)
        inside&=abs(normals@normal)>.85
        ids=np.flatnonzero(inside)
        if len(ids)<100:continue
        points=p[ids];centre=np.median(points,axis=0)
        for _ in range(3):
            _,_,vt=np.linalg.svd(points-centre,full_matrices=False);n=vt[-1]
            if n@normal<0:n=-n
            residual=abs((points-centre)@n);keep=residual<=max(.025,np.quantile(residual,.8))
            points=points[keep];centre=points.mean(0)
        if len(points)<70 or not(abs(n[2])<.12 or abs(n[2])>.985):continue
        # Keep vertical/horizontal structure local: no global Manhattan snapping.
        u=basis[:,0]-n*(basis[:,0]@n);u/=np.linalg.norm(u);v=np.cross(n,u);basis=np.c_[u,v,n]
        uv=(p[ids]-centre)@basis[:,:2]
        low=np.floor(uv.min(0)/CELL)*CELL;shape=(np.ceil((uv.max(0)-low)/CELL).astype(int)+1)[::-1]
        if np.prod(shape)>150000:continue
        ij=np.floor((uv-low)/CELL).astype(int);occupied=np.zeros(shape,bool);occupied[ij[:,1],ij[:,0]]=True
        if occupied.sum()*CELL*CELL<.8:continue
        # Avoid another modeled layer on an already covered parallel plane.
        duplicate=False
        for old in result:
            if abs(n@old['basis'][:,2])>.996 and abs((centre-old['centre'])@old['basis'][:,2])<.15:
                c=(centre-old['centre'])@old['basis'][:,:2];index=np.floor((c-old['low'])/CELL).astype(int)
                if 0<=index[1]<old['mask'].shape[0] and 0<=index[0]<old['mask'].shape[1] and old['mask'][index[1],index[0]]:
                    duplicate=True;break
        if duplicate:continue
        result.append({'centre':centre,'basis':basis,'low':low,'mask':bounded_mask(occupied),
            'observed_mask':occupied,'support_points':len(ids),'fit_p90_m':float(np.percentile(abs((p[ids]-centre)@n),90))})
    return result,len(patches),len(pc.points)


def patch_mesh(o3d,patch,frames,reference,gains,dest,index,strong=False):
    mask=patch['mask'];h,w=mask.shape
    strong_applied=bool(strong and patch['support_points']>=250 and patch['observed_mask'].sum()*CELL*CELL>=2.5 and patch['fit_p90_m']<=.15)
    protected=np.zeros_like(mask)
    if strong_applied:mask,protected=strong_wall_mask(patch['observed_mask'],CELL)
    centres=world_grid(patch['centre'],patch['basis'],patch['low'],(h,w),centres=True)
    support,free=free_space_votes(centres,frames)
    # Never close a repeatedly observed opening, including a doorway.
    rejected=(free.reshape(h,w)>=3)&(free.reshape(h,w)>=2*(support.reshape(h,w)+1))
    if strong_applied:rejected=ndimage.binary_dilation(rejected,iterations=1)
    rejected_count=int(np.sum(mask&rejected));mask=mask&~rejected
    labels,count=ndimage.label(mask);sizes=np.bincount(labels.ravel());good=sizes>=80;good[0]=False;mask&=good[labels]
    if not mask.any():return None
    patch['mask']=mask
    vertices=world_grid(patch['centre'],patch['basis'],patch['low'],(h+1,w+1))
    ids=np.arange((h+1)*(w+1)).reshape(h+1,w+1);a=ids[:-1,:-1][mask];b=a+1;c=a+w+1;d=c+1
    triangles=np.concatenate([np.c_[a,b,c],np.c_[b,d,c]])
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(vertices);mesh.triangles=o3d.utility.Vector3iVector(triangles)
    # One coherent texture chart per finite plane, not a different image per face.
    texel=.02;tw=max(2,int(np.ceil(w*CELL/texel)));th=max(2,int(np.ceil(h*CELL/texel)))
    texel=max(w*CELL/tw,h*CELL/th)
    yy,xx=np.indices((th,tw));uv=np.c_[(xx.ravel()+.5)*w*CELL/tw,(yy.ravel()+.5)*h*CELL/th]+patch['low']
    points=patch['centre']+uv@patch['basis'][:,:2].T
    tree=cKDTree(np.asarray(reference.points));distance,near=tree.query(points,workers=2)
    fallback=srgb_to_linear(np.asarray(reference.colors)[near]);sums=np.zeros_like(fallback);weight=np.zeros(len(points))
    n=patch['basis'][:,2]
    for node,f in frames.items():
        pixels,z=project(points,f['k_depth'],f['world_optical']);dh,dw=f['depth'].shape;x,y=np.rint(np.clip(pixels,-1e5,1e5)).astype(int).T
        ii=np.flatnonzero((z>.25)&(z<4.3)&(x>=0)&(x<dw)&(y>=0)&(y<dh))
        depth=f['depth'][y[ii],x[ii]];direction=f['world_optical'][:3,3]-points[ii]
        cosine=abs(direction@n)/np.maximum(np.linalg.norm(direction,axis=1),1e-9)
        valid=(depth>0)&(abs(depth-z[ii])<.12+.02*z[ii])&(cosine>.4)
        ii=ii[valid];cosine=cosine[valid]
        pixels,_=project(points[ii],f['k_rgb'],f['world_optical']);rh,rw=f['rgb'].shape[:2];x,y=np.rint(np.clip(pixels,-1e5,1e5)).astype(int).T
        good=(x>=0)&(x<rw)&(y>=0)&(y<rh);ii=ii[good];cosine=cosine[good]
        weights=cosine**4/np.maximum(z[ii],.5)**2
        colour=srgb_to_linear(f['rgb'][y[good],x[good]]/255.)*gains.get(str(node),np.ones(3))
        sums[ii]+=colour*weights[:,None];weight[ii]+=weights
    covered=weight>0
    # Sparse photographs must NOT be stretched over an entire structural plane.
    # Low coverage uses the full selected cloud's measured colour field instead.
    colour_source='selected_cloud_fused_colour_chart'
    if covered.mean()>=.6:
        fallback[covered]=sums[covered]/weight[covered,None];colour_source='multi_view_photos_with_fused_colour_fallback'
    image=fallback.reshape(th,tw,3)
    # Bounded low-frequency luminance normalization, not RGB hallucination.
    # This display option trades some real illumination variation for continuity.
    luminance=np.maximum(image@np.array([.2126,.7152,.0722]),.005)
    low=ndimage.gaussian_filter(luminance,sigma=.25/texel)
    correction=np.clip(np.median(low)/np.maximum(low,.005),.7,1.4)
    image*=correction[:,:,None]
    smooth=ndimage.gaussian_filter(image,sigma=(1.,1.,0))
    image=.7*image+.3*smooth
    texture=dest/f'plane_{index:03d}.png';Image.fromarray(np.uint8(linear_to_srgb(image)*255+.5)).save(texture)
    vertex_uv=np.c_[((vertices-patch['centre'])@patch['basis'][:,0]-patch['low'][0])/(w*CELL),
        ((vertices-patch['centre'])@patch['basis'][:,1]-patch['low'][1])/(h*CELL)]
    mesh.compute_vertex_normals();_,near=tree.query(vertices,workers=2)
    mesh.vertex_colors=o3d.utility.Vector3dVector(np.asarray(reference.colors)[near])
    return mesh,vertex_uv[triangles],texture,{'cells':int(mask.sum()),'inferred_cells':int(np.sum(mask&~patch['observed_mask'])),
        'strong_prior_applied':strong_applied,'protected_rectangular_opening_cells':int(protected.sum()),
        'free_space_cells_rejected':rejected_count,
        'texture_photo_coverage_fraction':float(covered.mean()),'colour_source':colour_source,
        'luminance_gain_bounds':[.7,1.4],'texture_shape':[tw,th]}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true')
    p.add_argument('--profile',choices=['conservative','strong-indoor'],default='conservative');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest']
    if sha256_file(mp)!=ptr['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());previous=json.loads((a.output/'manifest.json').read_text()) if a.resume else {}
    hashes=previous.get('inputs_sha256',{});results=previous.get('sessions',{})
    if previous and previous.get('profile','conservative')!=a.profile:raise ValueError('Profile changed during resume')
    for path in [Path(__file__),mp,ROOT/'config/current_map.json',ROOT/'src/structural_prior_profiles.py']:
        current=sha256_file(path)
        if str(path) in hashes and hashes[str(path)]!=current:raise ValueError('Source changed during resume')
        hashes[str(path)]=current
    for name in a.sessions or selected['sessions']:
        if name in results:continue
        dest=a.output/name;dest.mkdir();s=selected['sessions'][name]
        effective_strong=a.profile=='strong-indoor' and name in ['floor2','lobby']
        paths=[mp.parent/s['cloud'],mp.parent/s['trajectory'],Path(selected['database_integrity'][name]['database']),
            ROOT/'outputs/student_union_appearance_v2'/name/'appearance_fixed.ply']
        for path in paths:hashes[str(path)]=sha256_file(path)
        reference=o3d.io.read_point_cloud(str(paths[0]));poses=load_pose_csv(paths[1])
        print(name+': robust finite structural patch detection',flush=True)
        patches,proposed,points=discover(o3d,reference)
        print(json.dumps({'session':name,'proposed_patches':proposed,'structural_patches':len(patches),'points':points}),flush=True)
        con=sqlite3.connect(f'file:{paths[2]}?mode=ro',uri=True)
        previous_texture=json.loads((ROOT/'outputs/student_union_preserved_display_v1/manifest.json').read_text())['sessions'][name]['texture']['camera_frames']
        frames={n:decode_frame(con,n,s['preprocessing'],poses[n]) for n in previous_texture};con.close()
        gains=json.loads((ROOT/'outputs/student_union_appearance_v2'/name/'exposure_gains.json').read_text())
        original=o3d.io.read_triangle_mesh(str(paths[3]));flags=np.load(paths[3].parent/'inferred_faces.npy');original.remove_triangles_by_mask(flags)
        original.compute_triangle_normals();v=np.asarray(original.vertices);t=np.asarray(original.triangles);centres=v[t].mean(1);normals=np.asarray(original.triangle_normals)
        replace=np.zeros(len(t),bool);models=[];metadata=[];uvs=[];textures=[]
        for index,patch in enumerate(patches):
            built=patch_mesh(o3d,patch,frames,reference,gains,dest,index,strong=effective_strong)
            if built is None:continue
            mesh,uv,texture,report=built
            q=(centres-patch['centre'])@patch['basis'];ij=np.floor((q[:,:2]-patch['low'])/CELL).astype(int);x,y=ij.T;h,w=patch['mask'].shape
            band=.32 if report['strong_prior_applied'] else .20
            ids=np.flatnonzero((x>=0)&(x<w)&(y>=0)&(y<h)&(abs(q[:,2])<band))
            inside=patch['mask'][y[ids],x[ids]];replace[ids[inside]]=True
            metadata.append({'centre':patch['centre'].tolist(),'basis':patch['basis'].tolist(),'low':patch['low'].tolist(),
                'shape':list(patch['mask'].shape),'support_points':patch['support_points'],'fit_p90_m':patch['fit_p90_m'],**report})
            models.append(mesh);uvs.append(uv);textures.append(texture)
        original.remove_triangles_by_mask(replace);original.remove_unreferenced_vertices();combined=o3d.geometry.TriangleMesh(original)
        vertices=[np.asarray(original.vertices)];triangles=[np.asarray(original.triangles)];normal=[np.asarray(original.vertex_normals)];colours=[np.asarray(original.vertex_colors)]
        pages=[np.full(len(original.triangles),-1)];face_uv=[np.zeros((len(original.triangles),3,2))];offset=len(original.vertices)
        for i,(mesh,uv) in enumerate(zip(models,uvs)):
            combined+=mesh;vertices.append(np.asarray(mesh.vertices));triangles.append(np.asarray(mesh.triangles)+offset)
            normal.append(np.asarray(mesh.vertex_normals));colours.append(np.asarray(mesh.vertex_colors));pages.append(np.full(len(mesh.triangles),i));face_uv.append(uv);offset+=len(mesh.vertices)
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'structural.ply'),combined)
        write_glb(dest/'structural_textured.glb',np.concatenate(vertices),np.concatenate(triangles),np.concatenate(normal),
            srgb_to_linear(np.concatenate(colours)),face_pages=np.concatenate(pages),face_uv=np.concatenate(face_uv),atlas_paths=textures)
        export_rgb(combined,dest/'structural_rgb.glb')
        results[name]={'session':name,'profile':'strong-indoor' if effective_strong else 'conservative',
            'patches':metadata,'replaced_original_faces':int(replace.sum()),'topology':topology(combined),
            'display_glb':'structural_textured.glb','pose_changes':0,'navigation_ready':False,'modeled_geometry':True,
            'outputs':{p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in dest.iterdir() if p.is_file()}}
        (dest/'summary.json').write_text(json.dumps(results[name],indent=2)+'\n')
        for path,h in hashes.items():
            if sha256_file(Path(path))!=h:raise ValueError('Original input changed')
        (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'inputs_sha256':hashes,
            'profile':a.profile,
            'selected_manifest_sha256':ptr['manifest_sha256'],'selected_map_changed':False,'navigation_ready':False,
            'limitations':['Finite planar display model, not survey or a new SLAM optimization','Large openings remain; thin unobserved details may be lost',
                'Photometric interpolation is not measured image content in unseen areas','Only large near-horizontal/vertical local patches; stairs not flattened']},indent=2)+'\n')
        print(json.dumps({'complete':name,'patches':len(models),'replaced_faces':int(replace.sum())}),flush=True)
        del frames,original,combined,reference;gc.collect()


if __name__=='__main__':main()
