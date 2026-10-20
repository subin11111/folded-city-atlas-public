#!/usr/bin/env python3
"""Visually reviewed photo polygons -> RGB-D-supported planes and semantic map objects."""
import argparse
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import numpy as np
from scipy.spatial import cKDTree,Delaunay
from PIL import Image,ImageDraw,ImageFont

from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now,apply
from surface_model_utils import decode_frame,unproject,project
from texture_boundary_leveling import level_boundaries,replace_glb_images
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT=Path(__file__).resolve().parents[1]


def lift_pixels(pixels,k,pose,centre,normal):
    rays=np.c_[(pixels[:,0]-k[0,2])/k[0,0],(pixels[:,1]-k[1,2])/k[1,1],np.ones(len(pixels))]@pose[:3,:3].T
    denom=rays@normal;t=((centre-pose[:3,3])@normal)/np.where(abs(denom)>1e-8,denom,np.nan)
    return pose[:3,3]+rays*t[:,None],t


def existing_horizontal_plane(frame,annotation,centres,normals):
    """Use the current local floor/ceiling, including other frames' support.

    The photo supplies appearance and a bounded extent. Missing depth at this
    particular view does not discard a plane already supported in the map.
    """
    uv,z=project(centres,frame['k_rgb'],frame['world_optical']);x,y=np.rint(np.clip(uv,-1e6,1e6)).astype(int).T
    camera_z=frame['world_optical'][2,3];kind=annotation['class']
    side=(centres[:,2]>camera_z+.25) if kind=='ceiling' else (centres[:,2]<camera_z-.5)
    ii=np.flatnonzero(side&(abs(normals[:,2])>.9)&(z>.25)&(z<8)&(x>=0)&(x<720)&(y>=0)&(y<960))
    poly=np.asarray(annotation['polygon'],np.int32);mask=np.zeros((960,720),np.uint8);cv2.fillPoly(mask,[poly],1)
    for padding in [30,70,140]:
        expanded=cv2.dilate(mask,np.ones((padding*2+1,padding*2+1),np.uint8))
        ids=ii[expanded[y[ii],x[ii]]>0]
        if len(ids)<30:continue
        height=centres[ids,2];hist,bins=np.histogram(height,bins=np.arange(height.min()-.1,height.max()+.2,.10))
        mode=(bins[hist.argmax()]+bins[hist.argmax()+1])/2;ids=ids[abs(height-mode)<.20]
        if len(ids)<30:continue
        for _ in range(3):
            points=centres[ids];centre=points.mean(0);_,_,vt=np.linalg.svd(points-centre,full_matrices=False);normal=vt[-1]
            residual=abs((points-centre)@normal);ids=ids[residual<=max(.015,np.quantile(residual,.9))]
        if abs(normal[2])<.95 or len(ids)<30:continue
        residual=abs((centres[ids]-centre)@normal)
        if np.percentile(residual,90)>.06:continue
        recorded=ids[np.linspace(0,len(ids)-1,min(256,len(ids))).astype(int)]
        return (centre,normal),{'accepted':True,'support_source':'existing_local_map_horizontal_faces',
            'support_points':len(ids),'map_support_face_indices':recorded.tolist(),'context_padding_rgb_pixels':padding,
            'median_plane_distance_m':float(np.median(residual)),'p90_plane_distance_m':float(np.percentile(residual,90))}
    return None,None


def existing_display_wall_plane(frame,annotation,measured_plane,centres,normals):
    """Project a small wall-mounted sign onto the already selected display wall.

    This is an appearance/decal anchor, not a new raw-depth measurement. It
    avoids deleting protected stair geometry just to expose a measured sign.
    """
    c,n=measured_plane;uv,z=project(centres,frame['k_rgb'],frame['world_optical'])
    x,y=np.rint(np.clip(uv,-1e6,1e6)).astype(int).T;mask=np.zeros(frame['rgb'].shape[:2],np.uint8)
    cv2.fillPoly(mask,[np.asarray(annotation['polygon'],np.int32)],1)
    h,w=mask.shape;ids=np.flatnonzero((z>.25)&(z<8)&(x>=0)&(x<w)&(y>=0)&(y<h)&
        (abs(normals[:,2])<.5)&(abs(normals@n)>.95)&(abs((centres-c)@n)<.4))
    ids=ids[mask[y[ids],x[ids]]>0]
    if len(ids)<6:return None,None
    offsets=(centres[ids]-c)@n;hist,bins=np.histogram(offsets,bins=np.arange(-.4,.425,.025))
    mode=(bins[hist.argmax()]+bins[hist.argmax()+1])/2;ids=ids[abs(offsets-mode)<.025]
    if len(ids)<6:return None,None
    ns=normals[ids].copy();ns[(ns@n)<0]*=-1;normal=ns.mean(0);normal/=np.linalg.norm(normal);centre=centres[ids].mean(0)
    residual=abs((centres[ids]-centre)@normal)
    if np.percentile(residual,90)>.025:return None,None
    return (centre,normal),{'accepted':True,'support_source':'existing_local_map_vertical_faces',
        'support_points':len(ids),'map_support_face_indices':ids[np.linspace(0,len(ids)-1,min(256,len(ids))).astype(int)].tolist(),
        'median_plane_distance_m':float(np.median(residual)),'p90_plane_distance_m':float(np.percentile(residual,90)),
        'display_plane_offset_from_same_frame_RGBD_m':float(abs((centre-c)@n)),
        'scope':'Photo appearance reprojected onto existing selected display wall; not a new sensor-depth constraint.'}


def fitted_plane(frame,annotation,initial,alternate_planes=(),mapped_tree=None):
    poly=np.asarray(annotation['polygon'],np.float32);h,w=frame['depth'].shape
    mask=np.zeros(frame['rgb'].shape[:2],np.uint8);cv2.fillPoly(mask,[poly.astype(np.int32)],1)
    if annotation.get('plane_support_polygons'):
        mask[:]=0
        for region in annotation['plane_support_polygons']:cv2.fillPoly(mask,[np.asarray(region,np.int32)],1)
    for excluded in annotation.get('colour_exclusions',[]):cv2.fillPoly(mask,[np.asarray(excluded,np.int32)],0)
    small=cv2.resize(mask,(w,h),interpolation=cv2.INTER_NEAREST)>0
    xyz=apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical'])
    valid=(frame['depth']>0).ravel();normal=np.asarray(initial['basis'])[:,2].copy();centre=np.asarray(initial['centre']).copy()
    distance=(xyz-centre)@normal
    band=.18 if annotation['class']=='wall' else .35
    region=valid&small.ravel();selected=region&(abs(distance)<band)
    origin='same_frame_surrounding_frame_depth' if annotation.get('plane_support_polygons') else 'same_frame_polygon_depth'
    local_report=None
    if region.sum()>=40 and mapped_tree is not None:
        raw_points=xyz[region];sample=raw_points[np.linspace(0,len(raw_points)-1,min(600,len(raw_points))).astype(int)]
        rng=np.random.default_rng(73);best=None
        for _ in range(96):
            tri=sample[rng.choice(len(sample),3,replace=False)];nc=np.cross(tri[1]-tri[0],tri[2]-tri[0]);length=np.linalg.norm(nc)
            if length<1e-6:continue
            nc/=length
            horizontal=annotation['class'] in ['floor','ceiling']
            if (horizontal and abs(nc[2])<.85) or (not horizontal and abs(nc[2])>.5):continue
            residual=abs((sample-tri[0])@nc);count=int(np.sum(residual<.035))
            if best is None or count>best[0]:best=(count,tri[0],nc)
        if best and best[0]>=len(sample)*.65:
            residual=abs((raw_points-best[1])@best[2]);inlier=raw_points[residual<.05]
            cc=inlier.mean(0);_,_,vt=np.linalg.svd(inlier-cc,full_matrices=False);nc=vt[-1]
            nn=mapped_tree.query(inlier[::max(1,len(inlier)//200)],workers=2)[0]
            if np.median(nn)<.12:
                centre=cc;normal=nc;distance=(xyz-centre)@normal;selected=region&(abs(distance)<.06)
                origin+='; local_RGBD_RANSAC_confirmed_by_selected_map'
                local_report={'mapped_point_nn_median_m':float(np.median(nn)),'local_ransac_inlier_fraction':best[0]/len(sample)}
    if region.sum()>=20 and (selected.sum()<20 or selected.sum()<region.sum()*.35):
        best=None
        for candidate in alternate_planes:
            nc=np.asarray(candidate['basis'])[:,2];cc=np.asarray(candidate['centre'])
            residual=abs((xyz[region]-cc)@nc);score=float(np.median(residual))
            if score<band and (best is None or score<best[0]):best=(score,cc,nc)
        if best is not None:
            _,centre,normal=best;distance=(xyz-centre)@normal;selected=region&(abs(distance)<band)
            origin+='; matched_other_mapped_plane'
    if selected.sum()<20:
        expanded=cv2.dilate(small.astype(np.uint8),np.ones((11,11),np.uint8))>0
        selected=valid&expanded.ravel()&(abs(distance)<band);origin='same_frame_nearby_depth'
    if selected.sum()<12:return None,{'accepted':False,'reason':'Too few nearby mapped RGB-D points','points':int(selected.sum())}
    points=xyz[selected];offset=(points-centre)@normal
    hist,bins=np.histogram(offset,bins=np.arange(-band,band+.025,.025));mode=(bins[hist.argmax()]+bins[hist.argmax()+1])/2
    keep=abs(offset-mode)<.065;points=points[keep]
    if len(points)<12:return None,{'accepted':False,'reason':'No coherent local plane'}
    centre=points.mean(0)
    if len(points)>=30:
        _,_,vt=np.linalg.svd(points-centre,full_matrices=False);candidate=vt[-1]
        if candidate@normal<0:candidate=-candidate
        if candidate@normal>np.cos(np.deg2rad(15)):normal=candidate
    residual=abs((points-centre)@normal)
    if annotation['class'] in ['floor','ceiling'] and abs(normal[2])<.85:return None,{'accepted':False,'reason':'Photo region has no supported horizontal surface'}
    if np.percentile(residual,90)>.08:return None,{'accepted':False,'reason':'Local plane inconsistent'}
    return (centre,normal),{'accepted':True,'support_points':len(points),'support_source':origin,**(local_report or {}),
        'median_plane_distance_m':float(np.median(residual)),'p90_plane_distance_m':float(np.percentile(residual,90))}


def photo_mesh(o3d,frame,annotation,plane,dest):
    centre,normal=plane;poly=np.asarray(annotation['polygon'],float)
    boundary,t=lift_pixels(poly,frame['k_rgb'],frame['world_optical'],centre,normal)
    if not np.isfinite(boundary).all() or np.min(t)<.25 or np.max(t)>8:return None,{'accepted':False,'reason':'Invalid ray-plane intersection'}
    if np.linalg.norm(np.ptp(boundary,axis=0))>8:return None,{'accepted':False,'reason':'Projected polygon implausibly large'}
    lo=np.floor(poly.min(0)).astype(int);hi=np.ceil(poly.max(0)).astype(int);hi=np.minimum(hi,[719,959]);lo=np.maximum(lo,0)
    rgb=frame['rgb'][lo[1]:hi[1]+1,lo[0]:hi[0]+1].copy();hh,ww=rgb.shape[:2]
    if annotation.get('colour_exclusions'):
        visible=np.zeros((hh,ww),np.uint8);cv2.fillPoly(visible,[(poly-lo).astype(np.int32)],1)
        excluded=np.zeros_like(visible)
        for region in annotation['colour_exclusions']:cv2.fillPoly(excluded,[(np.asarray(region)-lo).astype(np.int32)],1)
        valid=(visible>0)&(excluded==0)
        y,x=np.indices((hh,ww));design=np.c_[np.ones(hh*ww),x.ravel()/max(ww,1),y.ravel()/max(hh,1)]
        colours=srgb_to_linear(rgb/255.).reshape(-1,3);weight=valid.ravel().astype(float)
        for _ in range(6):
            root=np.sqrt(weight);coef=np.linalg.lstsq(design*root[:,None],colours*root[:,None],rcond=None)[0]
            residual=np.linalg.norm(design@coef-colours,axis=1);weight=valid.ravel()*np.minimum(1.,.05/np.maximum(residual,1e-9))
        predicted=linear_to_srgb(np.clip(design@coef,0,1)).reshape(hh,ww,3)
        rgb[excluded>0]=np.uint8(predicted[excluded>0]*255+.5)
    texture=dest/(annotation['id']+'.png');Image.fromarray(rgb).save(texture)
    xs=np.linspace(lo[0],hi[0],max(2,int(ww/12))+1);ys=np.linspace(lo[1],hi[1],max(2,int(hh/12))+1)
    xx,yy=np.meshgrid(xs,ys);pixels=np.c_[xx.ravel(),yy.ravel()]
    inside=np.array([cv2.pointPolygonTest(poly.astype(np.float32),tuple(p),False)>=0 for p in pixels])
    pixels=np.unique(np.r_[poly,pixels[inside]],axis=0)
    xyz,t=lift_pixels(pixels,frame['k_rgb'],frame['world_optical'],centre,normal)
    if not np.isfinite(xyz).all():return None,{'accepted':False,'reason':'Unstable interior rays'}
    triangles=Delaunay(pixels).simplices
    mid=pixels[triangles].mean(1)
    triangles=triangles[[cv2.pointPolygonTest(poly.astype(np.float32),tuple(p),False)>=0 for p in mid]]
    tri_xyz=xyz[triangles]
    front=np.cross(tri_xyz[:,1]-tri_xyz[:,0],tri_xyz[:,2]-tri_xyz[:,0])
    backwards=np.sum(front*(frame['world_optical'][:3,3]-tri_xyz.mean(1)),axis=1)<0
    triangles[backwards]=triangles[backwards][:,[0,2,1]]
    toward=np.sign((frame['world_optical'][:3,3]-centre)@normal)
    offset={'wall':.003,'door':.007,'window':.007,'sign':.012,'floor':.004,'ceiling':.004}[annotation['class']]
    xyz+=normal*toward*offset
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(xyz);mesh.triangles=o3d.utility.Vector3iVector(triangles)
    mesh.compute_vertex_normals();u=(pixels-lo)/np.maximum(hi-lo,1)
    colours=cv2.remap(frame['rgb'],pixels[:,0].astype(np.float32)[:,None],pixels[:,1].astype(np.float32)[:,None],cv2.INTER_LINEAR).reshape(-1,3)/255.
    mesh.vertex_colors=o3d.utility.Vector3dVector(colours)
    p=xyz[triangles];area=np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1).sum()/2
    if annotation['class'] in ['door','window'] and area>6:return None,{'accepted':False,'reason':'Door/window polygon exceeds the reviewed local scale'}
    return (mesh,u[triangles],texture),{'accepted':True,'world_boundary':boundary.tolist(),'world_centre':boundary.mean(0).tolist(),
        'plane_centre':centre.tolist(),'plane_normal':normal.tolist(),'area_m2':float(area),'triangles':len(triangles),
        'texture':texture.name,'texture_sha256':sha256_file(texture),'display_offset_m':offset,
        'triangle_front_faces_reference_camera':True,
        'occluded_wall_colour_inferred':bool(annotation.get('colour_exclusions'))}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source',type=Path,default=ROOT/'outputs/student_union_continuous_selected_v4')
    p.add_argument('--annotations',type=Path,default=ROOT/'config/photo_semantic_annotations.json');a=p.parse_args()
    a.output.mkdir(exist_ok=False,parents=True)
    import open3d as o3d
    source=json.loads((a.source/'manifest.json').read_text());run={**source,'sessions':{},'inputs_sha256':dict(source['inputs_sha256'])}
    spec=json.loads(a.annotations.read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    for path in [Path(__file__),a.annotations,a.source/'manifest.json',ROOT/'src/texture_boundary_leveling.py']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    semantics=[];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',20)
    for n,s in source['sessions'].items():
        dest=a.output/n;dest.mkdir();base=a.source/n
        objects=[v for v in spec['objects'] if v['session']==n];poses=load_pose_csv(mp.parent/selected['sessions'][n]['trajectory'])
        reference=o3d.io.read_point_cloud(str(mp.parent/selected['sessions'][n]['cloud']))
        mapped_tree=cKDTree(np.asarray(reference.points))
        replacements={};lighting=[]
        for path in sorted(base.glob('continuous_*.png')):
            index=int(path.stem.split('_')[-1]);rgb=np.asarray(Image.open(path).convert('RGB'));labels=np.asarray(Image.open(base/f'photo_labels_{index:03d}.png'))
            coverage=np.asarray(Image.open(base/f'coverage_{index:03d}.png'));leveled,report=level_boundaries(rgb,labels,coverage==255)
            target=dest/path.name;Image.fromarray(leveled).save(target);replacements[sha256_file(path)]=target;lighting.append({'plane_index':index,**report})
        leveled_glb=dest/'leveled_base.glb';changed=replace_glb_images(base/'clean_textured.glb',leveled_glb,replacements)
        old=o3d.io.read_triangle_mesh(str(base/'clean_structural.ply'));old.compute_triangle_normals()
        v=np.asarray(old.vertices);t=np.asarray(old.triangles);face_c=v[t].mean(1);normals=np.asarray(old.triangle_normals).copy();remove=np.zeros(len(t),bool)
        protected=np.zeros(len(t),bool)
        if s.get('stair_guard'):
            guard=s['stair_guard'];half=np.asarray(guard['box_half_extents_m']);tree=cKDTree(np.asarray(guard['box_centres'])/half)
            protected=(tree.query(v/half,p=np.inf,workers=2)[0][t].min(1)<=1.) | (tree.query(face_c/half,p=np.inf,workers=2)[0]<=1.)
        con=sqlite3.connect(f'file:{selected["database_integrity"][n]["database"]}?mode=ro',uri=True)
        meshes=[];uvs=[];textures=[];session_semantics=[]
        for obj in objects:
            sightline=spec.get('clear_sightline_review',{}).get('maximum_optical_gap_m',{}).get(obj['id'])
            if sightline is not None:obj={**obj,'reviewed_clear_sightline_max_optical_gap_m':sightline}
            frame=decode_frame(con,obj['node'],selected['sessions'][n]['preprocessing'],poses[obj['node']])
            pack=np.load(base/f'mask_{obj["plane_index"]:03d}.npz');initial={key:pack[key] for key in ['centre','basis']}
            plane,fit=fitted_plane(frame,obj,initial,s['patches'],mapped_tree);record={**obj,'review_method':spec['method'],'plane_fit':fit,'source_database':selected['database_integrity'][n]['database']}
            if obj['class'] in ['floor','ceiling']:
                mapped_plane,mapped_fit=existing_horizontal_plane(frame,obj,face_c,normals)
                if mapped_plane is not None:
                    mapped_fit['same_frame_RGBD_fit']=fit;plane,fit=mapped_plane,mapped_fit;record['plane_fit']=fit
            if plane is not None and obj.get('anchor_to_existing_display_wall'):
                mapped_plane,mapped_fit=existing_display_wall_plane(frame,obj,plane,face_c,normals)
                if mapped_plane is None:raise ValueError('No consistent existing wall for reviewed decal '+obj['id'])
                mapped_fit['same_frame_RGBD_fit']=fit;plane,fit=mapped_plane,mapped_fit;record['plane_fit']=fit
            original_path=dest/(obj['id']+'_original.png');Image.fromarray(frame['rgb']).save(original_path)
            record.update(source_rgb=original_path.name,source_rgb_sha256=sha256_file(original_path),
                K_rgb=frame['k_rgb'].tolist(),world_optical=frame['world_optical'].tolist())
            annotated=Image.fromarray(frame['rgb']);draw=ImageDraw.Draw(annotated);poly=[tuple(p) for p in obj['polygon']]
            draw.line(poly+[poly[0]],fill='#00ff66',width=4);draw.text((12,12),obj['id']+' / '+obj['class'],font=font,fill='#00ff66')
            annotated.save(dest/(obj['id']+'_source.jpg'),quality=95)
            if plane is None:record['surface_added']=False;session_semantics.append(record);continue
            built,geometry=photo_mesh(o3d,frame,obj,plane,dest);record['geometry']=geometry
            if built is None:record['surface_added']=False;session_semantics.append(record);continue
            mesh,uv,texture=built
            record['surface_added']=bool(obj['add_surface']);record['is_openable']=obj['class']=='door'
            record['navigation_class']=obj['class'];record['passability_verified']=False
            if obj['add_surface']:
                centre,normal=plane;pixels,z=project(face_c,frame['k_rgb'],frame['world_optical'])
                inside=np.zeros(frame['rgb'].shape[:2],np.uint8);cv2.fillPoly(inside,[np.asarray(obj['polygon'],np.int32)],1)
                x,y=np.rint(np.clip(pixels,-1e6,1e6)).astype(int).T
                orientation=(abs(normals[:,2])>.85) if obj['class'] in ['floor','ceiling'] else (abs(normals[:,2])<.5)
                ii=np.flatnonzero((z>.2)&(x>=0)&(x<720)&(y>=0)&(y<960)&(abs((face_c-centre)@normal)<(.06 if obj['class']=='wall' else .10))&(abs(normals@normal)>.75)&orientation&~protected)
                remove[ii[inside[y[ii],x[ii]]>0]]=True
                # An explicitly reviewed clear RGB sightline can contradict
                # an old duplicate wall in front of the photographed object.
                # Carve only vertical source faces in this bounded polygon,
                # never floors, stairs, protected corridor or furniture in
                # a wall-completion region. Bounds are per-object, not global.
                sightline_max=obj.get('reviewed_clear_sightline_max_optical_gap_m')
                if sightline_max is not None:
                    if obj['class'] not in ['door','window','sign']:raise ValueError('Sightline carving requires a reviewed semantic object')
                    ray=np.c_[(pixels[:,0]-frame['k_rgb'][0,2])/frame['k_rgb'][0,0],
                        (pixels[:,1]-frame['k_rgb'][1,2])/frame['k_rgb'][1,1],np.ones(len(pixels))]@frame['world_optical'][:3,:3].T
                    expected=((centre-frame['world_optical'][:3,3])@normal)/(ray@normal)
                    jj=np.flatnonzero((z>.2)&(z<expected-.003)&(expected-z<float(sightline_max))&
                        (x>=0)&(x<720)&(y>=0)&(y<960)&(abs(normals[:,2])<.5)&~protected)
                    carved=jj[inside[y[jj],x[jj]]>0];remove[carved]=True
                    record['reviewed_sightline_carved_source_faces']=len(carved)
                meshes.append(mesh);uvs.append(uv);textures.append(texture)
            session_semantics.append(record)
            print(n+': '+obj['id']+' -> '+str(record['surface_added']),flush=True)
        con.close();np.save(dest/'removed_source_faces.npy',remove)
        if protected.any():np.save(dest/'protected_stair_source_faces.npy',protected)
        residual=dest/'residual.glb';filter_glb(leveled_glb,residual,~remove)
        old.remove_triangles_by_mask(remove);old.remove_unreferenced_vertices();combined=o3d.geometry.TriangleMesh(old)
        for mesh in meshes:combined+=mesh
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'structural.ply'),combined)
        append_planes(residual,dest/'structural_textured.glb',meshes,uvs,textures)
        for path in base.glob('mask_*.npz'):shutil.copyfile(path,dest/path.name)
        for pattern in ['coverage_*.png','photo_labels_*.png']:
            for path in base.glob(pattern):shutil.copyfile(path,dest/path.name)
        (dest/'semantic_objects.json').write_text(json.dumps(session_semantics,indent=2,ensure_ascii=False)+'\n')
        (dest/'boundary_leveling.json').write_text(json.dumps(lighting,indent=2)+'\n')
        run['sessions'][n]={**s,'semantic_objects':session_semantics,'semantic_surfaces_added':len(meshes),
            'semantic_replaced_faces':int(remove.sum()),'boundary_leveling_images':changed,
            'source_mesh':str((base/'clean_structural.ply').resolve()),'horizontal_faces_removed':int(np.sum(remove&(abs(normals[:,2])>.985)))}
        (dest/'summary.json').write_text(json.dumps(run['sessions'][n],indent=2,ensure_ascii=False)+'\n')
        semantics+=session_semantics
    run.update(created_utc=utc_now(),semantic_display=True,semantic_annotation_method=spec['method'],
        semantic_scope='Photo-referenced semantic surfaces and observed door states; no automatic global semantic completeness claim.')
    (a.output/'semantic_objects.json').write_text(json.dumps({'frame':'selected_map','objects':semantics},indent=2,ensure_ascii=False)+'\n')
    (a.output/'manifest.json').write_text(json.dumps(run,indent=2,ensure_ascii=False)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed '+path)


if __name__=='__main__':main()
