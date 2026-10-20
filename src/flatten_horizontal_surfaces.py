#!/usr/bin/env python3
"""Strong finite floor/ceiling sheets, with free-ray and staircase protection."""
import argparse,gc,json,shutil,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from coherent_photo_charts import coherent_labels,flatten_photo_lighting
from export_rtabmap_db import load_pose_csv
from inspect_flat_surfaces import selected_sources
from optimize_session import apply,sha256_file,utc_now
from reconstruct_continuous_planes import solve_log_gains
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from surface_model_utils import decode_frame,unproject,project
from texture_boundary_leveling import level_boundaries

ROOT=Path(__file__).resolve().parents[1]


def disk(radius):
    y,x=np.mgrid[-radius:radius+1,-radius:radius+1];return x*x+y*y<=radius*radius


def strong_mask(observed,forbidden,cell=.1,closing_m=1.,extension_m=.2,maximum_distance_m=3.,minimum_area=1.5):
    """Close tears/fill bounded interiors, but never override explicit openings."""
    if not observed.any():return observed.copy()
    radius=max(1,int(round(closing_m/cell)))
    candidate=ndimage.binary_closing(observed,structure=disk(radius))|observed
    candidate=ndimage.binary_fill_holes(candidate)
    candidate=ndimage.binary_dilation(candidate,structure=disk(max(1,int(round(extension_m/cell)))))
    candidate&=ndimage.distance_transform_edt(~observed)*cell<=maximum_distance_m
    candidate&=~forbidden
    labels,_=ndimage.label(candidate);sizes=np.bincount(labels.ravel());keep=sizes*cell*cell>=minimum_area;keep[0]=False
    return candidate&keep[labels]


def spaced_nodes(poses,spacing=.55,maximum=180):
    chosen=[];positions=[]
    for node,pose in poses.items():
        point=pose[:3,3]
        if not positions or np.min(np.linalg.norm(np.asarray(positions)-point,axis=1))>=spacing:
            chosen.append(node);positions.append(point)
    if len(chosen)>maximum:chosen=[chosen[i] for i in np.linspace(0,len(chosen)-1,maximum).astype(int)]
    return chosen


def stair_guard(poses,source_summary,derive=False):
    if source_summary.get('stair_guard'):
        old=source_summary['stair_guard'];return np.asarray(old['box_centres']),np.asarray(old['box_half_extents_m']),'previous_reviewed_main_stair_corridor'
    if not derive:return np.empty((0,3)),np.array([.7,.7,1.1]),'none'
    ids=list(poses);p=np.array([poses[i][:3,3] for i in ids]);smooth=ndimage.median_filter(p[:,2],size=5,mode='nearest')
    climb=np.zeros(len(p),bool)
    for i in range(len(p)):
        lo=max(0,i-3);hi=min(len(p)-1,i+3);rise=abs(smooth[hi]-smooth[lo]);travel=np.linalg.norm(p[hi,:2]-p[lo,:2])
        climb[i]=rise>.28 and rise/max(travel,.2)>.22
    climb=ndimage.binary_dilation(climb,iterations=1)
    return p[climb]+[0,0,-.55],np.array([.65,.65,1.05]),'camera_height_transition_guard_not_semantic_stair_survey'


def protected_points(points,centres,half):
    if not len(centres):return np.zeros(len(points),bool)
    return cKDTree(centres/half).query(np.asarray(points)/half,p=np.inf,workers=2)[0]<=1.


def weighted_level(z,area,hint):
    # The strongest height mode is a display prior, not a global pose change.
    near=abs(z-hint)<.20
    if near.sum()<10:near=np.ones(len(z),bool)
    z=z[near];area=area[near];order=np.argsort(z);return float(z[order][np.searchsorted(np.cumsum(area[order]),area.sum()/2)])


def vertical_structure_faces(o3d,mesh,centres,normals,summary):
    """Distinguish standing walls/furniture from steep little floor shards."""
    protected=np.zeros(len(mesh.triangles),bool);ids=np.flatnonzero(abs(normals[:,2])<.65)
    if len(ids):
        vertical=o3d.geometry.TriangleMesh(mesh);keep=np.zeros(len(mesh.triangles),bool);keep[ids]=True
        vertical.remove_triangles_by_mask(~keep);labels,counts,areas=vertical.cluster_connected_triangles();labels=np.asarray(labels);areas=np.asarray(areas)
        zmin=np.full(len(areas),np.inf);zmax=np.full(len(areas),-np.inf)
        xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)[ids]]
        np.minimum.at(zmin,labels,xyz[:,:,2].min(1));np.maximum.at(zmax,labels,xyz[:,:,2].max(1))
        standing=(zmax-zmin>=.45)&(areas>=.015);protected[ids]=standing[labels]
    # Exact existing continuous-wall primitives are always retained, even if
    # one small disconnected piece has a low local height span.
    for plane in summary.get('continuous_planes',[]):
        n=np.asarray(plane['basis'])[:,2];c=np.asarray(plane['centre'])
        if abs(n[2])>.3:continue
        near=(abs((centres-c)@n)<2e-5)&(abs(normals@n)>.995)
        protected|=near
    return protected


def ray_evidence(frames,height,low,shape,cell,margin=.30):
    support=np.zeros(shape,np.uint16);free=np.zeros(shape,np.uint16);h,w=shape
    for f in frames.values():
        valid=f['depth']>0;local=unproject(f['depth'],f['k_depth'])[valid];p=apply(local,f['world_optical']);camera=f['world_optical'][:3,3]
        for values,target in [(p[abs(p[:,2]-height)<.18,:2],support)]:
            xy=np.floor((values-low)/cell).astype(int);x,y=xy.T;ok=(x>=0)&(x<w)&(y>=0)&(y<h)
            keys=np.unique(y[ok]*w+x[ok]);target.ravel()[keys]+=1
        denominator=p[:,2]-camera[2];t=(height-camera[2])/np.where(abs(denominator)>1e-8,denominator,np.nan)
        crossing=(t>.02)&(t<.98)&(abs(p[:,2]-height)>margin)
        xy=camera[:2]+(p[crossing,:2]-camera[:2])*t[crossing,None]
        ij=np.floor((xy-low)/cell).astype(int);x,y=ij.T;ok=(x>=0)&(x<w)&(y>=0)&(y<h)
        keys=np.unique(y[ok]*w+x[ok]);free.ravel()[keys]+=1
    return support,free


def build_sheet(o3d,mask,low,height,cell,kind):
    h,w=mask.shape;yy,xx=np.indices((h+1,w+1));v=np.c_[xx.ravel()*cell+low[0],yy.ravel()*cell+low[1],np.full(xx.size,height)]
    indices=np.arange((h+1)*(w+1)).reshape(h+1,w+1);aa=indices[:-1,:-1][mask];b=aa+1;c=aa+w+1;d=c+1
    tri=np.r_[np.c_[aa,b,c],np.c_[b,d,c]]
    if kind=='ceiling':tri=tri[:,[0,2,1]]
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(v);mesh.triangles=o3d.utility.Vector3iVector(tri)
    mesh.compute_vertex_normals();uv=np.c_[xx.ravel()/w,yy.ravel()/h]
    return mesh,uv[tri]


def view_samples(frame,points,height,kind,tolerance=.45):
    uv,z=project(points,frame['k_rgb'],frame['world_optical']);rh,rw=frame['rgb'].shape[:2];dh,dw=frame['depth'].shape
    x,y=uv.T;valid=(z>.3)&(z<6.)&(x>=1)&(x<rw-2)&(y>=1)&(y<rh-2)
    x=np.nan_to_num(x,nan=-1e5,posinf=1e5,neginf=-1e5);y=np.nan_to_num(y,nan=-1e5,posinf=1e5,neginf=-1e5)
    dx=np.clip(np.rint(x*dw/rw),0,dw-1).astype(int);dy=np.clip(np.rint(y*dh/rh),0,dh-1).astype(int)
    depth=frame['depth'][dy,dx];delta=abs(depth-z);valid&=(depth>0)&(delta<tolerance)
    distance=np.linalg.norm(points-frame['world_optical'][:3,3],axis=1)
    incidence=abs(height-frame['world_optical'][2,3])/np.maximum(distance,.05)
    valid&=incidence>.18
    score=np.where(valid,incidence**2*np.exp(-delta**2/.12)/(1+distance**2*.1),0)
    # Flat point lists can exceed OpenCV remap's 32767-row limit. Sample
    # bilinearly as arrays instead, without inventing a large skinny image.
    sx=np.clip(x,0,rw-1);sy=np.clip(y,0,rh-1);x0=np.floor(sx).astype(int);y0=np.floor(sy).astype(int)
    x1=np.minimum(x0+1,rw-1);y1=np.minimum(y0+1,rh-1);ax=(sx-x0)[:,None];ay=(sy-y0)[:,None];im=frame['rgb']
    rgb=((1-ay)*((1-ax)*im[y0,x0]+ax*im[y0,x1])+ay*((1-ax)*im[y1,x0]+ax*im[y1,x1]))/255.
    return rgb,score


def sheet_texture(mask,low,height,kind,frames,source_points,source_colours,dest,plane_id,cfg):
    h,w=mask.shape;cell=cfg['cell_m'];side=cfg['texture_maximum_side_px'];metres=cfg['texture_metres_per_pixel']
    tw=max(2,min(side,int(w*cell/metres)));th=max(2,min(side,int(h*cell/metres)))
    cw=max(2,min(280,tw));ch=max(2,min(280,th));yy,xx=np.indices((ch,cw))
    coarse=np.c_[low[0]+(xx.ravel()+.5)*w*cell/cw,low[1]+(yy.ravel()+.5)*h*cell/ch,np.full(xx.size,height)]
    candidates=[]
    for node,f in frames.items():
        dz=f['world_optical'][2,3]-height
        if (kind=='floor' and not .6<dz<2.8) or (kind=='ceiling' and not -3.5<dz<-.25):continue
        rgb,score=view_samples(f,coarse,height,kind)
        if np.count_nonzero(score)>15:candidates.append((node,rgb.reshape(ch,cw,3),score.reshape(ch,cw)))
    candidates.sort(key=lambda v:-v[2].sum());candidates=candidates[:cfg['texture_maximum_frames']]
    yy,xx=np.indices((th,tw));points=np.c_[low[0]+(xx.ravel()+.5)*w*cell/tw,low[1]+(yy.ravel()+.5)*h*cell/th,np.full(xx.size,height)]
    near=cKDTree(source_points[:,:2]).query(points[:,:2],workers=2)[1]
    fallback=srgb_to_linear(source_colours[near]).reshape(th,tw,3)
    fallback=ndimage.gaussian_filter(fallback,(max(1.,.12/metres),max(1.,.12/metres),0))
    result=fallback.copy();labels=np.full((th,tw),65535,np.uint16);covered=np.zeros((th,tw),bool);gains=[]
    if candidates:
        images=[srgb_to_linear(x[1]) for x in candidates];weights=np.array([x[2] for x in candidates])
        gains=solve_log_gains(images,list(weights));small_labels=coherent_labels(weights,step=2,potts=.5)
        selected=cv2.resize(small_labels.astype(np.float32),(tw,th),interpolation=cv2.INTER_NEAREST).astype(int).ravel()
        for index,(node,_,_) in enumerate(candidates):
            indices=np.flatnonzero(selected==index);rgb,score=view_samples(frames[node],points[indices],height,kind);ok=score>0;ii=indices[ok]
            result.reshape(-1,3)[ii]=srgb_to_linear(rgb[ok])*gains[index];covered.ravel()[ii]=True;labels.ravel()[ii]=index
    # Fade only the photograph/fallback boundary, retaining detail inside
    # source photographs, then level broad illumination with bounded gains.
    feather=np.clip(ndimage.distance_transform_edt(covered)*max(w*cell/tw,h*cell/th)/.12,0,1)
    result=result*feather[:,:,None]+fallback*(1-feather[:,:,None])
    mask_tex=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0
    result,chart_lighting=flatten_photo_lighting(result,mask_tex,max(w*cell/tw,h*cell/th))
    rgb=np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5);rgb,lighting=level_boundaries(rgb,labels,covered)
    path=dest/(plane_id+'_texture.png');Image.fromarray(rgb).save(path)
    Image.fromarray(np.uint8(np.where(mask_tex,np.where(covered,255,80),0))).save(dest/(plane_id+'_photo_coverage.png'))
    return path,{'original_photo_nodes':[x[0] for x in candidates],'photo_fraction_on_sheet':float(covered[mask_tex].mean()),
        'texture_shape':[tw,th],'fallback':'smoothed_existing_horizontal_surface_RGB_not_generated_photography',
        'photograph_depth_tolerance_m':.45,'boundary_leveling':lighting,'bounded_chart_lighting':chart_lighting,'photo_boundary_feather_m':.12}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,default=ROOT/'config/flat_horizontal_surfaces.json');a=p.parse_args()
    cfg=json.loads(a.config.read_text());source_mp=ROOT/cfg['source_manifest'];source=json.loads(source_mp.read_text());base=source_mp.parent
    policy_path=ROOT/cfg['source_policy'];policy=json.loads(policy_path.read_text())
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    if sha256_file(mp)!=ptr['manifest_sha256'] or source['selected_manifest_sha256']!=ptr['manifest_sha256']:raise ValueError('Selected map changed')
    a.output.mkdir(parents=True,exist_ok=False)
    import open3d as o3d
    run={**source,'sessions':{},'inputs_sha256':dict(source['inputs_sha256']),'flat_horizontal_display':True,
        'base_display_manifest':str(source_mp),'base_display_policy':policy,'flat_config':cfg,'pose_changes':0}
    for path in [Path(__file__),a.config,source_mp,policy_path,ROOT/'src/append_plane_glb.py',ROOT/'src/inspect_flat_surfaces.py']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    semantic=[];global_floor_masks={};cell=cfg['cell_m']
    # Lobby first supplies an indoor-only clip for the exterior bridge.
    order=['lobby','floor2','bridge_12_sub','bridge_out1f','outside']
    for name in order:
        print('Begin flat sheets: '+name,flush=True);dest=a.output/name;dest.mkdir();s=source['sessions'][name];spec=cfg['sessions'][name]
        ply,glb=selected_sources(name,base,policy)
        run['inputs_sha256'][str(ply)]=sha256_file(ply);run['inputs_sha256'][str(glb)]=sha256_file(glb)
        old=o3d.io.read_triangle_mesh(str(ply));old.compute_triangle_normals();v=np.asarray(old.vertices);tri=np.asarray(old.triangles);xyz=v[tri];centres=xyz.mean(1)
        normals=np.asarray(old.triangle_normals).copy();areas=np.linalg.norm(np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]),axis=1)/2
        colours=np.asarray(old.vertex_colors);face_colours=colours[tri].mean(1) if len(colours) else np.full((len(tri),3),.5)
        poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);guard_c,guard_half,guard_source=stair_guard(poses,s,spec.get('derive_stair_guard',False))
        protected=protected_points(v,guard_c,guard_half)[tri].any(1)|protected_points(centres,guard_c,guard_half)
        semantic_objects=s.get('semantic_objects',[]);semantic_ids=np.zeros(len(tri),bool)
        # Original semantic primitives are last, in their recorded order.
        cursor=len(tri)-sum(o.get('geometry',{}).get('triangles',0) for o in semantic_objects if o.get('surface_added'))
        semantic_ranges={}
        for obj in semantic_objects:
            if not obj.get('surface_added'):continue
            stop=cursor+obj['geometry']['triangles'];semantic_ranges[obj['id']]=(cursor,stop)
            if obj['class'] not in ['floor','ceiling']:semantic_ids[cursor:stop]=True
            cursor=stop
        structures=vertical_structure_faces(o3d,old,centres,normals,s)
        np.save(dest/'vertical_structure_source_faces.npy',structures)
        protected|=semantic_ids|structures
        np.save(dest/'protected_source_faces.npy',protected);remove=np.zeros(len(tri),bool);meshes=[];uvs=[];textures=[];reports=[];frames={}
        if not spec.get('preserve_entire_source'):
            nodes=spaced_nodes(poses);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
            frames={node:decode_frame(con,node,selected['sessions'][name]['preprocessing'],poses[node]) for node in nodes};con.close()
        local_masks={}
        for plane in spec.get('planes',[]):
            lower,upper=plane['height_band_m'];ids=np.flatnonzero((centres[:,2]>=lower)&(centres[:,2]<upper)&(abs(normals[:,2])>.85)&(areas>1e-7)&~protected)
            if len(ids)<20 or areas[ids].sum()<1.5:
                reports.append({**plane,'accepted':False,'reason':'Insufficient unprotected horizontal support'});continue
            height=weighted_level(centres[ids,2],areas[ids],plane['mode_hint_m'])
            xy=centres[ids,:2];low=np.floor(xy.min(0)/cell)*cell-1.5;high=np.ceil(xy.max(0)/cell)*cell+1.5
            w,h=np.ceil((high-low)/cell).astype(int);shape=(h,w);index=np.floor((xy-low)/cell).astype(int);observed=np.zeros(shape,bool);observed[index[:,1],index[:,0]]=True
            observed=ndimage.binary_dilation(observed,structure=disk(1))
            support,free=ray_evidence(frames,height,low,shape,cell,cfg['free_space_endpoint_margin_m'])
            yy,xx=np.indices(shape);grid=np.c_[low[0]+(xx.ravel()+.5)*cell,low[1]+(yy.ravel()+.5)*cell,np.full(xx.size,height)]
            stair=protected_points(grid,guard_c,guard_half+[cell,cell,.001]).reshape(shape)
            free_guard=(free>=cfg['free_space_minimum_distinct_views'])&(free>support*1.5+1)
            free_guard=ndimage.binary_dilation(free_guard,structure=disk(int(np.ceil(cfg['free_space_guard_dilation_m']/cell))))
            forbidden=stair|free_guard
            if plane.get('clip_to_session_floor'):
                ref=global_floor_masks[plane['clip_to_session_floor']];ij=np.floor((grid[:,:2]-ref['low'])/cell).astype(int);x,y=ij.T
                clip=np.zeros(len(grid),bool);hh,ww=ref['mask'].shape;ii=np.flatnonzero((x>=0)&(x<ww)&(y>=0)&(y<hh));clip[ii]=ref['mask'][y[ii],x[ii]]
                clip=ndimage.binary_dilation(clip.reshape(shape),structure=disk(int(plane.get('clip_margin_m',0)/cell)))
                forbidden|=~clip
            mask=strong_mask(observed,forbidden,cell,cfg['closing_radius_m'],cfg['boundary_extension_m'],cfg['maximum_extrapolation_from_observed_m'],cfg['minimum_region_area_m2'])
            if plane.get('paired_floor') in local_masks:
                floor=local_masks[plane['paired_floor']];ij=np.floor((grid[:,:2]-floor['low'])/cell).astype(int);x,y=ij.T;fh,fw=floor['mask'].shape
                inherit=np.zeros(len(grid),bool);ii=np.flatnonzero((x>=0)&(x<fw)&(y>=0)&(y<fh));inherit[ii]=floor['mask'][y[ii],x[ii]]
                nearby=ndimage.distance_transform_edt(~observed)*cell<=cfg['maximum_extrapolation_from_observed_m']
                mask|=inherit.reshape(shape)&nearby&~forbidden
            if mask.sum()*cell*cell<cfg['minimum_region_area_m2']:
                reports.append({**plane,'accepted':False,'reason':'All support lies in protected stairs/free-space'});continue
            mesh,uv=build_sheet(o3d,mask,low,height,cell,plane['kind']);texture,photo=sheet_texture(mask,low,height,plane['kind'],frames,centres[ids],face_colours[ids],dest,plane['id'],cfg)
            n=np.floor((centres[:,:2]-low)/cell).astype(int);x,y=n.T;layer_face=(abs(normals[:,2])>.30)|(np.ptp(xyz[:,:,2],axis=1)<.35)
            ii=np.flatnonzero((x>=0)&(x<w)&(y>=0)&(y<h)&~protected&
                (centres[:,2]>=lower)&(centres[:,2]<upper)&layer_face)
            remove[ii[mask[y[ii],x[ii]]]]=True
            # Old horizontal semantic patches are superseded as whole objects;
            # their original photographs/semantic observations remain available.
            for obj in semantic_objects:
                if obj.get('surface_added') and obj['class']==plane['kind'] and lower<obj['geometry']['world_centre'][2]<upper:
                    begin,end=semantic_ranges[obj['id']];remove[begin:end]=~protected[begin:end]
            path=dest/(plane['id']+'_mask.npz');np.savez_compressed(path,mask=mask,observed=observed,forbidden=forbidden,stair_guard=stair,free_guard=free_guard,support=support,free=free,low=low,height=height,cell=cell)
            meshes.append(mesh);uvs.append(uv);textures.append(texture);local_masks[plane['id']]={'mask':mask,'low':low,'height':height}
            if name=='lobby' and plane['kind']=='floor':global_floor_masks[name]=local_masks[plane['id']]
            deviation=centres[ids,2]-height
            reports.append({**plane,'accepted':True,'height_m':height,'area_xy_m2':float(mask.sum()*cell*cell),'model_inferred_area_m2':float((mask&~observed).sum()*cell*cell),
                'observed_raster_area_m2':float(observed.sum()*cell*cell),'triangles':len(mesh.triangles),'height_span_m':0.,'maximum_abs_height_deviation_m':0.,
                'source_height_deviation_p90_m':float(np.percentile(abs(deviation),90)),'source_height_span_m':float(np.ptp(centres[ids,2])),
                'guarded_stair_cells':int(stair.sum()),'free_space_excluded_cells':int(free_guard.sum()),'mask_file':path.name,**photo})
            print(name+': '+plane['id']+' / '+str(round(mask.sum()*cell*cell,2))+' m2 flat at Z='+str(round(height,3)),flush=True)
        np.save(dest/'removed_source_faces.npy',remove);filtered=dest/'retained.glb';filter_glb(glb,filtered,~remove)
        remap=np.full(len(tri),-1,int);remap[~remove]=np.arange((~remove).sum())
        updated_semantics=[]
        for obj in semantic_objects:
            obj={**obj}
            if obj.get('surface_added'):
                begin,end=semantic_ranges[obj['id']];retained=remap[begin:end];retained=retained[retained>=0]
                obj['current_triangle_indices']=retained.tolist()
                if not len(retained):obj['surface_added']=False;obj['superseded_by_flat_horizontal_sheet']=True
            semantic.append(obj);updated_semantics.append(obj)
            for pattern in [obj.get('source_rgb',''),obj.get('geometry',{}).get('texture','')]:
                if pattern and (base/name/pattern).exists():shutil.copyfile(base/name/pattern,dest/pattern)
        old.remove_triangles_by_mask(remove);old.remove_unreferenced_vertices();combined=o3d.geometry.TriangleMesh(old)
        for mesh in meshes:combined+=mesh
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),combined)
        append_planes(filtered,dest/'clean_textured.glb',meshes,uvs,textures)
        s={**s,'semantic_objects':updated_semantics,'flat_sheets':reports,'source_mesh':str(ply),'source_texture':str(glb),'flat_replaced_source_faces':int(remove.sum()),
            'flat_protected_source_faces':int(protected.sum()),'stair_guard_kind':guard_source,
            'flat_stair_guard':{'centres':guard_c.tolist(),'half':guard_half.tolist()},'all_semantic_vertical_faces_retained':True}
        (dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n');run['sessions'][name]=s
        (a.output/'semantic_objects.json').write_text(json.dumps({'frame':'selected_map','objects':semantic},indent=2,ensure_ascii=False)+'\n')
        run.update(created_utc=utc_now(),scope='Strong finite horizontal display sheets; not pose optimization or surveyed navigation geometry.')
        (a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n')
        del old,combined,xyz,frames;gc.collect()
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)


if __name__=='__main__':main()
