#!/usr/bin/env python3
"""One presentation model: common levels, a continuous spandrel, real ceiling zones.

This is architectural display completion, NOT pose optimization or navigation
geometry. Original DBs/poses/XY/photo texels remain separate and unchanged.
"""
import argparse,gc,json,shutil,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.interpolate import PchipInterpolator
from scipy.spatial import cKDTree,Delaunay

from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from complete_verified_upper_walls import inside_polygon
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from photo_semantic_surfaces import lift_pixels
from presentation_geometry import LevelPresentation,warp_glb
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame,project,unproject

ROOT=Path(__file__).resolve().parents[1]


def clip_halfspace(poly,function):
    output=[];poly=np.asarray(poly,float)
    for first,second in zip(poly,np.roll(poly,-1,axis=0)):
        a,b=float(function(first)),float(function(second));ina,inb=a>=0,b>=0
        if ina:output.append(first)
        if ina!=inb:output.append(first+(second-first)*a/(a-b))
    return np.asarray(output).reshape(-1,2)


def bounded_photo_polygon(poly,frame,height):
    k=frame['k_rgb'];rotation=frame['world_optical'][:3,:3];numer=height-frame['world_optical'][2,3]
    if numer<=0:raise ValueError('Ceiling is not above its original camera')
    def denom(p):return np.array([(p[0]-k[0,2])/k[0,0],(p[1]-k[1,2])/k[1,1],1.])@rotation[2]
    result=clip_halfspace(poly,lambda p:denom(p)-numer/7.95)
    return clip_halfspace(result,lambda p:numer/.251-denom(p)) if len(result) else result


def ceiling_fit(points,max_tilt=20.):
    if len(points)<30:return None,{'accepted':False,'reason':'No original ceiling depth'}
    rng=np.random.default_rng(932);sample=points[rng.choice(len(points),min(1800,len(points)),replace=False)];best=None
    for _ in range(320):
        tri=sample[rng.choice(len(sample),3,replace=False)];n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);length=np.linalg.norm(n)
        if length<1e-7:continue
        n/=length
        if abs(n[2])<np.cos(np.deg2rad(max_tilt)):continue
        used=abs((sample-tri[0])@n)<.04
        if best is None or used.sum()>best[0]:best=(int(used.sum()),tri[0],n)
    if best is None:return None,{'accepted':False,'reason':'No photographed horizontal plane'}
    used=abs((points-best[1])@best[2])<.06
    for _ in range(4):
        p=points[used];centre=np.median(p,0);_,_,vt=np.linalg.svd(p-centre,full_matrices=False);normal=vt[-1];used=abs((points-centre)@normal)<.06
    p=points[used];p90=float(np.percentile(abs((p-centre)@normal),90));accepted=len(p)>=30 and used.mean()>.55 and p90<.06 and abs(normal[2])>np.cos(np.deg2rad(max_tilt))
    return ((centre,normal) if accepted else None),{'accepted':bool(accepted),'support_points':len(p),'total_points':len(points),'p90_m':p90,'centre':centre.tolist(),'normal':normal.tolist()}


def photo_ceiling_mesh(o3d,frame,region,source_height,display_height,model,dest,atrium):
    poly=np.asarray(region['polygon'],float);lo=poly.min(0);hi=poly.max(0);xx,yy=np.meshgrid(np.linspace(lo[0],hi[0],max(3,int((hi[0]-lo[0])/12))),np.linspace(lo[1],hi[1],max(3,int((hi[1]-lo[1])/12))))
    pixels=np.unique(np.r_[poly,np.c_[xx.ravel(),yy.ravel()]],axis=0);pixels=pixels[inside_polygon(pixels,poly)]
    xyz,t=lift_pixels(pixels,frame['k_rgb'],frame['world_optical'],[0,0,source_height],[0,0,1]);valid=np.isfinite(xyz).all(1)&(t>.25)&(t<8.)
    pixels,xyz=pixels[valid],xyz[valid];xyz=model.warp(xyz,region['session']);xyz[:,2]=display_height
    if len(xyz)<8:raise ValueError('Insufficient finite photographed ceiling polygon')
    triangles=Delaunay(pixels).simplices;centres=pixels[triangles].mean(1);keep=inside_polygon(centres,poly)
    lengths=np.linalg.norm(np.roll(xyz[triangles],1,axis=1)-xyz[triangles],axis=2).max(1);keep&=lengths<1.
    world=xyz[triangles].mean(1)
    if display_height<model.upper-.2:
        ij=np.floor((world[:,:2]-atrium['low'])/atrium['cell']).astype(int);x,y=ij.T;h,w=atrium['mask'].shape;inside=(x>=0)&(x<w)&(y>=0)&(y<h);core=np.zeros(len(world),bool);ii=np.flatnonzero(inside);core[ii]=atrium['core'][y[ii],x[ii]];keep&=~core
    triangles=triangles[keep][:,[0,2,1]];mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(xyz);mesh.triangles=o3d.utility.Vector3iVector(triangles);mesh.compute_vertex_normals()
    lower=np.maximum(np.floor(lo).astype(int),0);upper=np.minimum(np.ceil(hi).astype(int),[719,959]);texture=frame['rgb'][lower[1]:upper[1]+1,lower[0]:upper[0]+1].copy()
    # One coherent original-photo chart, bounded display exposure, no generated
    # content or per-triangle photo switching.
    luminance=np.median(texture.mean(2));gain=float(np.clip(165/max(luminance,1),.85,1.5));texture=np.uint8(np.clip(texture.astype(float)*gain,0,255));path=dest/(region['id']+'_texture.png');Image.fromarray(texture).save(path)
    uv=(pixels-lower)/np.maximum(upper-lower,1);return mesh,uv[triangles],path,{'exposure_gain':gain,'source_height_m':source_height,'display_joint_height_m':display_height,'triangles':len(triangles),'area_m2':float(mesh.get_surface_area())}


def ceiling_union(o3d,records,frames,h,z,model,dest,atrium):
    boundaries=[]
    for record in records:
        region=record['region'];f=frames[region['id']];poly=bounded_photo_polygon(region['polygon'],f,h)
        if len(poly)<3:continue
        q,t=lift_pixels(poly,f['k_rgb'],f['world_optical'],[0,0,h],[0,0,1]);ok=np.isfinite(q).all(1)&(t>.25)&(t<8.)
        if not ok.all():raise ValueError('Unbounded reviewed ceiling polygon')
        boundaries.append(q[:,:2])
    cell=.045;all_xy=np.concatenate(boundaries);low=np.floor(all_xy.min(0)/cell)*cell;high=np.ceil(all_xy.max(0)/cell)*cell;w,hh=np.ceil((high-low)/cell).astype(int)+1;mask=np.zeros((hh,w),np.uint8)
    for xy in boundaries:cv2.fillPoly(mask,[np.rint((xy-low)/cell).astype(np.int32)],1)
    yy,xx=np.indices(mask.shape);centres=low+np.c_[xx.ravel()+.5,yy.ravel()+.5]*cell;ij=np.floor((centres-atrium['low'])/atrium['cell']).astype(int);x,y=ij.T;ah,aw=atrium['core'].shape;inside=(x>=0)&(x<aw)&(y>=0)&(y<ah);ii=np.flatnonzero(inside);forbidden=np.zeros(len(x),bool);forbidden[ii]=atrium['core'][y[ii],x[ii]];mask.ravel()[forbidden]=0
    yy,xx=np.indices((hh+1,w+1));xy=low+np.c_[xx.ravel(),yy.ravel()]*cell;vertices=np.c_[xy,np.full(len(xy),z)];index=np.arange(len(xy)).reshape(hh+1,w+1);aa=index[:-1,:-1][mask>0];b=aa+1;c=aa+w+1;d=c+1;tri=np.r_[np.c_[aa,c,b],np.c_[b,c,d]];mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(vertices);mesh.triangles=o3d.utility.Vector3iVector(tri);mesh.compute_vertex_normals();uv=(xy-low)/(np.array([w,hh])*cell)
    # One UV chart for the union, with a broad photo-label transition. There
    # are no coplanar duplicate ceiling sheets and no per-triangle mosaic.
    tw,th=max(8,int(w*cell/.025)),max(8,int(hh*cell/.025));ty,tx=np.indices((th,tw));qxy=low+np.c_[(tx.ravel()+.5)*w*cell/tw,(ty.ravel()+.5)*hh*cell/th];world=np.c_[qxy,np.full(len(qxy),h)];result=np.full((th*tw,3),210.,float);covered=np.zeros(th*tw,bool);gains=[]
    for record in records:
        region=record['region'];f=frames[region['id']];px,depth=project(world,f['k_rgb'],f['world_optical']);good=inside_polygon(px,region['polygon'])&(depth>.25)&(depth<8.);rgb=cv2.remap(f['rgb'],px[:,0].reshape(th,tw).astype(np.float32),px[:,1].reshape(th,tw).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT).reshape(-1,3).astype(float)
        gain=float(np.clip(175/max(np.median(rgb[good].mean(1)) if good.any() else 175,1),.85,1.5));rgb=np.clip(rgb*gain,0,255);overlap=good&covered
        if overlap.sum()>100:rgb*=np.clip(np.median(result[overlap].mean(1))/max(np.median(rgb[overlap].mean(1)),1),.8,1.25)
        take=good&~covered;result[take]=rgb[take];covered|=good;gains.append(gain)
    texture=dest/'restroom_soffit_union_texture.png';Image.fromarray(np.uint8(np.clip(result.reshape(th,tw,3),0,255))).save(texture);np.savez_compressed(dest/'restroom_soffit_union_mask.npz',mask=mask>0,low=low,cell=cell,forbidden=forbidden.reshape(hh,w))
    return mesh,uv[tri],texture,{'id':'restroom_soffit_union','accepted':True,'kind':'photographed_ceiling','source_height_m':h,'display_joint_height_m':z,'triangles':len(tri),'area_m2':float(mesh.get_surface_area()),'photographs':[r['region']['node'] for r in records],'photo_exposure_gains':gains,'coplanar_duplicate_sheets':0,'mask_file':'restroom_soffit_union_mask.npz'}


def exterior_profile(points):
    lo,hi=np.percentile(points[:,0],[1,99]);xs=np.arange(lo,hi+.03,.035);ys=[]
    for x in xs:
        q=points[abs(points[:,0]-x)<.08,1];ys.append(float(np.percentile(q,97)) if len(q) else np.nan)
    valid=np.isfinite(ys);ys=np.interp(xs,xs[valid],np.asarray(ys)[valid]);ys=ndimage.gaussian_filter1d(ys,1.5)
    return np.c_[xs,ys+.04]


def make_spandrel(o3d,points,bottom_z,model,dest,cfg):
    # The opaque parapet is white on its indoor side and blue-gray outside.
    # Reconstruct both sides; a double-sided white source patch otherwise
    # shows its inside colour through the missing outside spandrel.
    front=exterior_profile(points);back=front-[0,cfg['spandrel_nominal_backing_m']];upper=model.warp(np.c_[front,np.full(len(front),cfg['spandrel_opaque_cap_source_m'])],'floor2')[:,2]
    vertices=np.r_[np.c_[front,np.full(len(front),bottom_z)],np.c_[front,upper],np.c_[back,np.full(len(back),bottom_z)],np.c_[back,upper]];n=len(front);outside=[];inside=[]
    for i in range(n-1):
        outside.extend([[i,i+1,n+i],[i+1,n+i+1,n+i],[n+i,n+i+1,3*n+i],[n+i+1,3*n+i+1,3*n+i]])
        inside.extend([[2*n+i,3*n+i,2*n+i+1],[2*n+i+1,3*n+i,3*n+i+1],[i,2*n+i,i+1],[i+1,2*n+i,2*n+i+1]])
    outside.extend([[0,n,2*n],[n,3*n,2*n],[n-1,3*n-1,2*n-1],[2*n-1,3*n-1,4*n-1]])
    meshes=[];uvs=[];textures=[]
    for label,faces,paint in [('outside',outside,cfg['spandrel_paint_rgb']),('inside_and_soffit',inside,cfg['spandrel_interior_rgb'])]:
        mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(vertices);mesh.triangles=o3d.utility.Vector3iVector(np.asarray(faces));mesh.compute_vertex_normals();path=dest/('spandrel_'+label+'.png');Image.new('RGB',(16,16),tuple(paint)).save(path);meshes.append(mesh);uvs.append(np.zeros((len(faces),3,2)));textures.append(path)
    return meshes,uvs,textures,{'triangles':sum(len(m.triangles) for m in meshes),'area_m2':float(sum(m.get_surface_area() for m in meshes)),'lower_joint_m':bottom_z,'opaque_cap_min_m':float(upper.min()),'opaque_cap_max_m':float(upper.max()),'upper_floor_contact_m':model.upper,'front_profile':front.tolist(),'nominal_backing_m':cfg['spandrel_nominal_backing_m'],'scope':'Closed presentation spandrel plus parapet, inferred backing thickness. Original photographed opaque boundary bounds the top; glass is not filled.'}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/architectural_connections.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];source=json.loads((base/'manifest.json').read_text());a.output.mkdir(parents=True,exist_ok=False)
    import open3d as o3d
    model=LevelPresentation(ROOT,cfg);pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());raw_atrium=np.load(ROOT/cfg['atrium_mask']);atrium={k:raw_atrium[k] for k in raw_atrium.files};atrium['core']=ndimage.binary_erosion(atrium['mask'],iterations=3)
    np.savez_compressed(a.output/'reviewed_atrium.npz',**atrium)
    inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),a.config,ROOT/'src/presentation_geometry.py',ROOT/'src/append_plane_glb.py',ROOT/'src/clean_structural_fragments.py',ROOT/'src/photo_semantic_surfaces.py',ROOT/'src/reconcile_multilevel_surfaces.py',mp,base/'manifest.json',ROOT/cfg['atrium_mask'],*[ROOT/v for v in cfg['height_models'].values()]]}
    frames={};region_fit={};groups={};parapet_points=[]
    for region in cfg['regions']:
        name,node=region['session'],region['node'];poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);db=Path(selected['database_integrity'][name]['database']);inputs[str(db)]=sha256_file(db);inputs[str((mp.parent/selected['sessions'][name]['trajectory']).resolve())]=sha256_file(mp.parent/selected['sessions'][name]['trajectory'])
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
            profile=dict(selected['sessions'][name]['preprocessing']);profile['max_depth']=cfg['ceiling_depth_limit_m'];f=decode_frame(con,node,profile,poses[node])
        frames[region['id']]=f;points=polygon_points(f,region['polygon'])
        if region['kind']=='wall':parapet_points.append(points);continue
        plane,fit=ceiling_fit(points,cfg['maximum_display_ceiling_tilt_deg'])
        if plane is None and region.get('neighbour_depth_nodes'):
            extras=[]
            with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
                for n in region['neighbour_depth_nodes']:
                    if n not in poses:continue
                    ff=decode_frame(con,n,profile,poses[n]);q=unproject(ff['depth'],ff['k_depth']);from optimize_session import apply
                    q=apply(q[ff['valid']],ff['world_optical']);uv,depth=project(q,f['k_rgb'],f['world_optical']);good=inside_polygon(uv,region['polygon'])&(q[:,2]>f['world_optical'][2,3]+.5)&(depth>.25)&(depth<8.);extras.append(q[good])
            if extras:
                plane,fit=ceiling_fit(np.concatenate([points,*extras]),cfg['maximum_display_ceiling_tilt_deg']);fit['neighbour_original_depth_nodes']=region['neighbour_depth_nodes']
        region_fit[region['id']]={'region':region,'plane':plane,'fit':fit}
        print(region['id']+' / confirmed plane '+str(fit['accepted']),flush=True)
    primary=region_fit[cfg['spandrel_source_region']]
    if primary['plane'] is None:raise ValueError('Original soffit has no physical ceiling support; cannot build the connection')
    source_h=float(primary['plane'][0][2]);display_h=float(model.warp(primary['plane'][0][None],'lobby')[0,2]);sessions={};cleanup={};new_semantics=[];semantics=json.loads((base/'semantic_objects.json').read_text())
    for name,spec in source['sessions'].items():
        print('Unified architectural presentation: '+name,flush=True);dest=a.output/name;dest.mkdir();mesh=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));mesh.compute_triangle_normals();xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32);centres=xyz.mean(1);keep=np.ones(len(xyz),bool)
        # Supersede the earlier isolated fascia patches with one shared surface.
        if name=='lobby':
            cursor=spec['original_source_faces']
            for region in spec.get('upper_wall_review',[]):
                count=region.get('added_triangles',0) if region.get('accepted') else 0
                if region['id'] in ['restroom_interfloor_fascia','restroom_interfloor_left_panel']:
                    pp=np.concatenate(parapet_points);xmin,xmax=np.percentile(pp[:,0],[1,99]);overlap=(xyz[cursor:cursor+count,:,0].max(1)>xmin-.08)&(xyz[cursor:cursor+count,:,0].min(1)<xmax+.08);keep[cursor:cursor+count]&=~overlap
                cursor+=count
        floor_trim=np.zeros(len(keep),bool)
        if name in ['floor2','bridge_12_sub']:
            front=exterior_profile(np.concatenate(parapet_points));front_y=np.interp(centres[:,0],front[:,0],front[:,1]);outside=(centres[:,0]>=front[0,0])&(centres[:,0]<=front[-1,0])&(centres[:,1]>front_y-cfg['spandrel_nominal_backing_m']+.02)&(centres[:,1]<front_y+1.)
            cursor=spec['retained_previous_prefix_faces']
            for plane in spec['flat_sheets']:
                if not plane.get('accepted'):continue
                count=plane['triangles']
                if plane['kind']=='floor' and '2f' in plane['id']:floor_trim[cursor:cursor+count]=outside[cursor:cursor+count]
                cursor+=count
            keep&=~floor_trim
        # Remove only ceiling fragments inside a known photographed ceiling
        # chart. Doors/windows/stairs/vertical source structure are not filtered.
        for record in region_fit.values():
            region=record['region']
            if region['session']!=name or record['plane'] is None:continue
            f=frames[region['id']];uv,z=project(centres,f['k_rgb'],f['world_optical']);inside=inside_polygon(uv,region['polygon']);near=abs(centres[:,2]-record['plane'][0][2])<.55
            keep&=~(inside&(z>.25)&(z<8.)&near&(abs(np.asarray(mesh.triangle_normals)[:,2])>.80))
        np.save(dest/'retained_source_faces.npy',keep);filter_glb(base/name/'clean_textured.glb',dest/'retained.glb',keep)
        mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();before=np.asarray(mesh.vertices).astype(np.float32).copy();after=model.warp(before,name);mesh.vertices=o3d.utility.Vector3dVector(after);mesh.compute_vertex_normals();warp_glb(dest/'retained.glb',dest/'level_normalized.glb',lambda p:model.warp(p,name))
        retained_count=len(mesh.triangles);meshes=[];uvs=[];textures=[];reports=[]
        if name=='lobby':
            group=[region_fit[k] for k in cfg['soffit_group'] if region_fit[k]['plane'] is not None];patch,uv,texture,report=ceiling_union(o3d,group,frames,source_h,display_h,model,dest,atrium);mesh+=patch;meshes.append(patch);uvs.append(uv);textures.append(texture);reports.append(report)
        for key,record in region_fit.items():
            region=record['region']
            if region['session']!=name:continue
            if key in cfg['soffit_group']:continue
            if record['plane'] is None:reports.append({'id':key,'accepted':False,'fit':record['fit']});continue
            h=source_h if key in cfg['soffit_group'] else float(record['plane'][0][2]);target=display_h if key in cfg['soffit_group'] else float(model.warp(record['plane'][0][None],name)[0,2])
            try:
                patch,uv,texture,report=photo_ceiling_mesh(o3d,frames[key],region,h,target,model,dest,atrium)
                if not len(patch.triangles):raise ValueError('Ceiling region is entirely in reviewed open atrium')
                mesh+=patch;meshes.append(patch);uvs.append(uv);textures.append(texture);reports.append({'id':key,'accepted':True,'kind':'photographed_ceiling','fit':record['fit'],**report})
            except ValueError as error:reports.append({'id':key,'accepted':False,'reason':str(error),'fit':record['fit']})
        if name=='lobby':
            bands,uv,texture,report=make_spandrel(o3d,np.concatenate(parapet_points),display_h,model,dest,cfg)
            for band in bands:mesh+=band
            meshes+=bands;uvs+=uv;textures+=texture;reports.append({'id':'continuous_interfloor_spandrel','accepted':True,'kind':'opaque_spandrel',**report})
        mesh.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),mesh);append_planes(dest/'level_normalized.glb',dest/'clean_textured.glb',meshes,uvs,textures)
        current=[o for o in semantics['objects'] if o['session']==name]
        for obj in current:
            for file in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                if file and (base/name/file).exists():shutil.copyfile(base/name/file,dest/file)
            if obj.get('geometry',{}).get('world_centre') is not None:obj['geometry']['world_centre']=model.warp(np.array(obj['geometry']['world_centre'])[None],name)[0].astype(float).tolist()
            obj['presentation_geometry_only']=True
        new_semantics+=current
        sessions[name]={'session':name,'architectural_regions':reports,'semantic_objects':current,'retained_source_triangles':retained_count,'superseded_source_faces':int((~keep).sum()),'trimmed_inferred_upper_floor_overhang_triangles':int(floor_trim.sum()),'source_mesh':str((base/name/'clean_structural.ply').resolve()),'source_texture':str((base/name/'clean_textured.glb').resolve()),'retained_xy_exact':bool(np.array_equal(before[:,:2],after[:,:2])),'maximum_retained_vertex_z_change_m':float(abs(after[:,2]-before[:,2]).max(initial=0)),'pose_changes':0,'bounds_min_m':mesh.get_min_bound().tolist(),'bounds_max_m':mesh.get_max_bound().tolist()}
        cleanup[name]={'before_faces':len(keep),'after_faces':len(mesh.triangles),'removed_faces':int((~keep).sum()),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for path in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(path.resolve())]=sha256_file(path)
        (dest/'summary.json').write_text(json.dumps(sessions[name],indent=2)+'\n');del mesh,xyz;gc.collect()
    manifest={'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**source['inputs_sha256'],**inputs},'selected_manifest_sha256':pointer['manifest_sha256'],'architectural_presentation':True,'inferred_ceilings_disabled':True,'global_ceiling_generation':False,'presentation_config':cfg,'presentation_pose_changes':0,'display_geometry_z_regularized':True,'source_selected_map_replaced':False,'navigation_ready':False,'base_display_manifest':str((base/'manifest.json').resolve()),'base_display_policy':{'sessions':{n:{'method':'structural'} for n in sessions}},'scope':'Unified architectural presentation, shared horizontal levels, photographed ceiling zones and modelled opaque inter-floor spandrel. NOT pose optimization or surveyed/navigation geometry.'}
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json')},indent=2)+'\n');(a.output/'semantic_objects.json').write_text(json.dumps({'frame':'presentation_z_normalized','objects':new_semantics},indent=2,ensure_ascii=False)+'\n')
    for path,h in manifest['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed: '+path)
    print('Unified presentation completed: '+str(a.output),flush=True)


if __name__=='__main__':main()
