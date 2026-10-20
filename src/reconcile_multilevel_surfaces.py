#!/usr/bin/env python3
"""Locally anchored planar sheets, shared landing heights and an open atrium."""
import argparse,gc,json,shutil,sqlite3
from collections import defaultdict
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
from flatten_horizontal_surfaces import view_samples,spaced_nodes,protected_points
from optimize_session import sha256_file,utc_now
from reconstruct_continuous_planes import solve_log_gains
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from surface_model_utils import decode_frame
from texture_boundary_leveling import level_boundaries

ROOT=Path(__file__).resolve().parents[1]


def limit_lattice_slopes(heights,spacing,max_slope_deg):
    """Project both coarse-triangle gradients into a bounded slope cone.

    Local fitted slopes alone do not bound the gradients between independently
    fitted vertices. Disjoint tile colors make each projection deterministic.
    This is a display regularizer, not a new observation of the floor.
    """
    z=np.asarray(heights,dtype=float).copy();h,w=z.shape;radius=spacing*np.tan(np.deg2rad(max_slope_deg))
    for iteration in range(1000):
        worst=0.
        for cy in range(2):
            for cx in range(2):
                yy,xx=np.meshgrid(np.arange(cy,h-1,2),np.arange(cx,w-1,2),indexing='ij');yy=yy.ravel();xx=xx.ravel()
                for second in [False,True]:
                    coords=[(yy,xx),(yy,xx+1),(yy+1,xx)] if not second else [(yy+1,xx+1),(yy+1,xx),(yy,xx+1)]
                    a,b,c=[z[pos] for pos in coords];gx=b-a;gy=c-a;length=np.hypot(gx,gy);worst=max(worst,float(length.max(initial=0)))
                    active=length>radius
                    if not active.any():continue
                    plus=(gx+gy)/np.sqrt(2);minus=(gx-gy)/np.sqrt(2);lo=np.zeros(len(a));hi=np.maximum(1.,length/radius)
                    for _ in range(28):
                        lam=(lo+hi)/2;outside=np.hypot(plus/(1+3*lam),minus/(1+lam))>radius
                        lo=np.where(outside,lam,lo);hi=np.where(outside,hi,lam)
                    lam=np.where(active,hi,0);gp=plus/(1+3*lam);gm=minus/(1+lam);nx=(gp+gm)/np.sqrt(2);ny=(gp-gm)/np.sqrt(2)
                    for pos,value in zip(coords,[a+lam*(nx+ny),b-lam*nx,c-lam*ny]):z[pos]=value
        if worst<=radius+1e-8:return z
    raise ValueError('Shared lattice slope projection did not converge')


def median_raster_points(points,cell=.2):
    ij=np.floor(points[:,:2]/cell).astype(int);keys=ij[:,0].astype(np.int64)*100000+ij[:,1];order=np.argsort(keys)
    _,start,count=np.unique(keys[order],return_index=True,return_counts=True);out=[]
    for s,n in zip(start,count):out.append(np.median(points[order[s:s+n]],axis=0))
    return np.asarray(out)


def anchor_lattice(points,spacing=2.,radius=2.3,huber=.06,max_slope_deg=4.):
    low=np.floor(points[:,:2].min(0)/spacing)*spacing-spacing;high=np.ceil(points[:,:2].max(0)/spacing)*spacing+spacing
    w,h=np.rint((high-low)/spacing).astype(int)+1;yy,xx=np.indices((h,w));query=np.c_[low[0]+xx.ravel()*spacing,low[1]+yy.ravel()*spacing]
    tree=cKDTree(points[:,:2]);heights=np.zeros(len(query));counts=[];cap=np.tan(np.deg2rad(max_slope_deg))
    for i,q in enumerate(query):
        ids=tree.query_ball_point(q,radius)
        if len(ids)<8:ids=np.atleast_1d(tree.query(q,k=min(32,len(points)))[1]).tolist()
        p=points[ids];xy=p[:,:2]-q;a=np.c_[xy,np.ones(len(xy))];base=1/(1+(np.linalg.norm(xy,axis=1)/1.2)**2);weights=base.copy()
        for _ in range(6):
            root=np.sqrt(weights);coef=np.linalg.lstsq(a*root[:,None],p[:,2]*root,rcond=None)[0];residual=a@coef-p[:,2]
            weights=base*np.minimum(1.,huber/np.maximum(abs(residual),1e-9))
        length=np.linalg.norm(coef[:2])
        if length>cap:
            coef[:2]*=cap/length;coef[2]=np.median(p[:,2]-xy@coef[:2])
        # Unsupported edge extrapolation uses the nearby source range rather
        # than allowing a fitted plane to invent extreme heights.
        heights[i]=np.clip(coef[2],np.percentile(p[:,2],10)-.05,np.percentile(p[:,2],90)+.05);counts.append(len(ids))
    initial=heights.reshape(h,w);bounded=limit_lattice_slopes(initial,spacing,max_slope_deg)
    return {'low':low,'spacing':spacing,'heights':bounded,'support_counts':np.asarray(counts).reshape(h,w),
        'maximum_slope_projection_adjustment_m':float(abs(bounded-initial).max())}


def sample_lattice(model,xy):
    q=(np.asarray(xy)-model['low'])/model['spacing'];h,w=model['heights'].shape
    x=np.clip(q[:,0],0,w-1-1e-9);y=np.clip(q[:,1],0,h-1-1e-9);ix=np.floor(x).astype(int);iy=np.floor(y).astype(int);fx=x-ix;fy=y-iy;z=model['heights']
    a=z[iy,ix];b=z[iy,ix+1];c=z[iy+1,ix];d=z[iy+1,ix+1]
    # Two affine triangles per coarse tile: shared vertices make neighboring
    # local planar patches continuous without forcing an entire floor to one Z.
    return np.where(fx+fy<=1,a+(b-a)*fx+(c-a)*fy,d+(c-d)*(1-fx)+(b-d)*(1-fy))


def remap_mask(mask,low,cell,other_low,other_cell,shape):
    yy,xx=np.indices(shape);xy=np.c_[other_low[0]+(xx.ravel()+.5)*other_cell,other_low[1]+(yy.ravel()+.5)*other_cell]
    ij=np.floor((xy-low)/cell).astype(int);x,y=ij.T;h,w=mask.shape;ii=np.flatnonzero((x>=0)&(x<w)&(y>=0)&(y<h));out=np.zeros(len(xy),bool);out[ii]=mask[y[ii],x[ii]]
    return out.reshape(shape)


def build_anchored_sheet(o3d,mask,low,cell,kind,lattice):
    h,w=mask.shape;yy,xx=np.indices((h+1,w+1));xy=np.c_[low[0]+xx.ravel()*cell,low[1]+yy.ravel()*cell];v=np.c_[xy,sample_lattice(lattice,xy)]
    ids=np.arange((h+1)*(w+1)).reshape(h+1,w+1);aa=ids[:-1,:-1][mask];b=aa+1;c=aa+w+1;d=c+1;tri=np.r_[np.c_[aa,b,c],np.c_[b,d,c]]
    if kind=='ceiling':tri=tri[:,[0,2,1]]
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(v);mesh.triangles=o3d.utility.Vector3iVector(tri);mesh.compute_vertex_normals()
    uv=np.c_[xx.ravel()/w,yy.ravel()/h];return mesh,uv[tri]


def variable_height_stair_guard(shape,low,cell,lattice,guard):
    yy,xx=np.indices(shape);xy=np.c_[low[0]+(xx.ravel()+.5)*cell,low[1]+(yy.ravel()+.5)*cell];blocked=np.zeros(len(xy),bool)
    for offset in [(0,0),(-.5,-.5),(.5,-.5),(-.5,.5),(.5,.5)]:
        q=xy+np.asarray(offset)*cell;points=np.c_[q,sample_lattice(lattice,q)]
        blocked|=protected_points(points,np.asarray(guard['centres']),np.asarray(guard['half'])+[cell,cell,.02])
    return blocked.reshape(shape)


def texture_sheet(mask,low,cell,kind,lattice,frames,points,colours,dest,plane_id,base_cfg):
    h,w=mask.shape;tw=max(2,min(1400,int(w*cell/.025)));th=max(2,min(1400,int(h*cell/.025)));cw=min(280,tw);ch=min(280,th)
    def world(width,height):
        yy,xx=np.indices((height,width));xy=np.c_[low[0]+(xx.ravel()+.5)*w*cell/width,low[1]+(yy.ravel()+.5)*h*cell/height]
        return np.c_[xy,sample_lattice(lattice,xy)]
    coarse=world(cw,ch);median_z=np.median(coarse[:,2]);candidates=[]
    for node,f in frames.items():
        dz=f['world_optical'][2,3]-median_z
        if (kind=='floor' and not .6<dz<2.8) or (kind=='ceiling' and not -3.5<dz<-.25):continue
        rgb,score=view_samples(f,coarse,coarse[:,2],kind)
        if np.count_nonzero(score)>15:candidates.append((node,rgb.reshape(ch,cw,3),score.reshape(ch,cw)))
    candidates.sort(key=lambda x:-x[2].sum());candidates=candidates[:36];full=world(tw,th);nearest=cKDTree(points[:,:2]).query(full[:,:2],workers=2)[1]
    fallback=srgb_to_linear(colours[nearest]).reshape(th,tw,3);fallback=ndimage.gaussian_filter(fallback,(4.8,4.8,0));result=fallback.copy();covered=np.zeros((th,tw),bool);labels=np.full((th,tw),65535,np.uint16)
    if candidates:
        gains=solve_log_gains([srgb_to_linear(x[1]) for x in candidates],[x[2] for x in candidates]);weights=np.asarray([x[2] for x in candidates]);small=coherent_labels(weights,step=2,potts=.5)
        chosen=cv2.resize(small.astype(np.float32),(tw,th),interpolation=cv2.INTER_NEAREST).astype(int).ravel()
        for index,(node,_,_) in enumerate(candidates):
            ii=np.flatnonzero(chosen==index);rgb,score=view_samples(frames[node],full[ii],full[ii,2],kind);valid=score>0;jj=ii[valid]
            result.reshape(-1,3)[jj]=srgb_to_linear(rgb[valid])*gains[index];covered.ravel()[jj]=True;labels.ravel()[jj]=index
    pixel_m=max(w*cell/tw,h*cell/th);feather=np.clip(ndimage.distance_transform_edt(covered)*pixel_m/.12,0,1)
    result=result*feather[:,:,None]+fallback*(1-feather[:,:,None]);region=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0
    result,lighting=flatten_photo_lighting(result,region,pixel_m);rgb=np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5);rgb,boundaries=level_boundaries(rgb,labels,covered)
    path=dest/(plane_id+'_texture.png');Image.fromarray(rgb).save(path);Image.fromarray(np.uint8(np.where(region,np.where(covered,255,80),0))).save(dest/(plane_id+'_photo_coverage.png'))
    return path,{'original_photo_nodes':[x[0] for x in candidates],'photo_fraction_on_sheet':float(covered[region].mean()),'texture_shape':[tw,th],
        'boundary_leveling':boundaries,'bounded_chart_lighting':lighting,'fallback':'existing_horizontal_RGB_only_no_generated_photography'}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/multilevel_surface_topology.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];source=json.loads((base/'manifest.json').read_text());grid=np.load(ROOT/cfg['topology_audit'])
    labels,count=ndimage.label(grid['possible_double_height']);sizes=np.bincount(labels.ravel());sizes[0]=0;void=labels==sizes.argmax()
    a.output.mkdir(parents=True,exist_ok=False);np.savez_compressed(a.output/'reviewed_atrium.npz',mask=void,low=grid['low'],cell=grid['cell'])
    import open3d as o3d
    layers={};group_points=defaultdict(list)
    for name,s in source['sessions'].items():
        old=o3d.io.read_triangle_mesh(s['source_mesh']);old.compute_triangle_normals();xyz=np.asarray(old.vertices)[np.asarray(old.triangles)];centres=xyz.mean(1);normals=np.asarray(old.triangle_normals)
        protected=np.load(base/name/'protected_source_faces.npy');colours=np.asarray(old.vertex_colors);colours=colours[np.asarray(old.triangles)].mean(1) if len(colours) else np.full((len(xyz),3),.5)
        for plane in s['flat_sheets']:
            if not plane.get('accepted'):continue
            lo,hi=plane['height_band_m'];ids=np.flatnonzero((centres[:,2]>=lo)&(centres[:,2]<hi)&(abs(normals[:,2])>.85)&~protected)
            layers[plane['id']]={'points':centres[ids].copy(),'colours':colours[ids].copy(),'plane':plane,'session':name}
            group_points[cfg['layer_groups'][plane['id']]].append(median_raster_points(centres[ids],cfg['source_raster_cell_m']))
        del old,xyz;gc.collect()
    lattices={}
    for group,points in group_points.items():
        merged=median_raster_points(np.concatenate(points),cfg['source_raster_cell_m']);model=anchor_lattice(merged,cfg['anchor_spacing_m'],cfg['anchor_neighbour_radius_m'],cfg['local_plane_huber_m'],cfg['local_plane_slope_limit_deg'])
        lattices[group]=model;np.savez_compressed(a.output/(group+'_anchors.npz'),**model)
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    run={**source,'sessions':{},'inputs_sha256':dict(source['inputs_sha256']),'multilevel_topology_repair':True,
        'topology_config':cfg,'previous_flat_manifest':str(base/'manifest.json'),'base_display_manifest':str(base/'manifest.json'),
        'base_display_policy':{'sessions':{n:{'method':'structural'} for n in source['sessions']}},'pose_changes':0}
    for path in [Path(__file__),a.config,base/'manifest.json',ROOT/cfg['topology_audit']]:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    semantics=json.loads((base/'semantic_objects.json').read_text());new_semantics=[]
    for name,s in source['sessions'].items():
        print('Reconcile local heights and atrium: '+name,flush=True);dest=a.output/name;dest.mkdir();before=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'))
        planes=[v for v in s['flat_sheets'] if v.get('accepted')];prefix=len(before.triangles)-sum(v['triangles'] for v in planes);keep=np.zeros(len(before.triangles),bool);keep[:prefix]=True
        np.save(dest/'retained_previous_faces.npy',keep);filter_glb(base/name/'clean_textured.glb',dest/'retained.glb',keep)
        before.remove_triangles_by_mask(~keep);before.remove_unreferenced_vertices();combined=o3d.geometry.TriangleMesh(before);meshes=[];uvs=[];textures=[];reports=[]
        poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);nodes=spaced_nodes(poses);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        frames={node:decode_frame(con,node,selected['sessions'][name]['preprocessing'],poses[node]) for node in nodes} if planes else {};con.close()
        for plane in planes:
            identity=plane['id'];pack=np.load(base/name/plane['mask_file']);data={k:pack[k].copy() for k in pack.files};old_mask=data['mask'].copy();mask=old_mask.copy();cell=float(data['cell']);low=data['low']
            atrium=remap_mask(void,grid['low'],float(grid['cell']),low,cell,mask.shape) if identity in cfg['void_excluded_layers'] else np.zeros_like(mask)
            mask&=~atrium
            # Do not infer a lower ceiling from the footprint of the floor.
            # Retain only bounded completion close to actual ceiling support.
            if identity in cfg['low_ceiling_layers']:
                mask&=ndimage.distance_transform_edt(~data['observed'])*cell<=cfg['low_ceiling_maximum_gap_fill_m']
            group=cfg['layer_groups'][identity];lattice=lattices[group]
            stair=variable_height_stair_guard(mask.shape,low,cell,lattice,s['flat_stair_guard']);mask&=~stair
            if not mask.any():reports.append({**plane,'accepted':False,'reason':'No support outside reviewed atrium/stair guard'});continue
            mesh,uv=build_anchored_sheet(o3d,mask,low,cell,plane['kind'],lattice)
            texture,photo=texture_sheet(mask,low,cell,plane['kind'],lattice,frames,layers[identity]['points'],layers[identity]['colours'],dest,identity,source['flat_config'])
            data.update(mask=mask,atrium_guard=atrium,stair_guard=data['stair_guard']|stair,forbidden=data['forbidden']|atrium|stair);np.savez_compressed(dest/plane['mask_file'],**data)
            vertices=np.asarray(mesh.vertices);used=np.unique(np.asarray(mesh.triangles));heights=vertices[used,2];mesh.compute_triangle_normals();slopes=np.rad2deg(np.arccos(np.clip(abs(np.asarray(mesh.triangle_normals)[:,2]),0,1)))
            samples=layers[identity]['points'];residual=sample_lattice(lattice,samples[:,:2])-samples[:,2]
            reports.append({**plane,**photo,'accepted':True,'height_m':float(np.median(heights)),'height_span_m':float(np.ptp(heights)),
                'maximum_abs_height_deviation_m':None,'height_model':'shared continuous piecewise-affine local planes, not one global Z',
                'anchor_group':group,'anchor_file':group+'_anchors.npz','triangles':len(mesh.triangles),'area_xy_m2':float(mask.sum()*cell*cell),
                'maximum_lattice_slope_projection_adjustment_m':float(lattice['maximum_slope_projection_adjustment_m']),
                'model_inferred_area_m2':float((mask&~data['observed']).sum()*cell*cell),
                'atrium_removed_area_m2':float((old_mask&atrium).sum()*cell*cell),'total_removed_previous_sheet_area_m2':float((old_mask&~mask).sum()*cell*cell),
                'source_anchor_residual_median_m':float(np.median(abs(residual))),'source_anchor_residual_p90_m':float(np.percentile(abs(residual),90)),
                'actual_local_triangle_slope_p90_deg':float(np.percentile(slopes,90))})
            meshes.append(mesh);uvs.append(uv);textures.append(texture);combined+=mesh
            print(identity+' / atrium cut '+str(round((old_mask&atrium).sum()*cell*cell,2))+'m2 / local anchor P90 '+str(round(np.percentile(abs(residual),90)*100,1))+'cm',flush=True)
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),combined);append_planes(dest/'retained.glb',dest/'clean_textured.glb',meshes,uvs,textures)
        current_semantics=[o for o in semantics['objects'] if o['session']==name]
        for obj in current_semantics:
            for file in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                if file and (base/name/file).exists():shutil.copyfile(base/name/file,dest/file)
        new_semantics+=current_semantics
        s={**s,'flat_sheets':reports,'semantic_objects':current_semantics,'source_mesh':str((base/name/'clean_structural.ply').resolve()),
            'source_texture':str((base/name/'clean_textured.glb').resolve()),'retained_previous_prefix_faces':prefix,'reviewed_atrium_model_area_m2':float(void.sum()*float(grid['cell'])**2)}
        run['sessions'][name]=s;(dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
        run.update(created_utc=utc_now(),scope='Locally anchored display sheets and reviewed open-atrium topology; original poses/vertical structures retained; not surveyed accuracy.')
        (a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n');(a.output/'semantic_objects.json').write_text(json.dumps({'frame':'selected_map','objects':new_semantics},indent=2,ensure_ascii=False)+'\n')
        del before,combined,frames;gc.collect()
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)


if __name__=='__main__':main()
