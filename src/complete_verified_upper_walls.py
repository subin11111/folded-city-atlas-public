#!/usr/bin/env python3
"""Photo-bounded upper walls first; small measured ceilings only at verified joints.

The selected poses and every existing triangle remain unchanged. The curtain
model follows an observed horizontal range profile rather than flattening a
curved balcony into a plane. Unknown space outside reviewed photographs stays open.
"""
import argparse,gc,json,shutil,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy.interpolate import PchipInterpolator
from scipy.spatial import Delaunay,cKDTree
from append_plane_glb import append_planes
from export_rtabmap_db import load_pose_csv
from optimize_session import apply,sha256_file,utc_now
from photo_semantic_surfaces import lift_pixels
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame,project,unproject

ROOT=Path(__file__).resolve().parents[1]


def inside_polygon(pixels,polygon):
    poly=np.asarray(polygon,np.float32)
    return np.array([cv2.pointPolygonTest(poly,tuple(p),False)>=0 for p in np.asarray(pixels,np.float32)],dtype=bool)


def robust_plane(points,horizontal=False):
    """Deterministic 3D RANSAC + trimmed TLS, not a height-mode prior."""
    rng=np.random.default_rng(901);sample=points[rng.choice(len(points),min(1600,len(points)),replace=False)];best=None
    for _ in range(320):
        tri=sample[rng.choice(len(sample),3,replace=False)];n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length
        if (horizontal and abs(n[2])<.985) or (not horizontal and abs(n[2])>.2):continue
        residual=abs((sample-tri[0])@n);count=int(np.sum(residual<.035))
        if best is None or count>best[0]:best=(count,tri[0],n)
    if best is None:return None,{'accepted':False,'reason':'No coherent plane with the required orientation'}
    selected=abs((points-best[1])@best[2])<.05
    if selected.sum()<30:return None,{'accepted':False,'reason':'Insufficient coherent support'}
    for _ in range(4):
        p=points[selected];centre=np.median(p,axis=0);_,_,vt=np.linalg.svd(p-centre,full_matrices=False);normal=vt[-1]
        selected=abs((points-centre)@normal)<.05
    residual=abs((points[selected]-centre)@normal);fraction=float(selected.mean())
    accepted=fraction>=.6 and np.percentile(residual,90)<=.06 and (abs(normal[2])>.985 if horizontal else abs(normal[2])<.2)
    return ((centre,normal) if accepted else None),{'accepted':bool(accepted),'support_points':int(selected.sum()),'total_points':len(points),
        'support_fraction':fraction,'plane_fit_p90_m':float(np.percentile(residual,90)),'centre':centre.tolist(),'normal':normal.tolist()}


def ray_directions(pixels,k,pose):
    return np.c_[(pixels[:,0]-k[0,2])/k[0,0],(pixels[:,1]-k[1,2])/k[1,1],np.ones(len(pixels))]@pose[:3,:3].T


def make_range_profile(points,camera,bin_deg=1.):
    v=points[:,:2]-camera[:2];theta=np.arctan2(v[:,1],v[:,0]);origin=np.arctan2(np.sin(theta).mean(),np.cos(theta).mean());theta=np.angle(np.exp(1j*(theta-origin)));radius=np.linalg.norm(v,axis=1)
    step=np.deg2rad(bin_deg);bins=np.floor(theta/step).astype(int);angles=[];ranges=[];counts=[]
    for index in np.unique(bins):
        q=radius[bins==index]
        if len(q)<3 or np.percentile(q,90)-np.percentile(q,10)>.30:continue
        angles.append(float(np.median(theta[bins==index])));ranges.append(float(np.median(q)));counts.append(len(q))
    if len(angles)<5:raise ValueError('Too few observed angular bins for curved wall')
    return {'origin':origin,'angles':np.asarray(angles),'ranges':np.asarray(ranges),'counts':np.asarray(counts)}


def profile_lift(pixels,frame,model,extrapolation_deg=2.):
    camera=frame['world_optical'][:3,3];directions=ray_directions(pixels,frame['k_rgb'],frame['world_optical']);theta=np.angle(np.exp(1j*(np.arctan2(directions[:,1],directions[:,0])-model['origin'])))
    low,high=model['angles'][0],model['angles'][-1];valid=(theta>=low-np.deg2rad(extrapolation_deg))&(theta<=high+np.deg2rad(extrapolation_deg))
    intervals=np.clip(np.searchsorted(model['angles'],theta)-1,0,len(model['angles'])-2)
    valid&=(model['angles'][intervals+1]-model['angles'][intervals])<=np.deg2rad(float(model.get('maximum_internal_gap_deg',4.)))
    range_=PchipInterpolator(model['angles'],model['ranges'],extrapolate=False)(np.clip(theta,low,high));t=range_/np.maximum(np.linalg.norm(directions[:,:2],axis=1),1e-8)
    xyz=camera+directions*t[:,None]
    if 'maximum_opaque_z' in model:valid&=xyz[:,2]<=float(model['maximum_opaque_z'])
    return xyz,t,valid


def measured_joint_line(wall_plane,ceiling_plane,frame):
    nw,nc=wall_plane[1],ceiling_plane[1];direction=np.cross(nw,nc);direction/=np.linalg.norm(direction)
    point=np.linalg.lstsq(np.array([nw,nc]),[nw@wall_plane[0],nc@ceiling_plane[0]],rcond=None)[0]
    uv,_=project(np.array([point-direction,point+direction]),frame['k_rgb'],frame['world_optical'])
    return np.cross(np.r_[uv[0],1.],np.r_[uv[1],1.])


def pixels_on_joint(line,pixels):
    pixels=np.asarray(pixels,float).copy();pixels[:,1]=-(line[0]*pixels[:,0]+line[2])/line[1];return pixels


def make_photo_patch(o3d,frame,region,lift,dest):
    polygon=np.asarray(region['polygon'],float);lo=np.maximum(np.floor(polygon.min(0)).astype(int),0);hi=np.minimum(np.ceil(polygon.max(0)).astype(int),[719,959])
    xs=np.linspace(lo[0],hi[0],max(2,int((hi[0]-lo[0])/8))+1);ys=np.linspace(lo[1],hi[1],max(2,int((hi[1]-lo[1])/8))+1);xx,yy=np.meshgrid(xs,ys);pixels=np.c_[xx.ravel(),yy.ravel()]
    pixels=np.unique(np.r_[polygon,pixels[inside_polygon(pixels,polygon)]],axis=0);xyz,t,valid=lift(pixels)
    valid&=np.isfinite(xyz).all(1)&(t>.25)&(t<12);pixels=pixels[valid];xyz=xyz[valid]
    if len(pixels)<8:raise ValueError('Insufficient supported polygon rays')
    triangles=Delaunay(pixels).simplices;centres=pixels[triangles].mean(1);triangles=triangles[inside_polygon(centres,polygon)]
    # Long triangles may bridge a profile's unsupported angular sector.
    centre_world,_,cv=lift(pixels[triangles].mean(1));length=np.linalg.norm(np.roll(xyz[triangles],1,axis=1)-xyz[triangles],axis=2).max(1)
    triangles=triangles[cv&(length<.55)]
    p=xyz[triangles];normal=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);back=np.sum(normal*(frame['world_optical'][:3,3]-p.mean(1)),axis=1)<0;triangles[back]=triangles[back][:,[0,2,1]]
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(xyz);mesh.triangles=o3d.utility.Vector3iVector(triangles);mesh.compute_vertex_normals()
    # A millimetric display offset avoids coplanar Z fighting; it is recorded
    # and does not move any pre-existing mesh coordinates.
    xyz=xyz+np.asarray(mesh.vertex_normals)*.002;mesh.vertices=o3d.utility.Vector3dVector(xyz)
    texture=dest/(region['id']+'_texture.png');Image.fromarray(frame['rgb'][lo[1]:hi[1]+1,lo[0]:hi[0]+1]).save(texture)
    uv=(pixels-lo)/np.maximum(hi-lo,1);return mesh,uv[triangles],texture


def missing_triangles(o3d,mesh,frame,scene,tolerance=.055):
    xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)];centre=xyz.mean(1);camera=frame['world_optical'][:3,3];v=centre-camera;distance=np.linalg.norm(v,axis=1)
    rays=np.c_[np.broadcast_to(camera,v.shape),v/np.maximum(distance[:,None],1e-8)].astype(np.float32);hit=scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
    # Only missing or background hits authorize a new opaque-wall fragment;
    # an observed foreground object is never deleted to expose the new plane.
    keep=(~np.isfinite(hit))|(hit>distance+tolerance)
    return keep,{'sampled_triangles':len(keep),'already_supported_or_foreground':int((~keep).sum()),'missing_or_background':int(keep.sum())}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/verified_upper_wall_completion.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];source=json.loads((base/'manifest.json').read_text())
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());a.output.mkdir(parents=True,exist_ok=False)
    import open3d as o3d
    run={**source,'sessions':{},'inputs_sha256':dict(source['inputs_sha256']),'verified_upper_wall_completion':True,'global_ceiling_generation':False,
        'base_display_manifest':str(base/'manifest.json'),'base_display_policy':{'sessions':{n:{'method':'structural'} for n in source['sessions']}},'pose_changes':0,
        'scope':'Photo-bounded opaque upper walls and small measured ceiling patches after verified wall-top junctions. Original faces/poses retained; no global heights/footprints.'}
    for path in [Path(__file__),a.config,base/'manifest.json',ROOT/'src/append_plane_glb.py',ROOT/'src/surface_model_utils.py']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    semantics=json.loads((base/'semantic_objects.json').read_text());cleanup={}
    for name,s in source['sessions'].items():
        print('Verified upper-wall stage: '+name,flush=True);dest=a.output/name;dest.mkdir();old=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));old.compute_triangle_normals();combined=o3d.geometry.TriangleMesh(old)
        scene=o3d.t.geometry.RaycastingScene();scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(old));meshes=[];uvs=[];textures=[];reports=[];wall_models={}
        regions=[r for r in cfg['wall_regions']+cfg['ceiling_regions'] if r['session']==name];poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory'])
        con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True);cache={}
        def frame(node,extended=False,high_confidence=False):
            key=(node,extended,high_confidence)
            if key not in cache:
                profile=dict(selected['sessions'][name]['preprocessing'])
                if extended:profile['max_depth']=cfg['fascia_support_max_depth_m']
                if high_confidence:profile['confidence']=cfg['fascia_single_view_extended_minimum_confidence']
                cache[key]=decode_frame(con,node,profile,poses[node])
            return cache[key]
        observed=o3d.io.read_triangle_mesh(str(ROOT/'outputs/student_union_preserved_display_v1'/name/'preserved_geometry.ply'));observed.compute_triangle_normals();xyz=np.asarray(observed.vertices)[np.asarray(observed.triangles)];centres=xyz.mean(1);normals=np.asarray(observed.triangle_normals)
        for original_region in regions:
            region={**original_region,'polygon':[list(v) for v in original_region['polygon']]}
            f=frame(region['node']);points=polygon_points(f,region['polygon']);is_ceiling=region in cfg['ceiling_regions'];report={'id':region['id'],'session':name,'node':region['node'],'kind':'verified_local_ceiling' if is_ceiling else 'upper_wall',
                'polygon':region['polygon'],'source_rgb_sha256':sha256_file(ROOT/'outputs/student_union_roadview_v1'/name/'photos'/f'{region["node"]:05d}.jpg'),'original_valid_depth_samples':len(points),'display_offset_m':.002}
            if region.get('depth_max_m'):
                profile={**selected['sessions'][name]['preprocessing'],'max_depth':region['depth_max_m']};extended=decode_frame(con,region['node'],profile,poses[region['node']]);points=polygon_points(extended,region['polygon']);report.update(region_depth_max_m=region['depth_max_m'],region_valid_depth_samples=len(points))
            try:
                if region.get('model')=='vertical_radial_profile':
                    uv,z=project(centres,f['k_rgb'],f['world_optical']);eligible=(z>.35)&(z<12)&(abs(normals[:,2])<.2)&(centres[:,2]>1.1)&(centres[:,2]<3.5)
                    ii=np.flatnonzero(eligible);ii=ii[inside_polygon(uv[ii],region['polygon'])];primary_high=polygon_points(frame(region['node'],True,True),region['polygon'])
                    support=[points,primary_high,centres[ii]];extended_points=polygon_points(frame(region['node'],True),region['polygon']);other_support=[]
                    # Independent source frames supply depth in dark/high pixels
                    # absent from this view. Selection is by the reviewed polygon.
                    for node in region.get('independent_review_nodes',[]):
                        other=frame(node,True);q=apply(unproject(other['depth'],other['k_depth']).reshape(-1,3),other['world_optical']);q=q[other['valid'].ravel()];uv,z=project(q,f['k_rgb'],f['world_optical']);ok=(z>.35)&(z<12)&(q[:,2]>1.1)&(q[:,2]<3.5);ii2=np.flatnonzero(ok);ii2=ii2[inside_polygon(uv[ii2],region['polygon'])];other_support.append(q[ii2])
                    other_points=np.concatenate(other_support);d1=cKDTree(other_points).query(extended_points,workers=2)[0];d2=cKDTree(extended_points).query(other_points,workers=2)[0]
                    support+=[extended_points[d1<cfg['fascia_cross_view_agreement_m']],other_points[d2<cfg['fascia_cross_view_agreement_m']]]
                    points=np.concatenate(support);profile=make_range_profile(points,f['world_optical'][:3,3],cfg['range_profile_bin_deg']);profile['maximum_internal_gap_deg']=cfg['maximum_profile_internal_gap_deg']
                    boundary=region['upper_opaque_boundary'];bn=boundary['session'];bps=load_pose_csv(mp.parent/selected['sessions'][bn]['trajectory']);bc=sqlite3.connect(f'file:{selected["database_integrity"][bn]["database"]}?mode=ro',uri=True)
                    bf=decode_frame(bc,boundary['node'],selected['sessions'][bn]['preprocessing'],bps[boundary['node']]);bc.close();bp=polygon_points(bf,boundary['polygon'])
                    if len(bp)<8:raise ValueError('Opaque parapet/glazing boundary is not established')
                    profile['maximum_opaque_z']=float(np.percentile(bp[:,2],95));np.savez_compressed(dest/(region['id']+'_profile.npz'),**profile)
                    wall_models[region['id']]={'profile':profile}
                    lift=lambda px:profile_lift(px,f,profile,cfg['maximum_profile_angular_extrapolation_deg'])
                    report.update(model='gravity-vertical curtain along cross-view-consistent measured range profile',profile_support_samples=len(points),profile_angular_bins=len(profile['angles']),profile_span_deg=float(np.rad2deg(np.ptp(profile['angles']))),
                        extended_depth_samples=len(extended_points),single_view_extended_high_confidence_samples=len(primary_high),single_view_extended_confidence_minimum=cfg['fascia_single_view_extended_minimum_confidence'],
                        cross_view_consistent_extended_samples=int((d1<cfg['fascia_cross_view_agreement_m']).sum()),maximum_profile_internal_gap_deg=cfg['maximum_profile_internal_gap_deg'],maximum_opaque_z_m=profile['maximum_opaque_z'],glazing_boundary_depth_samples=len(bp))
                else:
                    if len(points)<cfg['minimum_valid_depth_samples']:raise ValueError('Insufficient valid original depth')
                    plane,fit=robust_plane(points,horizontal=is_ceiling);report['fit']=fit
                    if plane is None:raise ValueError('Original depth does not establish a coherent physical plane')
                    centre,normal=plane
                    if not is_ceiling:
                        dependent=next((c for c in cfg['ceiling_regions'] if c['wall_region']==region['id']),None)
                        if dependent:
                            cp,cf=robust_plane(polygon_points(f,dependent['polygon']),horizontal=True)
                            if cp is not None:
                                line=measured_joint_line(plane,cp,f);manual=np.asarray(region['junction_pixels'],float);corrected=pixels_on_joint(line,manual);correction=abs(corrected[:,1]-manual[:,1])
                                if correction.max()>cfg['maximum_junction_pixel_correction_px']:raise ValueError('Measured joint is too far from the photographed wall/ceiling boundary')
                                original_polygon=[v[:] for v in region['polygon']];top=pixels_on_joint(line,np.asarray(region['polygon'][:2],float));region['polygon'][:2]=top.tolist();report.update(original_polygon=original_polygon,polygon=region['polygon'],joint_pixel_correction_px=correction.tolist(),measured_joint_image_line=line.tolist())
                                wall_models[region['id']]={'plane':plane,'joint_line':line}
                    if is_ceiling:
                        wall=wall_models.get(region['wall_region'])
                        if wall is None or wall.get('plane') is None:raise ValueError('Wall stage/junction was not established first')
                        pixels=pixels_on_joint(wall['joint_line'],np.asarray(region['junction_pixels'],float));junction,t=lift_pixels(pixels,f['k_rgb'],f['world_optical'],*wall['plane']);residual=abs((junction-centre)@normal)
                        original_polygon=[v[:] for v in region['polygon']];edge=pixels_on_joint(wall['joint_line'],np.asarray(region['polygon'][2:4],float));region['polygon'][2:4]=edge.tolist();report.update(original_polygon=original_polygon,polygon=region['polygon'])
                        report['wall_top_junction_disagreement_m']=residual.tolist()
                        if not np.isfinite(junction).all() or residual.max()>cfg['maximum_junction_disagreement_m']:raise ValueError('Measured ceiling disagrees with photographed wall-top junction')
                    def lift(px):
                        xyz,t=lift_pixels(px,f['k_rgb'],f['world_optical'],centre,normal);valid=np.isfinite(xyz).all(1)
                        if region.get('reuse_opaque_boundary'):valid&=xyz[:,2]<=wall_models[region['reuse_opaque_boundary']]['profile']['maximum_opaque_z']
                        return xyz,t,valid
                    if not is_ceiling and region['id'] not in wall_models:wall_models[region['id']]={'plane':plane}
                    report.update(model='original RGB-D RANSAC + TLS plane',plane_centre=centre.tolist(),plane_normal=normal.tolist())
                mesh,uv,texture=make_photo_patch(o3d,f,region,lift,dest);keep,missing=missing_triangles(o3d,mesh,f,scene);report['missing_surface_test']=missing
                uv=uv[keep];mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
                if not len(mesh.triangles):
                    report.update(accepted=True,added_triangles=0,area_m2=0.,reason='Already represented by existing observed geometry');reports.append(report);continue
                vertices=np.asarray(mesh.vertices);p3=vertices[np.asarray(mesh.triangles)];area=np.linalg.norm(np.cross(p3[:,1]-p3[:,0],p3[:,2]-p3[:,0]),axis=1).sum()/2
                report.update(accepted=True,added_triangles=len(mesh.triangles),area_m2=float(area),xyz_min=vertices.min(0).tolist(),xyz_max=vertices.max(0).tolist(),texture=texture.name,texture_sha256=sha256_file(texture))
                meshes.append(mesh);uvs.append(uv);textures.append(texture);combined+=mesh;scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
                print(region['id']+' / added '+str(len(mesh.triangles))+' / '+str(round(area,2))+'m2',flush=True)
            except ValueError as error:
                report.update(accepted=False,reason=str(error));print(region['id']+' / withheld: '+str(error),flush=True)
            reports.append(report)
        con.close();combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),combined);append_planes(base/name/'clean_textured.glb',dest/'clean_textured.glb',meshes,uvs,textures)
        for obj in [o for o in semantics['objects'] if o['session']==name]:
            for file in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                if file and (base/name/file).exists():shutil.copyfile(base/name/file,dest/file)
        s={**s,'upper_wall_review':reports,'original_source_faces':len(old.triangles),'source_mesh':str(base/name/'clean_structural.ply'),'source_texture':str(base/name/'clean_textured.glb')};run['sessions'][name]=s
        (dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n');cleanup[name]={'removed_faces':0,'before_faces':len(old.triangles),'after_faces':len(combined.triangles),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for path in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:
            path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
        del old,combined,scene,observed,xyz;gc.collect()
    run['created_utc']=utc_now();(a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n');(a.output/'semantic_objects.json').write_text(json.dumps(semantics,indent=2,ensure_ascii=False)+'\n')
    (a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)


if __name__=='__main__':main()
