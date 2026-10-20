#!/usr/bin/env python3
"""Display-only local height fields, preserving observed height discontinuities."""
import argparse
import json
from pathlib import Path
import struct

import numpy as np
from scipy import ndimage
from scipy.signal import find_peaks
from scipy.spatial import cKDTree

from clean_structural_fragments import filter_glb
from optimize_session import sha256_file,utc_now
from rebuild_structural_display import bounded_mask
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from structural_prior_profiles import regularize_height_planes

ROOT=Path(__file__).resolve().parents[1]
STEP=.1


def smooth_height(height,valid,maximum_motion=.06):
    near=ndimage.distance_transform_edt(~valid,return_distances=False,return_indices=True)
    filled=height[near[0],near[1]]
    guide=ndimage.median_filter(filled,size=3,mode='nearest')
    high=ndimage.maximum_filter(guide,size=3);low=ndimage.minimum_filter(guide,size=3)
    # Guard real steps BEFORE smoothing can reduce their apparent height.
    discontinuity=ndimage.binary_dilation((high-low)>.12,iterations=2)
    smooth=ndimage.gaussian_filter(np.where(valid,guide,0.),2.)/np.maximum(ndimage.gaussian_filter(valid.astype(float),2.),1e-8)
    result=filled.copy();result[valid]=height[valid]+np.clip(smooth[valid]-height[valid],-maximum_motion,maximum_motion)
    return result,discontinuity


def append_glb(source,target,mesh):
    if target.exists():raise FileExistsError(target)
    raw=source.read_bytes();length=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+length]);payload=bytearray(raw[28+length:])
    t=np.asarray(mesh.triangles);ids=t.ravel();attrs={}
    for label,array in [('POSITION',np.asarray(mesh.vertices)),('NORMAL',np.asarray(mesh.vertex_normals)),('COLOR_0',srgb_to_linear(np.asarray(mesh.vertex_colors)))]:
        data=np.asarray(array[ids],dtype='<f4');payload.extend(b'\0'*(-len(payload)%4));start=len(payload);payload.extend(data.tobytes())
        doc['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':data.nbytes,'target':34962})
        doc['accessors'].append({'bufferView':len(doc['bufferViews'])-1,'componentType':5126,'count':len(data),'type':'VEC3','min':data.min(0).tolist(),'max':data.max(0).tolist()})
        attrs[label]=len(doc['accessors'])-1
    material={'doubleSided':True,'extensions':{'KHR_materials_unlit':{}},'pbrMetallicRoughness':{'metallicFactor':0.,'roughnessFactor':1.}}
    doc['materials'].append(material);doc['meshes'][0]['primitives'].append({'attributes':attrs,'mode':4,'material':len(doc['materials'])-1})
    doc['buffers'][0]['byteLength']=len(payload);encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with target.open('wb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(payload)));f.write(struct.pack('<I4s',len(encoded),b'JSON'));f.write(encoded)
        f.write(struct.pack('<I4s',len(payload),b'BIN\0'));f.write(payload)


def height_bands(levels,halfwidth=.5):
    groups=[]
    for level in sorted(levels):
        if groups and level-groups[-1][-1]<=2*halfwidth:groups[-1].append(level)
        else:groups.append([level])
    return [(float(np.mean(g)),g[0]-halfwidth,g[-1]+halfwidth) for g in groups]


def build_field(o3d,points,colours,level,lower=None,upper=None,strong=False):
    lower=level-.5 if lower is None else lower;upper=level+.5 if upper is None else upper
    ids=np.flatnonzero((points[:,2]>=lower)&(points[:,2]<upper));p=points[ids];rgb=colours[ids]
    low=np.floor(p[:,:2].min(0)/STEP)*STEP;xy=np.floor((p[:,:2]-low)/STEP).astype(int);w,h=xy.max(0)+1
    linear=xy[:,1]*w+xy[:,0];order=np.argsort(linear);keys,starts,counts=np.unique(linear[order],return_index=True,return_counts=True)
    heights=np.zeros((h,w));occupied=np.zeros((h,w),bool)
    for key,start,count in zip(keys,starts,counts):
        values=p[order[start:start+count],2]
        if count>=2 and np.percentile(values,90)-np.percentile(values,10)<.18:
            occupied.flat[key]=True;heights.flat[key]=np.median(values)
    if occupied.sum()<500:return None
    smoothed,steps=smooth_height(heights,occupied);mask=bounded_mask(occupied,radius=2,maximum_hole_area=.25,cell=STEP)&~steps
    labels,num=ndimage.label(mask);sizes=np.bincount(labels.ravel());keep=sizes*STEP*STEP>=3.;keep[0]=False;mask&=keep[labels]
    plane_report=[]
    if strong:smoothed,plane_report=regularize_height_planes(smoothed,mask,STEP)
    # Vertices are the height samples; require all four corners of each cell.
    cells=mask[:-1,:-1]&mask[1:,:-1]&mask[:-1,1:]&mask[1:,1:]
    y,x=np.indices((h,w));v=np.c_[x.ravel()*STEP+low[0],y.ravel()*STEP+low[1],smoothed.ravel()]
    index=np.arange(h*w).reshape(h,w);a=index[:-1,:-1][cells];b=a+1;c=a+w;d=c+1
    if len(a)<200:return None
    t=np.concatenate([np.c_[a,b,c],np.c_[b,d,c]])
    tree=cKDTree(p);distance,near=tree.query(v,k=min(8,len(p)),workers=2);weight=1/np.maximum(distance,.025)**2
    color=(srgb_to_linear(rgb[near])*weight[:,:,None]).sum(1)/weight.sum(1)[:,None]
    field=color.reshape(h,w,3);field=.6*field+.4*ndimage.gaussian_filter(field,(1.,1.,0))
    luminance=np.maximum(field@np.array([.2126,.7152,.0722]),.005);lighting=ndimage.gaussian_filter(luminance,5.)
    reference=np.median(lighting[mask]);field*=np.clip(reference/lighting,.8,1.25)[:,:,None]
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(v);mesh.triangles=o3d.utility.Vector3iVector(t)
    mesh.vertex_colors=o3d.utility.Vector3dVector(linear_to_srgb(field.reshape(-1,3)));mesh.compute_vertex_normals()
    used=mask&occupied
    return mesh,{'low':low,'heights':smoothed,'cells':cells,'level':level,'measured_mask':occupied,'step_guard':steps,
        'plane_prior':plane_report,'actual_maximum_measured_height_motion_m':float(np.max(abs(smoothed[used]-heights[used]))) if used.any() else 0.}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sessions',nargs='+');parser.add_argument('--resume',action='store_true')
    parser.add_argument('--profile',choices=['conservative','strong-indoor'],default='conservative')
    a=parser.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());source=json.loads((a.source/'manifest.json').read_text())
    if sha256_file(mp)!=ptr['manifest_sha256'] or source['selected_manifest_sha256']!=ptr['manifest_sha256']:raise ValueError('Selected map mismatch')
    previous=json.loads((a.output/'manifest.json').read_text()) if a.resume else {}
    results=previous.get('sessions',{});hashes=previous.get('inputs_sha256',{})
    if previous and previous.get('profile','conservative')!=a.profile:raise ValueError('Profile changed during resume')
    for path in [Path(__file__),mp,a.source/'manifest.json',ROOT/'src/clean_structural_fragments.py',ROOT/'src/rebuild_structural_display.py',ROOT/'src/repair_visual_appearance.py',ROOT/'src/structural_prior_profiles.py']:
        current=sha256_file(path)
        if str(path) in hashes and hashes[str(path)]!=current:raise ValueError('Source changed during resume')
        hashes[str(path)]=current
    for n in a.sessions or selected['sessions']:
        if n in results:continue
        s=selected['sessions'][n]
        effective_strong=a.profile=='strong-indoor' and n in ['floor2','lobby']
        print(n+': step-preserving local height fields',flush=True);dest=a.output/n;dest.mkdir()
        for path in [mp.parent/s['cloud'],a.source/n/'structural.ply',a.source/n/'structural_textured.glb']:
            hashes[str(path)]=sha256_file(path)
        pc=o3d.io.read_point_cloud(str(mp.parent/s['cloud'])).voxel_down_sample(.075)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.3,max_nn=40))
        horizontal=abs(np.asarray(pc.normals)[:,2])>.7;p=np.asarray(pc.points)[horizontal];colours=np.asarray(pc.colors)[horizontal]
        hist,bins=np.histogram(p[:,2],bins=np.arange(p[:,2].min()-.1,p[:,2].max()+.15,.06))
        peaks,_=find_peaks(hist,distance=8,prominence=max(200,int(hist.max()*.06)))
        levels=[float((bins[k]+bins[k+1])/2) for k in peaks if hist[k]>max(700,hist.max()*.1)]
        original=o3d.io.read_triangle_mesh(str(a.source/n/'structural.ply'));original.compute_triangle_normals();t=np.asarray(original.triangles);v=np.asarray(original.vertices)
        centres=v[t].mean(1);normals=np.asarray(original.triangle_normals);replace=np.zeros(len(t),bool);fields=[];reports=[]
        for level,lower,upper in height_bands(levels):
            built=build_field(o3d,p,colours,level,lower,upper,strong=effective_strong)
            if built is None:continue
            mesh,field=built;h,w=field['cells'].shape;xy=np.floor((centres[:,:2]-field['low'])/STEP).astype(int);x,y=xy.T
            ii=np.flatnonzero((x>=0)&(x<w)&(y>=0)&(y<h)&(abs(normals[:,2])>.3))
            inside=field['cells'][y[ii],x[ii]]&(abs(centres[ii,2]-field['heights'][y[ii],x[ii]])<.20)
            replace[ii[inside]]=True;fields.append(mesh)
            reports.append({'level_mode_m':level,'nonoverlapping_height_band_m':[lower,upper],
                'cells':int(field['cells'].sum()),'area_xy_m2':float(field['cells'].sum()*STEP*STEP),
                'guarded_discontinuity_samples':int(field['step_guard'].sum()),
                'maximum_measured_height_motion_m':field['actual_maximum_measured_height_motion_m'],
                'measured_height_motion_limit_m':.24 if effective_strong else .06,'plane_prior':field['plane_prior'],
                'height_span_m':float(np.ptp(np.asarray(mesh.vertices)[np.unique(np.asarray(mesh.triangles)),2]))})
        filtered=dest/'walls_preserved.glb';filter_glb(a.source/n/'structural_textured.glb',filtered,~replace)
        original.remove_triangles_by_mask(replace);original.remove_unreferenced_vertices();combined=o3d.geometry.TriangleMesh(original)
        addition=o3d.geometry.TriangleMesh()
        for field in fields:addition+=field;combined+=field
        output_glb=dest/'structural_textured.glb'
        if len(addition.triangles):append_glb(filtered,output_glb,addition)
        else:
            # No floor model is invented where there is insufficient support.
            import shutil
            shutil.copyfile(filtered,output_glb)
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'structural.ply'),combined)
        results[n]={**source['sessions'][n],'heightfields':reports,'heightfield_replaced_faces':int(replace.sum()),
            'outputs':{p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in dest.iterdir() if p.is_file()}}
        (dest/'summary.json').write_text(json.dumps(results[n],indent=2)+'\n')
        (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'inputs_sha256':hashes,
            'profile':a.profile,
            'selected_manifest_sha256':ptr['manifest_sha256'],'selected_map_changed':False,'navigation_ready':False,
            'scope':'Finite wall patches plus local height fields; display modeling, not a new pose graph or a navigation mesh.'},indent=2)+'\n')
        print(json.dumps({'complete':n,'heightfields':reports,'replaced_faces':int(replace.sum())}),flush=True)
    for path,h in hashes.items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed')


if __name__=='__main__':main()
