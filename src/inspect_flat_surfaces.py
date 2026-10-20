#!/usr/bin/env python3
"""Inspect horizontal levels in the actually selected local display geometry."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def selected_sources(name,base,policy):
    if policy['sessions'][name]['method']=='balanced_only':
        return ROOT/'outputs/student_union_preserved_display_v1'/name/'preserved_geometry.ply',ROOT/'outputs/student_union_appearance_v2'/name/'balanced_only.glb'
    return base/name/'clean_structural.ply',base/name/'clean_textured.glb'


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    view=json.loads((ROOT/'config/current_local_view.json').read_text());base=ROOT/Path(view['structural_manifest']).parent;policy=json.loads((ROOT/view['structural_selection_policy']).read_text())
    results={}
    for name,s in selected['sessions'].items():
        ply,glb=selected_sources(name,base,policy);mesh=o3d.io.read_triangle_mesh(str(ply));mesh.compute_triangle_normals()
        xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)];centres=xyz.mean(1);area=np.linalg.norm(np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]),axis=1)/2
        normal=np.asarray(mesh.triangle_normals);keep=(abs(normal[:,2])>.90)&(area>1e-7)
        bins=np.arange(np.floor(centres[:,2].min()*10)/10-.2,np.ceil(centres[:,2].max()*10)/10+.25,.05)
        hist=np.histogram(centres[keep,2],bins=bins,weights=area[keep])[0];smooth=gaussian_filter1d(hist,1.5)
        peaks,_=find_peaks(smooth,distance=8,prominence=max(.3,smooth.max()*.025))
        poses=load_pose_csv(mp.parent/s['trajectory']);camera=np.array([v[:3,3] for v in poses.values()]);levels=[]
        for k in sorted(peaks,key=lambda k:-smooth[k]):
            level=(bins[k]+bins[k+1])/2;chosen=keep&(abs(centres[:,2]-level)<.25)
            levels.append({'height_m':float(level),'horizontal_area_within_25cm_m2':float(area[chosen].sum()),
                'xy_bounds':[centres[chosen,:2].min(0).tolist(),centres[chosen,:2].max(0).tolist()],
                'camera_z_minus_level_quantiles':np.quantile(camera[:,2]-level,[.05,.25,.5,.75,.95]).tolist()})
        results[name]={'source_ply':str(ply),'source_glb':str(glb),'triangles':len(xyz),'levels':levels,
            'camera_z_quantiles_m':np.quantile(camera[:,2],[0,.05,.25,.5,.75,.95,1]).tolist(),
            'height_histogram':{'bins':bins.tolist(),'area':hist.tolist()},'source_ply_sha256':sha256_file(ply),'source_glb_sha256':sha256_file(glb)}
        print(name,json.dumps({'camera_z':results[name]['camera_z_quantiles_m'],'levels':levels},ensure_ascii=False),flush=True)
    (a.output/'horizontal_inventory.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':results,
        'selected_manifest_sha256':ptr['manifest_sha256'],'source_manifest':str(base/'manifest.json')},indent=2)+'\n')


if __name__=='__main__':main()
