#!/usr/bin/env python3
"""Refine only continuous-plane photograph charts; keep geometry exactly fixed."""
import argparse
import gc
import json
from pathlib import Path
import shutil
import sqlite3
import numpy as np

from optimize_session import sha256_file,utc_now
from export_rtabmap_db import load_pose_csv
from surface_model_utils import decode_frame
from reconstruct_continuous_planes import build_mesh
from aligned_photo_charts import texture_plane
from append_plane_glb import append_planes

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--resume',action='store_true');p.add_argument('--sessions',nargs='+');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True);source=json.loads((a.source/'manifest.json').read_text())
    prior=a.output/'manifest.json';run=json.loads(prior.read_text()) if prior.exists() else {**source,'sessions':{}}
    for path in [Path(__file__),a.source/'manifest.json',ROOT/'src/coherent_photo_charts.py',ROOT/'src/aligned_photo_charts.py',ROOT/'src/reconstruct_continuous_planes.py']:
        path=path.resolve();h=sha256_file(path)
        if str(path) in run['inputs_sha256'] and run['inputs_sha256'][str(path)]!=h:raise ValueError('Source changed')
        run['inputs_sha256'][str(path)]=h
    checkpoint_inputs=a.output/'run_inputs.json'
    if checkpoint_inputs.exists():
        if json.loads(checkpoint_inputs.read_text())!=run['inputs_sha256']:raise ValueError('Checkpoint inputs changed')
    else:checkpoint_inputs.write_text(json.dumps(run['inputs_sha256'],indent=2)+'\n')
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    import open3d as o3d
    for n,s in source['sessions'].items():
        if n in run['sessions']:continue
        if a.sessions and n not in a.sessions and s.get('continuous_planes'):continue
        dest=a.output/n;dest.mkdir(exist_ok=True);folder=a.source/n
        shutil.copyfile(folder/'structural.ply',dest/'structural.ply')
        planes=s.get('continuous_planes',[])
        if not planes:
            shutil.copyfile(folder/'structural_textured.glb',dest/'structural_textured.glb');run['sessions'][n]=s
        else:
            shutil.copyfile(folder/'removed_source_faces.npy',dest/'removed_source_faces.npy')
            spec=selected['sessions'][n];poses=load_pose_csv(mp.parent/spec['trajectory']);ids=sorted({node for plane in planes for node in plane['selected_photo_nodes']})
            con=sqlite3.connect(f'file:{selected["database_integrity"][n]["database"]}?mode=ro',uri=True)
            frames={node:decode_frame(con,node,spec['preprocessing'],poses[node]) for node in ids};con.close()
            reference=o3d.io.read_point_cloud(str(mp.parent/spec['cloud']));meshes=[];uvs=[];textures=[];reports=[]
            for path in sorted(folder.glob('mask_*.npz')):
                index=int(path.stem.split('_')[-1]);pack=np.load(path);plane={key:pack[key] for key in ['centre','basis','low']};mask=pack['mask']
                matches=[r for r in planes if np.allclose(r['centre'],plane['centre']) and r['shape']==list(mask.shape)]
                if len(matches)!=1:raise ValueError('Ambiguous saved plane')
                report=matches[0];mesh,uv=build_mesh(o3d,plane,mask)
                checkpoint=dest/f'chart_report_{index:03d}.json'
                texture=dest/f'continuous_{index:03d}.png'
                if checkpoint.exists():
                    saved=json.loads(checkpoint.read_text());photoreport=saved['report']
                    if sha256_file(texture)!=saved['texture_sha256']:raise ValueError('Checkpoint texture changed')
                else:
                    texture,photoreport=texture_plane(plane,mask,frames,report['selected_photo_nodes'],reference,dest,index)
                    checkpoint.write_text(json.dumps({'report':photoreport,'texture_sha256':sha256_file(texture)},indent=2)+'\n')
                meshes.append(mesh);uvs.append(uv);textures.append(texture)
                reports.append({**{k:v for k,v in report.items() if not k.startswith('overlap_linear_rgb')},**photoreport});shutil.copyfile(path,dest/path.name)
                print(n+': coherent photograph chart '+str(index),flush=True)
            append_planes(folder/'residual.glb',dest/'structural_textured.glb',meshes,uvs,textures)
            run['sessions'][n]={**s,'patches':reports,'continuous_planes':reports,'photo_refinement_geometry_unchanged':True}
            del frames,reference;gc.collect()
        run.update(created_utc=utc_now(),texture_refinement='Bounded SIFT-based photo alignment, coherent labels and bounded lighting leveling; geometry unchanged')
        (dest/'summary.json').write_text(json.dumps(run['sessions'][n],indent=2)+'\n');prior.write_text(json.dumps(run,indent=2)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed '+path)


if __name__=='__main__':main()
