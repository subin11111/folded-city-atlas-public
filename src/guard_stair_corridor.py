#!/usr/bin/env python3
"""Keep the inspected main-stair corridor's existing geometry and materials."""
import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from rebuild_structural_display import world_grid
from reconstruct_continuous_planes import build_mesh

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(exist_ok=False,parents=True)
    import open3d as o3d
    run=json.loads((a.source/'manifest.json').read_text())
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    for path in [Path(__file__),a.source/'manifest.json',a.baseline/'manifest.json',ROOT/'src/append_plane_glb.py',ROOT/'src/clean_structural_fragments.py']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    for n,s in run['sessions'].items():
        dest=a.output/n;dest.mkdir()
        if n!='floor2':
            for path in (a.source/n).iterdir():
                if path.is_file() and path.suffix in ['.ply','.glb','.npz','.npy','.png','.json']:shutil.copyfile(path,dest/path.name)
            continue
        poses=load_pose_csv(mp.parent/selected['sessions'][n]['trajectory'])
        anchors=np.array([poses[node][:3,3] for node in [145,1092,1150]])
        # Reviewed photos establish the staircase. Intermediate camera heights
        # select the climb, within 4m XY of these photographed stair locations.
        ids=[node for node,pose in poses.items() if .35<pose[2,3]<3.7 and np.min(np.linalg.norm(anchors[:,:2]-pose[:2,3],axis=1))<4.]
        centres=np.array([poses[node][:3,3] for node in ids])+[0,0,-.55]
        half=np.array([1.2,1.2,1.25]);tree=cKDTree(centres/half)
        def protected(points):return tree.query(np.asarray(points)/half,p=np.inf,workers=2)[0]<=1.
        baseline=a.baseline/n/'clean_structural.ply';old=o3d.io.read_triangle_mesh(str(baseline));old.compute_triangle_normals()
        v=np.asarray(old.vertices);tri=np.asarray(old.triangles);face_c=v[tri].mean(1);normal=np.asarray(old.triangle_normals).copy()
        protected_faces=protected(v)[tri].any(1)|protected(face_c)
        remove=np.zeros(len(tri),bool);meshes=[];uvs=[];textures=[];reports=[];excluded=0
        cloud=o3d.io.read_point_cloud(str(mp.parent/selected['sessions'][n]['cloud']));colour_tree=cKDTree(np.asarray(cloud.points))
        for path in sorted((a.source/n).glob('mask_*.npz')):
            index=int(path.stem.split('_')[-1]);pack=np.load(path);data={key:pack[key] for key in pack.files}
            plane={key:data[key] for key in ['centre','basis','low']};mask=data['mask'].copy();h,w=mask.shape
            report=next(r for r in s['continuous_planes'] if np.allclose(r['centre'],plane['centre']) and r['shape']==list(mask.shape))
            corners=world_grid(plane['centre'],plane['basis'],plane['low'],(h+1,w+1))
            guard=protected(corners).reshape(h+1,w+1)
            forbidden=guard[:-1,:-1]|guard[1:,:-1]|guard[:-1,1:]|guard[1:,1:]
            forbidden=ndimage.binary_dilation(forbidden,iterations=1)
            count=int((mask&forbidden).sum());excluded+=count;mask&=~forbidden
            if not mask.any():continue
            data['mask']=mask;data['stair_guard']=forbidden
            np.savez_compressed(dest/path.name,**data)
            mesh,uv=build_mesh(o3d,plane,mask)
            _,nearest=colour_tree.query(np.asarray(mesh.vertices),workers=2)
            mesh.vertex_colors=o3d.utility.Vector3dVector(np.asarray(cloud.colors)[nearest])
            q=(face_c-plane['centre'])@plane['basis'];ij=np.floor((q[:,:2]-plane['low'])/.06).astype(int);x,y=ij.T
            ii=np.flatnonzero((abs(q[:,2])<.32)&(abs(normal@plane['basis'][:,2])>.65)&(x>=0)&(x<w)&(y>=0)&(y<h))
            remove[ii[mask[y[ii],x[ii]]]]=True
            for prefix in ['continuous','coverage','photo_labels']:
                source=a.source/n/f'{prefix}_{index:03d}.png';shutil.copyfile(source,dest/source.name)
            coverage=cv2.imread(str(dest/f'coverage_{index:03d}.png'),cv2.IMREAD_GRAYSCALE)
            resized=cv2.resize(mask.astype(np.uint8),(coverage.shape[1],coverage.shape[0]),interpolation=cv2.INTER_NEAREST)>0
            coverage[~resized]=0;cv2.imwrite(str(dest/f'coverage_{index:03d}.png'),coverage)
            reports.append({**report,'cells':int(mask.sum()),'new_supported_cells':int((mask&~data['observed']).sum()),
                'stair_guard_excluded_cells':count,'photo_coverage_on_surface_fraction':float((coverage[resized]==255).mean())})
            meshes.append(mesh);uvs.append(uv);textures.append(dest/f'continuous_{index:03d}.png')
        remove[protected_faces]=False
        np.save(dest/'removed_source_faces.npy',remove);np.save(dest/'protected_stair_source_faces.npy',protected_faces)
        retained=len(tri)-int(remove.sum());old.remove_triangles_by_mask(remove);old.remove_unreferenced_vertices()
        residual=dest/'residual.glb';filter_glb(a.baseline/n/'clean_textured.glb',residual,~remove)
        combined=o3d.geometry.TriangleMesh(old)
        for mesh in meshes:combined+=mesh
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'structural.ply'),combined)
        append_planes(residual,dest/'structural_textured.glb',meshes,uvs,textures)
        s.update(patches=reports,continuous_planes=reports,rebuilt_groups=len(reports),retained_previous_faces=retained,
            replaced_original_faces=int(remove.sum()),horizontal_faces_removed=0,
            stair_guard={'reviewed_photo_nodes':[145,1092,1150],'camera_nodes':ids,'box_centres':centres.tolist(),
                'box_half_extents_m':half.tolist(),'preserved_source_triangles':int(protected_faces.sum()),
                'excluded_new_plane_area_m2':excluded*.06**2,'scope':'All existing geometry and materials inside the photographed stair corridor; no new modeled wall cells in that volume.'})
        (dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
        print('Protected stair triangles: '+str(protected_faces.sum()),flush=True)
    run.update(created_utc=utc_now(),stair_corridor_protection=True)
    (a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed '+path)


if __name__=='__main__':main()
