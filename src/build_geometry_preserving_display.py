#!/usr/bin/env python3
"""Full-map conservative display: unchanged TSDF vertices, no new faces or smoothing."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np

from audit_display_depth import compare_depths,depth_image
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from reconstruct_surface_portfolio import texture_mesh
from refine_surface_display import standardize_glb
from repair_surface_fragmentation import export_rgb,topology
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def component_keep(areas,labels,minimum=.005):
    areas=np.asarray(areas);labels=np.asarray(labels)
    return areas[labels]>=minimum


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());cfg_path=ROOT/'config/student_union_surface_portfolio.json';cfg=json.loads(cfg_path.read_text())
    hashes={str(path):sha256_file(path) for path in [pointer_path,mp,cfg_path,Path(__file__),
        ROOT/'src/surface_model_utils.py',ROOT/'src/reconstruct_surface_portfolio.py',ROOT/'src/refine_surface_display.py']}
    source=ROOT/'outputs/student_union_surface_portfolio_v1';records={}
    for name,record in selected['sessions'].items():
        print(name+': unchanged vertices / micro-fragment cleanup',flush=True)
        dest=a.output/name;dest.mkdir();source_summary=json.loads((source/name/'summary.json').read_text())
        paths=[source/name/'tsdf_5cm.ply',mp.parent/record['trajectory'],Path(selected['database_integrity'][name]['database'])]
        for path in paths:hashes[str(path)]=sha256_file(path)
        original=o3d.io.read_triangle_mesh(str(paths[0]));mesh=o3d.geometry.TriangleMesh(original)
        labels,counts,areas=mesh.cluster_connected_triangles();counts=np.asarray(counts);areas=np.asarray(areas)
        keep=component_keep(areas,labels);original_vertices=np.asarray(original.vertices)
        old_triangles=np.asarray(original.triangles);expected=original_vertices[old_triangles[keep]]
        mesh.remove_triangles_by_mask(~keep)
        # Verify the exact retained face coordinates BEFORE dropping unused vertices.
        assert np.array_equal(np.asarray(mesh.vertices)[np.asarray(mesh.triangles)],expected)
        mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
        assert np.array_equal(np.asarray(mesh.vertices)[np.asarray(mesh.triangles)],expected)
        del expected
        o3d.io.write_triangle_mesh(str(dest/'preserved_geometry.ply'),mesh);export_rgb(mesh,dest/'preserved_geometry.glb')
        poses=load_pose_csv(paths[1]);con=sqlite3.connect(f'file:{paths[2]}?mode=ro',uri=True)
        nodes=source_summary['texture']['camera_frames'];frames={n:decode_frame(con,n,record['preprocessing'],poses[n]) for n in nodes}
        primary=cfg['representative_frames'][name][0];unused=sorted(set(poses)-set(nodes))
        audit=min(unused,key=lambda n:abs(n-primary)) if unused else primary
        probes={label:decode_frame(con,n,record['preprocessing'],poses[n]) for label,n in [('source',primary),('audit',audit)]}
        if name=='floor2':probes['stairs']=decode_frame(con,1078,record['preprocessing'],poses[1078])
        if name=='bridge_12_sub':probes['stairs']=decode_frame(con,108,record['preprocessing'],poses[108])
        con.close();checks={}
        for label,f in probes.items():
            before=depth_image(o3d,original,f);after=depth_image(o3d,mesh,f)
            checks[label]={'node':f['id'],**compare_depths(before,after,f['depth'])}
        print(name+': native photo texture, no decimation',flush=True)
        texture=texture_mesh(mesh,frames,cfg['texture'],dest);assignment=texture.pop('camera_assignment')
        np.save(dest/'texture_frame_assignment.npy',np.asarray(assignment,np.int32))
        standardize_glb(dest/'photo_textured.glb',dest/'preserved_textured.glb')
        records[name]={'session':name,'candidate':record['candidate'],'source_mesh':str(paths[0]),
            'vertex_displacement_m':0.,'new_triangles':0,'decimation':False,'smoothing':False,'hole_filling':False,
            'exact_retained_triangle_coordinates_verified':True,'minimum_component_area_m2':.005,
            'original_components':len(counts),'removed_components':int(np.sum(areas<.005)),
            'removed_triangles':int((~keep).sum()),'removed_surface_area_m2':float(areas[areas<.005].sum()),
            'topology':topology(mesh),'camera_nodes':{label:f['id'] for label,f in probes.items()},
            'depth_checks':checks,'texture':texture,'navigation_ready':False,
            'outputs':{p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in dest.iterdir() if p.is_file()}}
        (dest/'summary.json').write_text(json.dumps(records[name],indent=2)+'\n')
        for path,h in hashes.items():
            if sha256_file(Path(path))!=h:raise ValueError('Frozen input changed')
        (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':records,
            'inputs_sha256':hashes,'selected_manifest_sha256':pointer['manifest_sha256'],
            'source_map_replaced':False,'all_inputs_unchanged':True,'navigation_ready':False,
            'policy':'Full 5cm TSDF display; subset of original triangles only. RGB texture does not alter geometry.',
            'limitations':['Original 5cm TSDF approximates the selected point cloud; it is not sensor truth.',
                'Removing tiny components may remove a real tiny isolated feature; original files remain.',
                'A removed foreground fragment can expose background, changing ray-depth statistics.',
                'Larger holes and existing registration error are intentionally not concealed.']},indent=2)+'\n')
        print(json.dumps({'complete':name,'components':records[name]['topology']['components'],
            'triangles':len(mesh.triangles),'vertex_motion':0}),flush=True)


if __name__=='__main__':main()
