#!/usr/bin/env python3
"""Same-camera interior renders and isolated horizontal geometry comparisons."""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def panel(images,title,path,font,labels):
    sheet=Image.new('RGB',(700*len(images),790),'white');draw=ImageDraw.Draw(sheet)
    draw.text((12,10),title,font=font,fill='#172538')
    for i,(im,label) in enumerate(zip(images,labels)):
        draw.text((700*i+10,49),label,font=font,fill='#172538');sheet.paste(Image.fromarray(im),(700*i,90))
    sheet.save(path)


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);p.add_argument('--sessions',nargs='+');a=p.parse_args()
    a.assets.mkdir(exist_ok=True,parents=True);run=json.loads((a.bundle/'manifest.json').read_text())
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    prior=a.assets/'manifest.json';saved=json.loads(prior.read_text()) if prior.exists() else {};figures=saved.get('figures',[]);completed=saved.get('completed_sessions',[])
    import open3d as o3d
    renderer=o3d.visualization.rendering.OffscreenRenderer(700,700);renderer.scene.set_background([.055,.065,.08,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',19)
    for name in a.sessions or run['sessions']:
        if name in completed:continue
        s=run['sessions'][name];dest=a.assets/name;dest.mkdir(exist_ok=True);planes=[x for x in s['flat_sheets'] if x['accepted']]
        before=o3d.io.read_triangle_mesh(s['source_mesh']);before.compute_triangle_normals();old_centres=np.asarray(before.vertices)[np.asarray(before.triangles)].mean(1)
        after=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        views={};frame_rgb={}
        for plane in planes:
            nodes=plane['original_photo_nodes'];node=nodes[0] if nodes else min(poses,key=lambda n:abs(poses[n][2,3]-plane['height_m']-(1.5 if plane['kind']=='floor' else -1.5)))
            f=decode_frame(con,node,selected['sessions'][name]['preprocessing'],poses[node]);pose=f['world_optical'].copy();pitch=-22 if plane['kind']=='floor' else 27
            pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',pitch,degrees=True).as_matrix()
            k=f['k_rgb'].copy();k[:2]*=700/960;k[0,2]+=(700-720*700/960)/2
            views[plane['id']]={'K':k.tolist(),'world_optical':pose.tolist(),'node':node,'additional_camera_pitch_deg':pitch}
            frame_rgb[plane['id']]=f['rgb']
        if name=='floor2':
            for node in [145,1092]:
                f=decode_frame(con,node,selected['sessions'][name]['preprocessing'],poses[node]);k=f['k_rgb'].copy();k[:2]*=700/960;k[0,2]+=(700-720*700/960)/2
                views['protected_stair_'+str(node)]={'K':k.tolist(),'world_optical':f['world_optical'].tolist(),'node':node,'additional_camera_pitch_deg':0}
        con.close();renders={key:[] for key in views}
        for mode in ['before','after','gray']:
            renderer.scene.clear_geometry()
            if mode=='gray':
                mesh=o3d.geometry.TriangleMesh(after);mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.65));mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';renderer.scene.add_geometry('model',mesh,mat)
                renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            else:
                path=Path(s['source_texture']) if mode=='before' else a.bundle/name/'clean_textured.glb';model=o3d.io.read_triangle_model(str(path))
                for mat in model.materials:mat.shader='defaultUnlit'
                renderer.scene.add_model('model',model)
            for key,view in views.items():
                renderer.setup_camera(np.asarray(view['K']),inverse(np.asarray(view['world_optical'])),700,700);renders[key].append(np.asarray(renderer.render_to_image()).copy())
        for key,images in renders.items():
            path=dest/(key+'_interior.png');panel(images,name+' / '+key+' | 같은 카메라 · 표시용 강한 수평면 / 포즈 불변',path,font,['직전 사진 의미 지도','넓게 채운 평평한 바닥·천장','새 기하 · 색 제외'])
            figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'type':'same_camera_interior','camera':views[key]})
        # Isolate each level to compare planarity without the new solid ceiling
        # hiding the floor from an exterior camera. Both panels use gray geometry.
        start=len(after.triangles)-sum(x['triangles'] for x in planes)
        for plane in planes:
            old=o3d.geometry.TriangleMesh(before);lower,upper=plane['height_band_m'];keep=(old_centres[:,2]>=lower)&(old_centres[:,2]<upper)&(abs(np.asarray(before.triangle_normals)[:,2])>.30)
            old.remove_triangles_by_mask(~keep);old.remove_unreferenced_vertices()
            new=o3d.geometry.TriangleMesh(after);keep_new=np.zeros(len(new.triangles),bool);keep_new[start:start+plane['triangles']]=True;start+=plane['triangles'];new.remove_triangles_by_mask(~keep_new);new.remove_unreferenced_vertices()
            bounds=new.get_axis_aligned_bounding_box();c=bounds.get_center();extent=max(bounds.get_extent()[:2]);eye=c+np.array([.15,-.55,.85 if plane['kind']=='floor' else -.85])*extent
            images=[]
            for mesh in [old,new]:
                renderer.scene.clear_geometry();mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.65));mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';renderer.scene.add_geometry('layer',mesh,mat)
                renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.]);renderer.setup_camera(55.,c,eye,[0,1,0]);images.append(np.asarray(renderer.render_to_image()).copy())
            path=dest/(plane['id']+'_geometry.png');panel(images,name+' / '+plane['id']+' | 같은 높이 구역 · 색 없는 기하 비교',path,font,['직전 높이가 흔들린 표면','Z 높이를 하나로 고정한 넓은 평면'])
            figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'type':'isolated_gray_geometry','plane':plane['id'],'camera':{'centre':c.tolist(),'eye':eye.tolist(),'up':[0,1,0]}})
        completed.append(name);prior.write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'completed_sessions':completed,
            'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'renderer_sha256':sha256_file(Path(__file__)),
            'scope':'Actual before/after geometry and textured renders; pitch changes explicitly recorded; not independent measurement accuracy.'},indent=2)+'\n')
        print('Saved horizontal comparisons: '+name+' / '+str(len(views)+len(planes)),flush=True)


if __name__=='__main__':main()
