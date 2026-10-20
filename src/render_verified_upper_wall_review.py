#!/usr/bin/env python3
"""Actual before/after five-session renders; new wall/ceiling faces highlighted."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from optimize_session import inverse,sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);a=p.parse_args();a.assets.mkdir(parents=True,exist_ok=True)
    run=json.loads((a.bundle/'manifest.json').read_text());before=Path(run['base_display_manifest']).parent;road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',22)
    views=[]
    for name,node,pitch in [('lobby',713,0),('lobby',167,0),('lobby',713,18),('lobby',579,0),('floor2',726,0),('floor2',728,0),('floor2',282,0)]:
        v=min([n for n in nodes if n['session']==name],key=lambda n:abs(n['node']-node));pose=np.asarray(v['world_optical']).copy();pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',pitch,degrees=True).as_matrix();k=np.asarray(v['K']).copy();k[:2]*=800/960;k[0,2]+=(800-720*800/960)/2
        views.append({'session':name,'node':v['node'],'K':k.tolist(),'world_optical':pose.tolist(),'source_photo':v['photo'],'additional_upward_pitch_deg':pitch})
    import open3d as o3d
    r=o3d.visualization.rendering.OffscreenRenderer(800,800);r.scene.set_background([.055,.065,.08,1.]);r.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR));images=[[] for _ in views]
    for mode in ['before','after','highlight']:
        r.scene.clear_geometry()
        for name,s in run['sessions'].items():
            folder=(before if mode=='before' else a.bundle)/name
            if mode=='highlight':
                mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'));mesh.compute_vertex_normals();tri=np.asarray(mesh.triangles);xyz=np.asarray(mesh.vertices)[tri];colour=np.full((len(tri),3),.55);cursor=s['original_source_faces']
                for region in s['upper_wall_review']:
                    count=region.get('added_triangles',0) if region.get('accepted') else 0
                    colour[cursor:cursor+count]=[.1,.75,.85] if region['kind']=='upper_wall' else [1.,.55,.1];cursor+=count
                flat=o3d.geometry.TriangleMesh();flat.vertices=o3d.utility.Vector3dVector(xyz.reshape(-1,3));flat.triangles=o3d.utility.Vector3iVector(np.arange(len(xyz)*3).reshape(-1,3));flat.vertex_colors=o3d.utility.Vector3dVector(np.repeat(colour,3,axis=0));mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultUnlit';r.scene.add_geometry(name,flat,mat)
            else:
                model=o3d.io.read_triangle_model(str(folder/'clean_textured.glb'))
                for mat in model.materials:mat.shader='defaultUnlit'
                r.scene.add_model(name,model)
        for i,view in enumerate(views):
            r.setup_camera(np.asarray(view['K']),inverse(np.asarray(view['world_optical'])),800,800);images[i].append(np.asarray(r.render_to_image()).copy())
        print('Upper-wall review: '+mode,flush=True)
    figures=[]
    for view,rendered in zip(views,images):
        original=Image.open(road/view['source_photo']).convert('RGB');original.thumbnail((800,800));photo=Image.new('RGB',(800,800),'#e9edf0');photo.paste(original,((800-original.width)//2,(800-original.height)//2));pitch=view['additional_upward_pitch_deg'];description='원본 시점' if not pitch else f'렌더 시선만 위로 {pitch}도'
        sheet=Image.new('RGB',(3200,900),'white');draw=ImageDraw.Draw(sheet);draw.text((15,10),f'{view["session"]}:{view["node"]} | {description} · 다섯 기록 · 기존 포즈/층 높이 불변',font=font,fill='#172538')
        labels=['기준 원본 사진','이전: 관측면만 / 벽 상부 결손','수정: 사진 범위의 벽부터 보완','청록: 새 벽 / 주황: 검증한 국소 천장']
        if pitch:labels[0]+=f' (렌더는 위 {pitch}도)'
        for i,(image,label) in enumerate(zip([np.asarray(photo),*rendered],labels)):
            draw.text((800*i+12,57),label,font=font,fill='#172538');sheet.paste(Image.fromarray(image),(800*i,100))
        suffix='' if not pitch else '_up'+str(pitch);file=f'{view["session"]}_{view["node"]}{suffix}_comparison.png';sheet.save(a.assets/file);figures.append({'file':file,'sha256':sha256_file(a.assets/file),'camera':view,'source_photo_sha256':sha256_file(road/view['source_photo'])})
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'renderer_sha256':sha256_file(Path(__file__)),
        'scope':'Original photographs and actual GLB renders; colors identify new modeled faces, not measured accuracy or automatic semantics.'},indent=2)+'\n')


if __name__=='__main__':main()
