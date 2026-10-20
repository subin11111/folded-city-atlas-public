#!/usr/bin/env python3
"""Show the rejected inferred ceilings versus unchanged observed source surfaces."""
import argparse,json,shutil
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from optimize_session import inverse,sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);a=p.parse_args();a.assets.mkdir(parents=True,exist_ok=True)
    run=json.loads((a.bundle/'manifest.json').read_text());before=Path(run['base_display_manifest']).parent
    road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',22)
    views=[]
    for session,node,pitch in [('lobby',516,0),('lobby',579,0),('floor2',445,0),('floor2',512,0),('floor2',1050,0),('lobby',516,35),('floor2',1050,30)]:
        item=min([v for v in nodes if v['session']==session],key=lambda v:abs(v['node']-node));k=np.asarray(item['K']).copy();k[:2]*=800/960;k[0,2]+=(800-720*800/960)/2
        pose=np.asarray(item['world_optical']).copy();pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',pitch,degrees=True).as_matrix()
        views.append({'session':session,'node':item['node'],'K':k.tolist(),'world_optical':pose.tolist(),'source_photo':item['photo'],
            'additional_upward_pitch_deg':pitch,'base_world_optical':item['world_optical']})
    import open3d as o3d
    r=o3d.visualization.rendering.OffscreenRenderer(800,800);r.scene.set_background([.055,.065,.08,1.])
    r.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    images=[[] for _ in views]
    for mode in ['before','after','gray']:
        r.scene.clear_geometry()
        for name in run['sessions']:
            folder=(before if mode=='before' else a.bundle)/name
            if mode=='gray':
                mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'));mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.65))
                mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';r.scene.add_geometry(name,mesh,mat)
                r.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            else:
                model=o3d.io.read_triangle_model(str(folder/'clean_textured.glb'))
                for mat in model.materials:mat.shader='defaultUnlit'
                r.scene.add_model(name,model)
        for i,view in enumerate(views):
            r.setup_camera(np.asarray(view['K']),inverse(np.asarray(view['world_optical'])),800,800);images[i].append(np.asarray(r.render_to_image()).copy())
        print('Ceiling withdrawal renders: '+mode,flush=True)
    figures=[]
    for view,rendered in zip(views,images):
        original=Image.open(road/view['source_photo']).convert('RGB');original.thumbnail((800,800));photo=Image.new('RGB',(800,800),'#e9edf0');photo.paste(original,((800-original.width)//2,(800-original.height)//2))
        pitch=view['additional_upward_pitch_deg'];description='원본 시점' if not pitch else f'촬영 위치에서 렌더 시선만 위로 {pitch}도'
        sheet=Image.new('RGB',(3200,900),'white');draw=ImageDraw.Draw(sheet);draw.text((15,10),f'{view["session"]}:{view["node"]} | {description} · 다섯 기록 · 벽/바닥/계단 좌표 불변',font=font,fill='#172538')
        original_label='원본 사진' if not pitch else f'기준 원본 사진 (오른쪽 렌더는 위 {pitch}도)'
        for i,(image,label) in enumerate(zip([np.asarray(photo),*rendered],[original_label,'기각: 제가 높이를 추정해 만든 천장','수정: 추가 천장 없음 / 기존 관측면만','수정 기하: 미관측 공간은 열어 둠'])):
            draw.text((800*i+12,57),label,font=font,fill='#172538');sheet.paste(Image.fromarray(image),(800*i,100))
        suffix='' if not pitch else '_up'+str(pitch);file=f'{view["session"]}_{view["node"]}{suffix}_comparison.png';sheet.save(a.assets/file)
        figures.append({'file':file,'sha256':sha256_file(a.assets/file),'camera':view,'source_rgb_sha256':sha256_file(road/view['source_photo'])})
    shutil.copyfile(a.bundle/'ceiling_withdrawal_audit.json',a.assets/'ceiling_withdrawal_audit.json')
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'renderer_sha256':sha256_file(Path(__file__)),'scope':'Actual five-session renders and original photographs. Inferred ceiling surfaces are absent, not merely hidden by a viewing toggle.'},indent=2)+'\n')


if __name__=='__main__':main()
