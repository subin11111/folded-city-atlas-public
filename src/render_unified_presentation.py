#!/usr/bin/env python3
"""Original photos and actual full-map before/after presentation renders."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from optimize_session import inverse,sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);p.add_argument('--views',help='Comma-separated session:node:pitch selections');a=p.parse_args();a.assets.mkdir(parents=True,exist_ok=False);run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run.get('previous_material_manifest',run['base_display_manifest'])).parent;road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',22)
    views=[]
    requested=[('lobby',713,18),('lobby',579,10),('lobby',167,8),('floor2',728,0),('floor2',726,0),('lobby',516,15)]
    if a.views:requested=[(s,int(n),float(p)) for s,n,p in [v.split(':') for v in a.views.split(',')]]
    for name,n,pitch in requested:
        v=min([v for v in nodes if v['session']==name],key=lambda v:abs(v['node']-n));pose=np.asarray(v['world_optical']).copy()
        if run.get('architectural_presentation'):
            from presentation_geometry import LevelPresentation
            pose[:3,3]=LevelPresentation(ROOT,run['presentation_config']).warp(pose[:3,3][None],name)[0]
        pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',pitch,degrees=True).as_matrix();k=np.asarray(v['K']).copy();k[:2]*=800/960;k[0,2]+=(800-720*800/960)/2;views.append({'session':name,'node':v['node'],'source_photo':v['photo'],'source_world_optical':v['world_optical'],'world_optical':pose.tolist(),'virtual_display_camera_z_regularized':bool(run.get('architectural_presentation')),'K':k.tolist(),'additional_pitch_deg':pitch})
    import open3d as o3d
    r=o3d.visualization.rendering.OffscreenRenderer(800,800);r.scene.set_background([.055,.065,.08,1.]);r.scene.view.set_antialiasing(True);r.scene.view.set_sample_count(4);r.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR));panels=[[] for _ in views]
    for mode in ['before','after','neutral']:
        r.scene.clear_geometry()
        for name in run['sessions']:
            folder=(base if mode=='before' else a.bundle)/name
            if mode=='neutral':
                mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'));mesh.compute_vertex_normals();mesh.paint_uniform_color([.6,.6,.6]);mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';r.scene.add_geometry(name,mesh,mat)
            else:
                model=o3d.io.read_triangle_model(str(folder/'clean_textured.glb'))
                for mat in model.materials:mat.shader='defaultUnlit'
                r.scene.add_model(name,model)
        for i,v in enumerate(views):
            r.setup_camera(np.asarray(v['K']),inverse(np.asarray(v['world_optical'])),800,800);panels[i].append(np.asarray(r.render_to_image()).copy())
        print('Unified presentation render: '+mode,flush=True)
    figures=[]
    for v,images in zip(views,panels):
        photo=Image.open(road/v['source_photo']).convert('RGB');photo.thumbnail((800,800));panel=Image.new('RGB',(800,800),'#e9edf0');panel.paste(photo,((800-photo.width)//2,(800-photo.height)//2));sheet=Image.new('RGB',(3200,900),'white');draw=ImageDraw.Draw(sheet);draw.text((12,10),f'{v["session"]}:{v["node"]} | 1·2층 통합 표시 모델 · 원본 지도 별도 보존',font=font,fill='#172538');labels=[f'원본 사진 (렌더만 위로 {v["additional_pitch_deg"]}도)','이전: 끊긴 층 경계','통합: 연속 벽띠 + 확인된 천장만','통합 모델의 실제 기하']
        if run.get('service_slab_completion'):labels[1:3]=['이전: 분리된 천장·바닥 / 표지 누락','이후: 경계 안 닫힌 층 구조 + 원본 표지']
        if run.get('horizontal_continuity_completion'):labels[1:3]=['이전: 천장 파편 / 바닥 중복·색 조각','이후: 연속 천장 / 바닥 면·재질 정리']
        if run.get('floor_photo_preservation'):labels[1:3]=['철회: 단색화한 바닥 재질','수정: 사진 기반 바닥 무늬 복원 / 기하 동일']
        if run.get('photo_seam_reconstruction'):labels[1:3]=['이전: 사진 경계·질감 결손','이후: 원본 RGB-D 사진 확대 / 경계 블렌딩']
        if run.get('photo_sign_finish'):labels[1:3]=['이전: 사진 경계 / PRIME HALL 글자 결손','이후: 사진 경계 블렌딩 + 원본 간판 전체']
        if run.get('visual_floor_illumination'):labels[1:3]=['이전: 심한 반사광·그림자·밝기 경계','시각 검토 후보: 무늬 보존 / 국소 조명 보정']
        if run.get('intrinsic_floor_material'):labels[1:3]=['이전: 조명 자국이 남은 사진 바닥','시각 검토 후보: 원본 타일 재질 / 조명 분리']
        if run.get('modeled_floor_material'):labels[1:3]=['이전: 사진에 구워진 조명 경계','보기용 재질 후보: 돌 질감 + 관측된 타일 특징']
        if run.get('edge_finish'):labels[1:3]=['이전: 사용자 선호 v15 / 뾰족한 경계','이후: 색감 유지 / 작은 외곽 톱니·사진 경계만 마감']
        if run.get('lower_ceiling_sign_clearance'):labels[1:3]=['이전: 낮은 천장 면이 간판 상단을 가림','이후: 확인된 간섭 천장만 올림 / 간판·2층 바닥 고정']
        if run.get('ceiling_expansion_and_closure'):labels[1:3]=['이전: 천장 결손 / 열린 층간 설비 공간','이후: 천장 확장 + 설비 공간 둘레 폐쇄 / 층 높이 유지']
        if run.get('ceiling_material_continuity'):labels[1:3]=['이전: 메운 면의 회색·질감 단절','이후: 실제 저장된 색·주변 질감 연결 / 기하 동일']
        if run.get('shared_contour_finish'):labels[1:3]=['이전: 사용자 승인 재질 / 지그재그 외곽','이후: 접합 경계를 함께 매끈하게 / 색감·층 높이 유지']
        if run.get('shadow_lift'):labels[1:3]=['이전: 사진에 구워진 짙은 암부','이후: 표면 암부만 밝게 / 빈 공간·형상 그대로']
        if run.get('shadow_lift',{}).get('config',{}).get('method')=='colour_preserving_denoised':labels[1:3]=['기준 v5: 기존 색감 / 어두운 표면','수정 후보 v3: 색 유지 / 약한 암부 보정']
        if run.get('final_mesh_finish'):labels[1:3]=['정리 전: 부유 파편·겹친 옛 면','정리 후: 색 유지 / 중복·파편·미세 경계 정리']
        for i,(im,label) in enumerate(zip([np.asarray(panel),*images],labels)):draw.text((800*i+12,55),label,font=font,fill='#172538');sheet.paste(Image.fromarray(im),(800*i,100))
        file=f'{v["session"]}_{v["node"]}_presentation.png';sheet.save(a.assets/file)
        Image.fromarray(images[0]).save(a.assets/f'{v["session"]}_{v["node"]}_before.png');Image.fromarray(images[1]).save(a.assets/f'{v["session"]}_{v["node"]}_after.png')
        figures.append({'file':file,'sha256':sha256_file(a.assets/file),'camera':v,'source_rgb_sha256':sha256_file(road/v['source_photo'])})
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'before_manifest':str((base/'manifest.json').resolve()),'before_manifest_sha256':sha256_file(base/'manifest.json'),'renderer_sha256':sha256_file(Path(__file__)),'scope':'Actual full-map GLB/PLY renders and unchanged original perspective photos. Architectural display completion, not surveyed pose accuracy.'},indent=2)+'\n')


if __name__=='__main__':main()
