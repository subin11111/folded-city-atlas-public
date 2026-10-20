#!/usr/bin/env python3
"""Edit real prefix snapshots and historical offline evidence into an honest MP4.

No pose morphs, fake UI screenshots, synthetic sensor frames, or claim that the
historical final mesh was generated online. Recorded source time, compute time
and edited presentation time are distinct and labelled.
"""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from optimize_session import sha256_file, utc_now

ROOT=Path(__file__).resolve().parents[1]
NAMES={'outside':'외부','lobby':'1층 로비','bridge_out1f':'외부–로비 브리지',
       'floor2':'2층·메인 계단','bridge_12_sub':'서브 계단'}


def fit_image(path, size, background='#18212c'):
    im=Image.open(path).convert('RGB');im.thumbnail(size,getattr(Image,'Resampling',Image).LANCZOS)
    result=Image.new('RGB',size,background);result.paste(im,((size[0]-im.width)//2,(size[1]-im.height)//2))
    return result


def compact_processing_graph(inventory_path, output):
    """Layout-only derivative of actual DDS endpoints, not an rqt screenshot."""
    inventory=json.loads(inventory_path.read_text())['inventory']
    labels={'capture_db_replay':'Capture DB replay', 'rgbd_confidence_filter':'RGB-D confidence filter',
        'stored_loop_reverification':'Stored loop re-verification',
        'incremental_prefix_optimizer':'Incremental prefix optimizer',
        'prefix_cloud_assembler':'Prefix cloud assembler','evidence_recorder':'Evidence recorder',
        'offline_final_artifact_loader':'OFFLINE artifact loader',
        'final_reference_recorder':'Final reference recorder','rviz':'RViz'}
    rows=[r for r in inventory if r['name'] in labels]
    names={r['name'] for r in rows}
    if len(names)<9:raise ValueError('Actual eight processing nodes + RViz required')
    lines=['digraph DDS {rankdir=TB;bgcolor="#eef4f0";nodesep=.22;ranksep=.45;',
        'node [shape=box,style="rounded,filled",fillcolor="white",fontname="DejaVu Sans",fontsize=18,margin=".15,.12"];',
        'edge [color="#4c7468",arrowsize=.8];']
    for name in labels:
        lines.append(json.dumps(name)+' [label='+json.dumps(labels[name])+'];')
    for pub in rows:
        topics={t[0] for t in pub['publishers'] if t[0].startswith('/folded_city_demo/')}
        for sub in rows:
            shared=topics&{t[0] for t in sub['subscribers']}
            if shared:
                tail=' [constraint=false,style=dashed]' if shared=={'/folded_city_demo/ack'} else ''
                lines.append(json.dumps(pub['name'])+' -> '+json.dumps(sub['name'])+tail+';')
    lines.append('}')
    output.with_suffix('.dot').write_text('\n'.join(lines))
    subprocess.run(['dot','-Tpng',str(output.with_suffix('.dot')),'-o',str(output)],check=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--replay',type=Path,required=True)
    p.add_argument('--cartographic',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);(a.output/'stills').mkdir()
    events=json.loads((a.replay/'events.json').read_text());run=json.loads((a.replay/'manifest.json').read_text())
    inventory=a.replay/'graph_delivery/ros_graph_inventory.json'
    graph=a.output/'actual_dds_compact.png'
    compact_processing_graph(inventory,graph)
    import open3d as o3d
    renderer=o3d.visualization.rendering.OffscreenRenderer(1030,630)
    renderer.scene.set_background([.045,.06,.085,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',30)
    small=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',22)
    tiny=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',18)
    bold=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',40)
    fps=12;frames=0;chapters=[];sources={}
    command=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24',
        '-video_size','1920x1080','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast',
        '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output/'mapping_story.mp4')]
    encoder=subprocess.Popen(command,stdin=subprocess.PIPE)
    def emit(image,seconds):
        nonlocal frames
        for _ in range(round(seconds*fps)):
            encoder.stdin.write(np.asarray(image,np.uint8).tobytes());frames+=1
    def base(title,subtitle,offline=False):
        canvas=Image.new('RGB',(1920,1080),'#101923');d=ImageDraw.Draw(canvas)
        d.text((28,22),title,font=bold,fill='#edf6f2')
        d.text((30,80),subtitle,font=small,fill='#f5b867' if offline else '#8ddfc5')
        d.rectangle((0,1035,1920,1080),fill='#1b2b37')
        d.text((24,1046),'실제 기록 재생 + 실제 재계산 / 편집 영상 · 최종 v9는 별도 오프라인·수동 검토 결과',font=tiny,fill='#b9cad3')
        return canvas
    def evidence_card(title,subtitle,files,seconds):
        image=base(title,subtitle,True)
        width=(1860-20*(len(files)-1))//len(files)
        for i,path in enumerate(files):
            image.paste(fit_image(path,(width,870)),(30+i*(width+20),145));sources[str(path)]=sha256_file(path)
        chapters.append({'start_s':frames/fps,'title':title,'source_files':[str(p) for p in files],
                         'mode':'historical_offline_evidence_not_rerun'})
        emit(image,seconds)
    intro=base('Folded City · 기록에서 지도까지','기록 재생 / 인과적 prefix 최적화 / 오프라인 최종 마감의 구분')
    d=ImageDraw.Draw(intro)
    for y,line in enumerate(['① 폰 앱이 저장한 RGB-D·VIO를 촬영 순서대로 재생',
        '② 양 끝 프레임이 도착한 저장 루프 후보만 기하 재검증',
        '③ 실제 강건 4DoF 그래프 풀이 → 이전 점군을 새 포즈로 다시 투영',
        '④ 선택된 5개 세션 통합·표면/사진/수동 마감은 과거 오프라인 결과',
        '새 센서 실시간 촬영이나 최종 메시의 완전 자동 생성을 입증하는 영상은 아닙니다.']):
        d.text((85,240+y*115),line,font=font if y<4 else small,fill='#e2ece8' if y<4 else '#f5b867')
    emit(intro,6)
    evidence_card('실제로 실행한 ROS 처리 노드','rqt_graph 원본 창 · 메시지 연결 / SLAM 프레임 그래프와 별개',
        [a.replay/'graph_delivery/rqt_native_window.png'],6)
    final_per_session={r['session']:r for r in events}
    cameras={}
    for name,row in final_per_session.items():
        z=np.load(row['snapshot']);q=z['points'];low,high=np.percentile(q,[1,99],axis=0)
        centre=(low+high)/2;extent=max(float(np.linalg.norm(high-low)),8.)
        cameras[name]=(centre,extent)
    for event in events:
        z=np.load(event['snapshot']);name=event['session'];centre,extent=cameras[name]
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(z['points']))
        cloud.colors=o3d.utility.Vector3dVector(z['rgb']/255.)
        mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultUnlit';mat.point_size=2.
        renderer.scene.clear_geometry();renderer.scene.add_geometry('prefix',cloud,mat)
        renderer.setup_camera(52.,centre,centre+np.array([.70,-.90,.85])*extent*.55,[0,0,1])
        pixels=np.asarray(renderer.render_to_image()).copy()
        image=base(f'기록 재생 · {NAMES[name]}',
            f'현재까지 {event["seen_count"]:,} 프레임 · 원기록 {event["recorded_elapsed_s"]:.1f}초 · 40프레임 묶음 가속 재생')
        image.paste(Image.fromarray(pixels),(26,140))
        image.paste(fit_image(Path(event['photo']),(280,375)),(1080,140))
        image.paste(fit_image(graph,(520,875),'white'),(1380,140))
        ImageDraw.Draw(image).text((1400,145),'실제 DDS 연결 · 영상용 축약 배치',font=tiny,fill='#23463b')
        d=ImageDraw.Draw(image);d.text((1080,530),'현재 원본 RGB',font=small,fill='#c1d5cd')
        solver=event['solver'];status='최적화 갱신' if solver['applied'] else ('수렴 보류' if solver['ran'] else '관측 누적')
        lines=[status,f'검증 루프 {solver["verified_loops_available"]}개',
            f'풀이 {solver["seconds"]:.3f}초',f'최대 포즈 갱신',f'{solver["maximum_pose_update_m"]:.3f} m',
            '기존 포즈 초기값 사용','원본 DB 수정 없음','프레임별 그래프: 아래',
            'ROS 처리 연결: 오른쪽']
        for j,line in enumerate(lines):d.text((1080,585+j*42),line,font=small,fill='#8ddfc5' if j==0 else '#c4d0d7')
        # Actual current-frame graph, all arrived nodes only. No invented loop.
        raw=z['raw_poses'][:,:2,3];before=z['before_poses'][:,:2,3];after=z['poses'][:,:2,3]
        final=np.load(final_per_session[name]['snapshot']);allxy=final['raw_poses'][:,:2,3]
        lo,hi=allxy.min(0)-1,allxy.max(0)+1;scale=min(980/(hi-lo)[0],190/(hi-lo)[1]);c=(lo+hi)/2
        def xy(points):return [tuple(v) for v in ((points-c)*[scale,-scale]+[535,900])]
        d.rectangle((26,790,1056,1020),fill='#192733')
        for points,color in [(raw,'#687781'),(before,'#e6ac66'),(after,'#5ce5bc')]:
            if len(points)>1:d.line(xy(points),fill=color,width=2)
        lookup={int(n):i for i,n in enumerate(z['ids'])}
        for a_id,b_id in z['loops']:
            d.line(xy(after[[lookup[int(a_id)],lookup[int(b_id)]]]),fill='#eb6974',width=1)
        for pt in xy(after):d.ellipse((pt[0]-1,pt[1]-1,pt[0]+1,pt[1]+1),fill='#5ce5bc')
        d.text((40,797),'프레임 그래프 XY · 회색 VIO / 주황 갱신 전 / 초록 갱신 후 / 빨강 검증 루프',font=tiny,fill='#c6d8d2')
        if event['sequence']==0 or event['solver']['applied']:
            path=a.output/'stills'/f'{event["sequence"]:04d}_{name}.png';image.save(path)
        if event['sequence']==0 or events[event['sequence']-1]['session']!=name:
            chapters.append({'start_s':frames/fps,'title':f'기록 재생 {name}','mode':'actual_prefix_recalculation'})
        emit(image,1.5 if event['solver']['applied'] else .5)
        print(f'VIDEO_PREFIX {event["sequence"]+1}/{len(events)}',flush=True)
    # A hard chapter boundary; the selected final poses do not magically emerge
    # from this intentionally simpler replay backend.
    transition=base('이후는 별도의 오프라인 처리 이력입니다','새 prefix 풀이 결과 ≠ 사용자가 선택한 최종 포즈 / 실시간 자동 연결로 가장하지 않음',True)
    d=ImageDraw.Draw(transition)
    for j,line in enumerate(['세션별 전처리·다중 후보 비교 → 사용자 선택',
        '공통 특징·원본 RGB-D로 세션 간 연결 검토',
        '선택 포즈로 점군 재생성 → 표면·사진 텍스처',
        '층 구조·개방 로비 보존 → 간판·재질·중복 메시 마감']):
        d.text((80,230+j*150),line,font=font,fill='#e0ebe7')
    emit(transition,6)
    evidence_card('오프라인 · 선택한 다섯 세션 통합','J01 / N05 / H03 / J04 / H03 · 강체 배치와 검증 한계를 포함한 실제 과거 출력',
        [ROOT/'docs/assets/selected-map-review/selected_map_before_after.png'],6)
    evidence_card('오프라인 · 점군에서 표면으로','실제 표면 비교 자료 · 연속화가 형상을 바꾼 실패와 대안도 보존',
        [ROOT/'docs/assets/surface-continuity-review/floor2/oblique_comparison.png'],6)
    evidence_card('오프라인 · 층 연결과 개방 로비','확인된 층 구조와 사진 지지 표면 · 추론 표면은 길찾기 바닥이 아님',
        [ROOT/'docs/assets/unified-presentation-20261018/lobby_516_presentation.png'],6)
    evidence_card('오프라인 · 색감 보존과 메시 마감','최종 v9 실제 전후 · 원본 사진 / 이전 / 최종 / 기하',
        [ROOT/'docs/assets/final-mesh-cleanup-20261019/lobby_445_presentation.png'],7)
    evidence_card('최종 지도 · student_union_final_mesh_v9','사용자 확정 버전 · 원본 표지·계단·층 구분 유지 / 안전한 길찾기 승인은 별도',
        [ROOT/'docs/assets/final-mesh-cleanup-20261019/native_scene_render.png'],7)
    evidence_card('일반 지도 레이어 · 길 / 건물 / 층별 실내 영역','OSM 실제 길·건물 101개 + 실내 표시 범위 · 임시 위치 정합 / 경로는 아직 미검증',
        [a.cartographic/'campus_f1.png',a.cartographic/'indoor_f2.png'],6)
    encoder.stdin.close()
    if encoder.wait()!=0:raise RuntimeError('Video encoding failed')
    probe=json.loads(subprocess.check_output(['ffprobe','-v','quiet','-print_format','json','-show_streams','-show_format',str(a.output/'mapping_story.mp4')]))
    source_manifest=a.replay/'manifest.json'
    result={'created_utc':utc_now(),'video_sha256':sha256_file(a.output/'mapping_story.mp4'),
        'dimensions':[1920,1080],'fps':fps,'frames':frames,'duration_seconds':frames/fps,
        'chapters':chapters,'historical_source_sha256':sources,
        'replay_manifest_sha256':sha256_file(source_manifest),'replay_wall_seconds':run['wall_seconds'],
        'contains_synthetic_sensor_data':False,'pose_morphing':False,'final_mesh_generated_online':False,
        'video_is_edited_accelerated_replay_not_real_time_benchmark':True,
        'loop_proposals_historical_not_new_place_recognition':True,
        'ffprobe':probe,'renderer_sha256':sha256_file(Path(__file__))}
    (a.output/'manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('VIDEO_COMPLETE '+str(a.output/'mapping_story.mp4'),flush=True)


if __name__=='__main__':main()
