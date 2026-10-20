#!/usr/bin/env python3
"""Save measured seam summaries and real open/closed elevator photographs."""
import argparse,json,sqlite3
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from export_rtabmap_db import load_pose_csv
from surface_model_utils import decode_frame
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);a=p.parse_args();a.assets.mkdir(exist_ok=True,parents=True)
    run=json.loads((a.bundle/'manifest.json').read_text());objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    lighting={};font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',24)
    small=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',19)
    image=Image.new('RGB',(1200,570),'white');draw=ImageDraw.Draw(image)
    draw.text((25,20),'사진 경계 밝기 보정: 같은 경계 표본의 로그 휘도 차이',font=font,fill='#172538')
    draw.text((25,58),'학습에 사용한 경계의 자체 일관성 지표 · 독립 조명/측량 정확도가 아님',font=small,fill='#596475')
    for i,name in enumerate(['floor2','lobby']):
        charts=json.loads((a.bundle/name/'boundary_leveling.json').read_text());measured=[v for v in charts if v.get('boundaries')]
        before=float(np.median([v['median_absolute_boundary_jump_before'] for v in measured]));after=float(np.median([v['median_absolute_boundary_jump_after'] for v in measured]))
        lighting[name]={'all_charts':len(charts),'charts_with_accepted_seam_samples':len(measured),'adjusted_charts':sum(v.get('applied',False) for v in charts),
            'rejected_worsening_charts':sum(v.get('candidate_median_absolute_boundary_jump_after',0)>v.get('median_absolute_boundary_jump_before',0) for v in measured),
            'median_chart_absolute_log_luminance_jump_before':before,'median_chart_absolute_log_luminance_jump_after':after,
            'relative_reduction':1-after/before,'same_samples_self_consistency_not_independent_accuracy':True}
        x=45+i*595;draw.text((x,108),name,font=font,fill='#172538')
        for j,(label,value,colour) in enumerate([('이전',before,'#d77a45'),('보정 후',after,'#2c8c98')]):
            y=175+j*100;draw.text((x,y),label,font=small,fill='#172538');draw.rectangle((x+90,y,x+90+int(value/.045*395),y+43),fill=colour)
            draw.text((x+95,y+48),f'{value:.5f}',font=small,fill='#172538')
        draw.text((x,397),f'경계 표본 차이 중앙값 {100*(1-after/before):.1f}% 감소',font=small,fill='#172538')
        draw.text((x,436),f'악화 후보 {lighting[name]["rejected_worsening_charts"]}개는 적용하지 않음',font=small,fill='#596475')
    draw.text((25,520),'색상·글자를 평균내지 않고, 저텍스처 경계의 노출 gain만 조정. 반사·실제 그림자는 남음.',font=small,fill='#596475')
    chart=a.assets/'brightness_boundary_comparison.png';image.save(chart)
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());s=selected['sessions']['lobby'];poses=load_pose_csv(mp.parent/s['trajectory'])
    con=sqlite3.connect(f'file:{selected["database_integrity"]["lobby"]["database"]}?mode=ro',uri=True)
    sheet=Image.new('RGB',(1000,765),'white');d=ImageDraw.Draw(sheet)
    d.text((15,12),'같은 엘리베이터: 열린 관측과 닫힌 관측을 따로 기록',font=font,fill='#172538')
    for i,(node,label) in enumerate([(433,'#433 실제 열린 문'),(485,'#485 실제 닫힌 문 · 표시 기준')]):
        frame=decode_frame(con,node,s['preprocessing'],poses[node]);photo=Image.fromarray(frame['rgb']);photo.thumbnail((480,640))
        d.text((15+500*i,56),label,font=small,fill='#172538');sheet.paste(photo,(10+500*i,90))
    d.text((15,739),'닫힌 문 사진 표면은 영구 벽/통행 금지 판정이 아님. 원본 RGB만 사용.',font=small,fill='#596475')
    elevator=a.assets/'elevator_open_closed_originals.png';sheet.save(elevator);con.close()
    summary={'created_utc':utc_now(),'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'reviewed_objects':len(objects),'classes':dict(Counter(o['class'] for o in objects)),
        'added_photo_surfaces':sum(o.get('surface_added',False) for o in objects),
        'sum_of_added_surface_areas_m2_not_net_gap_area':sum(o['geometry']['area_m2'] for o in objects if o.get('surface_added')),
        'lighting':lighting,'reviewed_sightline_carved_source_faces_by_object':{o['id']:o['reviewed_sightline_carved_source_faces'] for o in objects if o.get('reviewed_sightline_carved_source_faces')},
        'figures':[{'file':f.name,'sha256':sha256_file(f)} for f in [chart,elevator]],'source_sha256':sha256_file(Path(__file__))}
    (a.assets/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
