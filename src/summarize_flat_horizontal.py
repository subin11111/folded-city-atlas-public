#!/usr/bin/env python3
"""Small evidence maps separating measured support, inferred fill and openings."""
import argparse,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);a=p.parse_args();a.assets.mkdir(parents=True,exist_ok=True)
    run=json.loads((a.bundle/'manifest.json').read_text());totals=defaultdict(float);sessions={};figures=[]
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',20)
    for name,s in run['sessions'].items():
        planes=[v for v in s['flat_sheets'] if v['accepted']]
        sessions[name]={'planes':len(planes),'floor_area_xy_m2':sum(v['area_xy_m2'] for v in planes if v['kind']=='floor'),
            'ceiling_area_xy_m2':sum(v['area_xy_m2'] for v in planes if v['kind']=='ceiling'),
            'modeled_inferred_area_m2':sum(v['model_inferred_area_m2'] for v in planes),
            'replaced_source_faces':s['flat_replaced_source_faces'],'protected_source_faces':s['flat_protected_source_faces'],
            'stair_guard_kind':s['stair_guard_kind'],'levels':[{k:v[k] for k in ['id','kind','height_m','area_xy_m2','model_inferred_area_m2','photo_fraction_on_sheet','source_height_deviation_p90_m']} for v in planes]}
        for key in ['floor_area_xy_m2','ceiling_area_xy_m2','modeled_inferred_area_m2']:totals[key]+=sessions[name][key]
        for plane in planes:
            if name not in ['floor2','lobby'] or plane['id'].endswith('1f') and name=='floor2':continue
            m=np.load(a.bundle/name/plane['mask_file']);mask=m['mask'];rgb=np.full((*mask.shape,3),245,np.uint8)
            rgb[m['observed']]=[155,160,166];rgb[mask&~m['observed']]=[43,165,105];rgb[m['free_guard']]=[50,135,220];rgb[m['stair_guard']]=[210,90,65]
            image=Image.fromarray(np.flipud(rgb));image.thumbnail((700,650));sheet=Image.new('RGB',(1000,790),'white');draw=ImageDraw.Draw(sheet)
            draw.text((15,12),name+' / '+plane['id']+f' | Z={plane["height_m"]:.3f}m · 평면 {plane["area_xy_m2"]:.1f}m²',font=font,fill='#172538')
            draw.text((15,48),'회색: 기존 수평 지지 / 초록: 모델로 채움 / 파랑: 관측 빈 공간 / 빨강: 계단 보호',font=font,fill='#172538')
            sheet.paste(image,((1000-image.width)//2,95));draw.text((15,748),'추정 면은 실측 바닥/천장 확정이나 길찾기 승인 결과가 아님.',font=font,fill='#596475')
            path=a.assets/(plane['id']+'_support_map.png');sheet.save(path);figures.append({'file':path.name,'sha256':sha256_file(path),'plane':plane['id']})
    summary={'created_utc':utc_now(),'sessions':sessions,'totals':dict(totals),'sheets':sum(s['planes'] for s in sessions.values()),
        'area_scope':'Sum of separate session floor/ceiling sheets; overlaps and two sides counted, not net building area.',
        'model_scope':'Fixed horizontal display modeling, not new pose optimization or surveyed accuracy.',
        'figures':figures,'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'source_sha256':sha256_file(Path(__file__))}
    (a.assets/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary['totals']),flush=True)


if __name__=='__main__':main()
