#!/usr/bin/env python3
"""OSM streets/buildings + separately labelled indoor display polygons.

Local metre XY is stored as custom vector JSON, never falsely called WGS84
GeoJSON. Indoor geographic features are explicitly coarse GNSS/manual preview.
No captured trajectory becomes a road or an approved route. No OSM writes.
"""
import argparse
import html
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.parse import urlencode

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from audit_sessions import horizontal_enu
from optimize_session import sha256_file, utc_now
from prepare_roadview_pack import enu_to_lonlat

ROOT=Path(__file__).resolve().parents[1]


def mask_rings(mask, low, cell, simplify=.10):
    contours,hierarchy=cv2.findContours(np.uint8(mask),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None: return []
    rings=[]
    for i,c in enumerate(contours):
        if abs(cv2.contourArea(c))*cell*cell < .06: continue
        q=cv2.approxPolyDP(c,simplify/cell,True).reshape(-1,2)
        if len(q)<3: continue
        # Raster contour follows supported cell centres, not surveyed walls.
        xy=np.asarray(low)+(q+.5)*cell
        depth=0;parent=int(hierarchy[0,i,3])
        while parent>=0:
            depth+=1;parent=int(hierarchy[0,parent,3])
        rings.append({'xy_m':np.r_[xy,xy[:1]].tolist(),'hole':bool(depth%2)})
    return rings


def to_geo_polygons(rings, convert):
    # Use containment to attach preserved holes to their nearest outer ring.
    outers=[r for r in rings if not r['hole']];result=[]
    for outer in outers:
        ring=np.array(outer['xy_m'],np.float32)
        holes=[]
        for h in rings:
            if h['hole'] and cv2.pointPolygonTest(ring,tuple(h['xy_m'][0]),False)>=0:
                # Nested islands get their own outer feature; normal floor masks
                # have one outer/hole level. Reject ambiguous nesting below.
                containers=[o for o in outers if cv2.pointPolygonTest(np.array(o['xy_m'],np.float32),tuple(h['xy_m'][0]),False)>=0]
                smallest=min(containers,key=lambda o:abs(cv2.contourArea(np.array(o['xy_m'],np.float32))))
                if smallest is outer: holes.append(h)
        result.append([convert(outer['xy_m']),*[convert(h['xy_m']) for h in holes]])
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--osm-cache',type=Path);p.add_argument('--osm-xml-cache',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    pointer=json.loads((ROOT/'config/current_local_view.json').read_text());bundle=(ROOT/pointer['structural_manifest']).parent
    georef=json.loads((ROOT/'outputs/student_union_roadview_v1/georeference.json').read_text())
    manual=json.loads((ROOT/'config/osm_manual_preview.json').read_text())
    origin=(georef['origin_latitude_deg'],georef['origin_longitude_deg'])
    rotation=np.array(georef['R_map_xy_to_EN']);translation=np.array(georef['t_map_xy_to_EN_m'])
    offset=np.array([manual['east_m'],manual['north_m']])
    query=f'[out:json][timeout:25];(way["building"](around:180,{origin[0]},{origin[1]});way["highway"](around:180,{origin[0]},{origin[1]}););out geom;'
    source=a.output/'osm_source.json'
    if a.osm_xml_cache:
        root=ET.fromstring(a.osm_xml_cache.read_bytes())
        osm_nodes={n.attrib['id']:{'lat':float(n.attrib['lat']),'lon':float(n.attrib['lon'])} for n in root.findall('node')}
        elements=[]
        for way in root.findall('way'):
            tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')}
            if not ('building' in tags or 'highway' in tags):continue
            refs=[n.attrib['ref'] for n in way.findall('nd')]
            if not all(n in osm_nodes for n in refs):continue
            elements.append({'type':'way','id':int(way.attrib['id']),'tags':tags,'geometry':[osm_nodes[n] for n in refs]})
        source.write_text(json.dumps({'elements':elements,'source_xml_sha256':sha256_file(a.osm_xml_cache)}))
        (a.output/'osm_source.xml').write_bytes(a.osm_xml_cache.read_bytes())
    elif a.osm_cache:
        source.write_bytes(a.osm_cache.read_bytes())
    else:
        req=Request('https://overpass-api.de/api/interpreter',data=urlencode({'data':query}).encode(),
                    headers={'User-Agent':'FoldedCityAtlas/0.1 (single user-requested campus vector view)'})
        with urlopen(req,timeout=35) as response: source.write_bytes(response.read())
    osm=json.loads(source.read_text());features=[];local_osm=[]
    for element in osm.get('elements',[]):
        geometry=element.get('geometry',[]);tags=element.get('tags',{})
        if not geometry: continue
        coordinates=[[g['lon'],g['lat']] for g in geometry]
        building='building' in tags and coordinates[0]==coordinates[-1]
        if 'building' in tags and not building: continue
        feature={'type':'Feature','id':f'osm_way_{element["id"]}',
            'properties':{'source':'OpenStreetMap','osm_type':'way','osm_id':element['id'],
                'feature_type':'building' if building else 'street','tags':tags,'routing_approved':False},
            'geometry':{'type':'Polygon' if building else 'LineString','coordinates':[coordinates] if building else coordinates}}
        features.append(feature)
        en=np.array([horizontal_enu(lat,lon,origin)[:2] for lon,lat in coordinates])
        xy=(en-translation-offset)@rotation
        local_osm.append({'id':feature['id'],'kind':feature['properties']['feature_type'],
            'name':tags.get('name',''),'highway':tags.get('highway'),'xy_m':xy.tolist()})
    (a.output/'campus_osm.geojson').write_text(json.dumps({'type':'FeatureCollection','features':features},ensure_ascii=False)+'\n')
    levels=[];geo=[]
    def convert(xy):
        en=np.asarray(xy)@rotation.T+translation+offset
        lon,lat=enu_to_lonlat(en[:,0],en[:,1],origin)
        return np.c_[lon,lat].tolist()
    for level in (1,2):
        file=bundle/f'main_{level}f_floor_mask.npz';z=np.load(file)
        allowed=z['allowed'].astype(bool)&~z['forbidden'].astype(bool)
        rings=mask_rings(allowed,z['low'],float(z['cell']))
        coverage=mask_rings(z['coverage'].astype(bool)&~z['forbidden'].astype(bool),z['low'],float(z['cell']))
        row={'level_id':f'student_union_1_f{level}','label':f'{level}층','rings':rings,
            'observed_support_rings':coverage,'display_height_m':float(z['height']),
            'role':'display_floor_envelope_NOT_room_boundary_or_walkability',
            'source_mask':str(file.relative_to(ROOT)),'source_sha256':sha256_file(file),
            'routing_approved':False}
        levels.append(row)
        geo.append({'type':'Feature','properties':{'level_id':row['level_id'],'name':f'제1학생회관 {level}층 촬영 영역',
            'kind':'indoor_display_extent','georeference':'coarse_GNSS_plus_unmeasured_2m_E_2m_S_preview',
            'routing_approved':False,'contains_inferred_display_completion':True},
            'geometry':{'type':'MultiPolygon','coordinates':to_geo_polygons(rings,convert)}})
    (a.output/'indoor_coarse.geojson').write_text(json.dumps({'type':'FeatureCollection','features':geo},ensure_ascii=False)+'\n')
    vector={'frame':'selected map XY metres; display Z is not routing height','osm':local_osm,'levels':levels,
        'place_features':[],'verified_route_nodes':[],'verified_route_edges':[],
        'note':'No room boundaries, place names or usable routing connections have been invented.'}
    (a.output/'map_vectors.json').write_text(json.dumps(vector,ensure_ascii=False,indent=2)+'\n')
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',23)
    small=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',17)
    def render(level,overview=False):
        selected=levels[level-1];allxy=np.concatenate([np.array(r['xy_m']) for r in selected['rings']])
        low,high=allxy.min(0)-5,allxy.max(0)+5
        if overview:
            c=(low+high)/2;low=c-115;high=c+115
        scale=min(1100/(high-low)[0],950/(high-low)[1]);centre=(low+high)/2
        if overview:centre=rotation@centre+translation+offset
        def pixels(xy):
            coordinates=np.asarray(xy)
            if overview:coordinates=coordinates@rotation.T+translation+offset
            q=(coordinates-centre)*[scale,-scale]+[600,580]
            return [tuple(map(float,p)) for p in q]
        canvas=Image.new('RGB',(1200,1200),'#f4f4ef');draw=ImageDraw.Draw(canvas)
        for item in local_osm:
            pts=pixels(item['xy_m'])
            if item['kind']=='building':draw.polygon(pts,fill='#d6d8d2',outline='#aeb6b0')
            else:
                width=5 if item['highway'] in ('footway','path','steps','pedestrian') else 12
                draw.line(pts,fill='#c7d0c7',width=width+3);draw.line(pts,fill='white',width=width)
        # Separate mask maintains holes, including central atrium at upper level.
        floor_mask=Image.new('L',canvas.size,0);pen=ImageDraw.Draw(floor_mask)
        for ring in sorted(selected['rings'],key=lambda r:r['hole']):
            pen.polygon(pixels(ring['xy_m']),fill=0 if ring['hole'] else 255)
        canvas.paste(Image.new('RGB',canvas.size,'#d6eee6'),mask=floor_mask);draw=ImageDraw.Draw(canvas)
        for ring in selected['rings']:draw.line(pixels(ring['xy_m']),fill='#3e8e7b',width=2)
        for item in local_osm:
            if item['name']:
                q=np.mean(pixels(item['xy_m']),axis=0)
                if 40<q[0]<1120 and 155<q[1]<1090:draw.text(tuple(q),item['name'][:18],font=small,fill='#535c55',anchor='mm')
        draw.rectangle((0,0,1200,140),fill='white')
        draw.text((30,23),f'제1학생회관 · {selected["label"]} / '+('길·건물 지도' if overview else '실내 영역 지도'),font=font,fill='#173e34')
        draw.text((30,68),'회색: OSM 건물 · 흰색: OSM 길 · 초록: 현재 실내 표시 영역',font=small,fill='#42564e')
        draw.text((30,101),'GNSS 임시 배치 · 호실/문/보행 연결 미등록 · 촬영 궤적을 길로 쓰지 않음',font=small,fill='#a46426')
        if overview:draw.text((1110,160),'↑ N',font=font,fill='#23463b')
        draw.rectangle((0,1110,1200,1200),fill='white')
        draw.text((24,1125),'© OpenStreetMap contributors · openstreetmap.org/copyright',font=small,fill='#42564e')
        draw.text((24,1155),'실내 폴리곤은 보기용 범위이며 건물 외벽 실측/통행 가능 영역이 아닙니다.',font=small,fill='#7b6653')
        file=a.output/(f'campus_f{level}.png' if overview else f'indoor_f{level}.png');canvas.save(file)
        # Pure SVG coordinates remain editable, retaining original polygons.
        path=[]
        for ring in selected['rings']:
            pts=pixels(ring['xy_m']);path.append('M '+' L '.join(f'{x:.2f},{y:.2f}' for x,y in pts)+' Z')
        svg=f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 1200"><rect width="1200" height="1200" fill="#f4f4ef"/><path d="{" ".join(path)}" fill="#d6eee6" stroke="#3e8e7b" stroke-width="2" fill-rule="evenodd"/><text x="30" y="50" font-size="25">{html.escape(selected["label"])} — display extent, not walkability</text></svg>'
        if not overview:(a.output/f'indoor_f{level}.svg').write_text(svg)
    for level in (1,2):render(level);render(level,True)
    index='''<!doctype html><meta charset="utf-8"><title>Folded City · 길/건물/실내 지도</title><style>body{margin:30px;font-family:system-ui;background:#f4f4ef;color:#23463b}button{padding:12px;margin:5px}img{max-width:90vw;max-height:80vh}p{color:#7b6653}</style><h1>제1학생회관</h1><p>길·건물은 OSM, 실내는 현재 표시 지도에서 분리한 영역입니다. 실내 배치는 임시이며 길찾기는 아직 승인되지 않았습니다.</p><button onclick="show('campus_f1.png')">주변 길·건물</button><button onclick="show('indoor_f1.png')">1층</button><button onclick="show('indoor_f2.png')">2층</button><br><img id="map" src="campus_f1.png" alt="길·건물·실내 표시 지도"><script>function show(p){document.getElementById('map').src=p}</script>'''
    (a.output/'index.html').write_text(index)
    report={'created_utc':utc_now(),'selected_display_manifest_sha256':pointer['structural_manifest_sha256'],
        'OSM_source_sha256':sha256_file(source),'OSM_query_template':query,
        'OSM_query_executed_by_this_run':not bool(a.osm_cache or a.osm_xml_cache),
        'OSM_source_xml_sha256':sha256_file(a.osm_xml_cache) if a.osm_xml_cache else None,
        'OSM_features':len(features),
        'OSM_attribution':'© OpenStreetMap contributors','OSM_licence':'https://www.openstreetmap.org/copyright',
        'source_licence':'OSM data ODbL; derivative indoor data retained separately',
        'georeference':georef,'manual_preview_offset':manual,'OSM_edits_or_uploads':False,
        'routing_ready':False,'coordinate_units':'map_vectors.json metres; *.geojson longitude/latitude WGS84',
        'levels':[{k:v for k,v in l.items() if k not in ('rings','observed_support_rings')} for l in levels],
        'code_sha256':sha256_file(Path(__file__))}
    (a.output/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(a.output),'OSM_features':len(features),'levels':2,'routing_ready':False}),flush=True)


if __name__=='__main__':main()
