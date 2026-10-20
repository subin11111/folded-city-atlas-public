"""사진 검색은 화면 표시용이다. 사진의 시간 순서로 경로를 만들지 않는다."""
import array
import base64
from collections import defaultdict
from functools import lru_cache
import hashlib
import json
import math
import sys
import zlib
from navigation import ROOT,load_navigation

def wrap(angle):return (angle+math.pi)%(2*math.pi)-math.pi

@lru_cache(maxsize=1)
def load_library():
    folder=ROOT/'public/data';regions=json.loads((folder/'photo_regions.json').read_text('utf-8'))
    if hashlib.sha256((folder/'navigation.json').read_bytes()).hexdigest()!=regions['navigation_sha256']:
        raise ValueError('보행 지도가 변경되었습니다. 사진 위치 대응을 다시 생성해야 합니다.')
    records=json.loads((folder/'photo_library.json').read_text('utf-8'));groups=defaultdict(list);arrays={}
    for lv,blob in regions['levels'].items():
        values=array.array('H');values.frombytes(zlib.decompress(base64.b64decode(blob)))
        if sys.byteorder!='little':values.byteswap()
        arrays[lv]=values
    for p in records:
        if p.get('navigation_eligible'):groups[p['region']].append(p)
    return records,groups,arrays

def region(node):
    _,_,arrays=load_library();lv,r,c=node
    if not load_navigation().free(*node):return 0
    return arrays[lv][r*load_navigation().cols+c]

@lru_cache(maxsize=16384)
def path_distance(a,b):
    if a==b:return 0.
    found=load_navigation().search(a,b,avoid_stairs=True)
    return found['distance'] if found else math.inf

def result(photo,distance,angle,context='floor'):
    return {'id':photo['id'],'url':'/'+photo['url'],'display_url':'/'+photo['display_url'] if photo.get('display_url') else None,'thumb_url':'/'+photo['thumb_url'],
        'position':photo['position'],'heading':photo['heading'],'pitch':photo['pitch'],
        'distance_m':round(distance,2),'heading_delta_deg':round(math.degrees(abs(angle))),
        'quality_score':photo['quality_score'],'quality_flags':photo['quality_flags'],
        'source_sha256':photo['sha256'],'session':photo['session'],'node':photo['node'],
        'context':context,'is_panorama':False,'is_exact_virtual_view':False,
        'position_match':'nearby original perspective image, not a synthesized view'}

def match(node,yaw,allow_turn=False):
    _,groups,_=load_library();p=load_navigation().position(node);candidates=[]
    for photo in groups.get(region(node),[]):
        distance=math.dist(photo['position'][:2],p[:2]);angle=wrap(photo['heading']-yaw)
        if distance>5 or (not allow_turn and abs(angle)>math.radians(55)):continue
        score=distance+.9*(1-photo['quality_score'])+.3*abs(photo['pitch'])
        if not allow_turn:score+=abs(angle)*1.6
        candidates.append((score,photo,distance,angle))
    for _,photo,distance,angle in sorted(candidates,key=lambda v:v[0])[:24]:
        # 연결 영역과 최단 보행 거리 모두 검사한다. 문 반대편/벽 너머의 사진은 제외.
        length=path_distance(node,tuple(photo['nav_node']))
        if length<=distance*1.35+1.2:return result(photo,distance,angle)
    return None

def stair_match(position,yaw,allow_turn=False):
    records,_,_=load_library();eye=[*position[:2],position[2]+1.55];candidates=[]
    for photo in records:
        if not photo.get('pose_available') or not photo['usable_exposure'] or 'heading' not in photo:continue
        if photo['session'] not in ('floor2','bridge_12_sub'):continue
        height=abs(photo['position'][2]-eye[2]);distance=math.dist(photo['position'],eye);angle=wrap(photo['heading']-yaw)
        if height>.8 or distance>3 or abs(photo['pitch'])>math.radians(75):continue
        if not allow_turn and abs(angle)>math.radians(65):continue
        score=distance+.8*(1-photo['quality_score'])+(0 if allow_turn else abs(angle)*1.4)
        candidates.append((score,photo,distance,angle))
    if not candidates:return None
    _,photo,distance,angle=min(candidates,key=lambda v:v[0]);return result(photo,distance,angle,'stairs')
