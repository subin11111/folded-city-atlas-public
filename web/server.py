"""Folded City 로컬 웹과 SQLite 장소/경로 API. 실행에는 표준 Python만 필요하다."""
from __future__ import annotations
import argparse
import heapq
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sqlite3
from navigation import load_navigation
import walk
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parent
PUBLIC = ROOT / "public"


def find_route(start, end, mode="demo", avoid_stairs=False, database=None, policy='shortest'):
    """촬영 기록이 아닌 공간 격자에서 최단 경로를 계산한다."""
    if mode not in ("demo", "verified"):
        raise ValueError("지원하지 않는 경로 모드입니다.")
    places={p['id']:p for p in json.loads((PUBLIC/'data/catalog.json').read_text('utf-8'))}
    if start not in places or end not in places:
        raise ValueError('등록된 장소를 출발지와 도착지로 선택해 주세요.')
    if mode=='verified':
        return {'found':False,'route':[],'segments':[], 'message':'현장 검증을 마친 보행 지도는 아직 없습니다. 공간 기반 경로는 기본 모드에서 확인할 수 있습니다.'}
    nav=load_navigation()
    if avoid_stairs:policy='elevator'
    nav.portal_allowed({'kind':'walk'},policy=policy)
    if any(not places[key].get('navigation_node') for key in (start,end)):
        return {'found':False,'route':[],'segments':[],'message':'CU는 학생회관 1층으로 안내되어 있으나 정확한 출입구 좌표를 확인 중입니다. 임의의 위치로 안내하지 않습니다.'}
    a,b=(tuple(places[key]['navigation_node']) for key in (start,end))
    result=nav.search(a,b,policy=policy)
    if result is None:
        return {'found':False,'route':[],'segments':[],'message':
            '계단을 제외한 연결 경로가 없습니다. 엘리베이터 승강장과 통로를 확인해 주세요.' if policy=='elevator' else
            '확인 가능한 보행면이 연결되지 않습니다. 벽이나 미관측 영역을 가로지르는 경로는 만들지 않습니다.'}
    route,segments=nav.assemble(result)
    photos=json.loads((PUBLIC/'data/photos.json').read_text('utf-8'))
    for segment in segments:
        p=route[segment['start_index']]['position']
        candidates=[photo for photo in photos if (photo['position'][2]>2.1)==(p[2]>2.1)]
        if candidates and not segment['photo_node']:segment['photo_node']=min(candidates,key=lambda photo:math.dist(photo['position'][:2],p[:2]))['id']
        if segment['portal_id']=='main_stairs':segment['photo_node']='floor2:73'
    alternatives=[]
    for alternative,label in [('shortest','최단 도보 · 계단 포함'),('main_stairs','메인 계단 이용'),('sub_stairs','오른쪽 서브계단 이용'),('elevator','계단 제외 · 엘리베이터 후보')]:
        candidate=nav.search(a,b,policy=alternative)
        portals=[p['id'] for p,_ in candidate['steps'] if p] if candidate else []
        alternatives.append({'policy':alternative,'label':label,'found':candidate is not None,
            'distance_m':round(candidate['distance'],1) if candidate else None,'portals':portals,
            'operation_verified':False if 'elevator' in portals else None})
    return {'found':True,'mode':'spatial','start':places[start]['name'],'end':places[end]['name'],
        'distance_m':round(result['distance'],1),'graph_distance_m':result['distance'],
        'node_count':len(route),'expanded_cells':result['expanded'],'review_connectors':len([s for s in segments if s['kind']!='walk']),
        'field_verified':False,'recording_trajectory_used':False,'algorithm':'A*','grid_resolution_m':nav.cell,
        'policy':policy,'alternatives':alternatives,'elevator_used':any(s['kind']=='elevator' for s in segments),
        'route':route,'segments':segments,
        'note':'선택한 이동 방식 내 거리 최소 경로입니다. 보행면·문·계단은 현장 검증 전입니다. 엘리베이터는 승강장 사이의 개념적 연결이며 운행·문턱·접근성·대기시간은 확인되지 않았습니다. 최단시간이나 무장애 경로를 보장하지 않습니다.'}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(PUBLIC), **kwargs)

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/api/route":
            q = parse_qs(url.query)
            try:
                value = find_route(q.get("start", ["outside_start"])[0], q.get("end", ["prime_hall"])[0],
                                   q.get("mode", ["demo"])[0], q.get("avoidStairs", ["0"])[0] == "1",policy=q.get('policy',['shortest'])[0])
                self.send_json(200, value)
            except (ValueError, sqlite3.Error) as error:
                self.send_json(400, {"error": str(error)})
        elif url.path in ('/api/walk/start','/api/walk/options','/api/walk/move'):
            q=parse_qs(url.query)
            try:
                goal=q.get('goal',['prime_hall'])[0]
                policy=q.get('policy',['shortest'])[0]
                if url.path.endswith('/start'):value=walk.start(q.get('place',['outside_start'])[0],goal,policy=policy)
                elif url.path.endswith('/move'):value=walk.transition(walk.parse_node(q.get('node',[''])[0]),float(q.get('yaw',['0'])[0]),q.get('choice',[''])[0],goal,policy=policy)
                else:value=walk.snapshot(walk.parse_node(q.get('node',[''])[0]),float(q.get('yaw',['0'])[0]),goal,policy=policy)
                self.send_json(200,value)
            except (ValueError,KeyError) as error:self.send_json(400,{'error':str(error)})
        elif url.path == "/api/health":
            self.send_json(200, {"ok": True, "name": "Folded City", "local": True})
        else:
            super().do_GET()

    def send_json(self, status, value):
        raw = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=8767)
    p.add_argument("--host", default="127.0.0.1")
    a = p.parse_args()
    print(f"Folded City 로컬 주소: http://{a.host}:{a.port}", flush=True)
    # A page opens many static-asset connections together, especially on a cold mobile cache.
    ThreadingHTTPServer.request_queue_size = 64
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()
