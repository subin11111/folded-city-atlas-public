"""공간 격자 위 A*: 촬영 궤적과 촬영 순서를 입력으로 받지 않는다."""
from __future__ import annotations
import base64
from functools import lru_cache
import heapq
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parent


class Navigation:
    def __init__(self, data):
        self.data=data
        self.cell=data['cell_m']
        self.origin=data['origin_xy']
        self.rows,self.cols=data['shape']
        self.grids={lv:base64.b64decode(g['walkable_bits']) for lv,g in data['levels'].items()}
        self.portals={}
        for p in data['portals']:
            a=tuple(p['a']);b=tuple(p['b'])
            length=sum(math.dist(x,y) for x,y in zip(p['polyline'],p['polyline'][1:]))
            self.portals.setdefault(a,[]).append((b,length,p,False))
            self.portals.setdefault(b,[]).append((a,length,p,True))

    def free(self,level,row,col):
        if level not in self.grids or not(0<=row<self.rows and 0<=col<self.cols):return False
        index=row*self.cols+col
        return bool(self.grids[level][index//8] & (1<<(index%8)))

    def position(self,node):
        level,row,col=node
        return [self.origin[0]+col*self.cell,self.origin[1]+row*self.cell,self.data['levels'][level]['z']+.03]

    def snap(self,level,xy,maximum=1.0):
        level='1' if level=='outside' else level
        col=round((xy[0]-self.origin[0])/self.cell);row=round((xy[1]-self.origin[1])/self.cell)
        radius=math.ceil(maximum/self.cell)
        candidates=[]
        for r in range(row-radius,row+radius+1):
            for c in range(col-radius,col+radius+1):
                if self.free(level,r,c):
                    p=self.position((level,r,c));d=math.dist(xy,p[:2])
                    if d<=maximum:candidates.append((d,(level,r,c)))
        if not candidates:raise ValueError('목적지 주변의 보행면을 확인하지 못했습니다. 벽 너머로 강제 연결하지 않습니다.')
        return min(candidates)[1]

    def portal_allowed(self,portal,avoid_stairs=False,policy='shortest'):
        if policy not in ('shortest','elevator','main_stairs','sub_stairs'):
            raise ValueError('지원하지 않는 이동 방식입니다.')
        if avoid_stairs or policy=='elevator':return portal['kind']!='stairs'
        if portal['kind']=='elevator':return False
        if policy=='main_stairs' and portal['kind']=='stairs':return portal['id']=='main_stairs'
        if policy=='sub_stairs' and portal['kind']=='stairs':return portal['id']=='sub_stairs'
        return True

    def neighbours(self,node,avoid_stairs=False,policy='shortest'):
        level,r,c=node
        for dr,dc in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            if not self.free(level,r+dr,c+dc):continue
            # 두 벽의 모서리 사이를 대각선으로 뚫고 지나가지 않는다.
            if dr and dc and not(self.free(level,r+dr,c) and self.free(level,r,c+dc)):continue
            yield (level,r+dr,c+dc),self.cell*math.hypot(dr,dc),None,False
        for other,length,portal,reverse in self.portals.get(node,[]):
            if not self.portal_allowed(portal,avoid_stairs,policy):continue
            yield other,length,portal,reverse

    def search(self,a,b,avoid_stairs=False,heuristic=True,policy='shortest'):
        self.portal_allowed({'kind':'walk'},avoid_stairs,policy)
        target=self.position(b)
        def estimate(n):return math.dist(self.position(n),target) if heuristic else 0.
        queue=[(estimate(a),0.,a)];dist={a:0.};previous={};expanded=0
        while queue:
            _,g,n=heapq.heappop(queue)
            if g!=dist[n]:continue
            expanded+=1
            if n==b:break
            for other,cost,portal,reverse in self.neighbours(n,avoid_stairs,policy):
                new=g+cost
                if new+1e-10<dist.get(other,math.inf):
                    dist[other]=new;previous[other]=(n,portal,reverse)
                    heapq.heappush(queue,(new+estimate(other),new,other))
        if b not in dist:return None
        nodes=[b];steps=[]
        while nodes[-1]!=a:
            before,portal,reverse=previous[nodes[-1]]
            steps.append((portal,reverse));nodes.append(before)
        return {'nodes':nodes[::-1],'steps':steps[::-1],'distance':dist[b],'expanded':expanded}

    def assemble(self,result):
        """격자 경로와 계단의 연결 형상을 반환한다. 벽을 가로질러 평활화하지 않는다."""
        nodes=result['nodes'];steps=result['steps'];route=[];segments=[]
        def add(p,level,kind,identity):
            route.append({'id':identity,'position':[round(v,4) for v in p], 'level':level,'session':'navigation','source_node':None,'kind':kind})
        add(self.position(nodes[0]),nodes[0][0],'walk','nav:start')
        for i,(portal,reverse) in enumerate(steps):
            before,after=nodes[i:i+2]
            if portal:
                points=portal['polyline'][::-1] if reverse else portal['polyline']
                label=portal['name'];kind=portal['kind']
                start=len(route)-1
                for j,p in enumerate(points[1:]):
                    level=after[0] if j==len(points)-2 else (kind if kind in ('stairs','elevator') else before[0])
                    add(p,level,kind,f"nav:{portal['id']}:{j}")
                distance=sum(math.dist(x,y) for x,y in zip(points,points[1:]))
            else:
                label=f"{after[0]}층 보행 공간";kind='walk';start=len(route)-1
                add(self.position(after),after[0],kind,f'nav:{after[0]}:{after[1]}:{after[2]}')
                distance=math.dist(self.position(before),self.position(after))
            if segments and segments[-1]['label']==label:
                segments[-1]['distance']+=distance;segments[-1]['end_index']=len(route)-1
            else:
                segments.append({'label':label,'kind':kind,'distance':distance,'start_index':start,'end_index':len(route)-1,
                    'portal_id':portal['id'] if portal else None,'photo_node':portal.get('photo_node') if portal else None})
        return route,segments


@lru_cache(maxsize=1)
def load_navigation():
    return Navigation(json.loads((ROOT/'public/data/navigation.json').read_text('utf-8')))
