"""보행 지도상의 사람 시점 탐색. 촬영 순서를 경로로 사용하지 않는다."""
from collections import deque
from functools import lru_cache
import json
import math
import photo_library
from navigation import load_navigation,ROOT

STEP_M=2.4
EYE_HEIGHT=1.55

def wrap(angle):return (angle+math.pi)%(2*math.pi)-math.pi

def parse_node(value):
    try:
        lv,r,c=value.split(':');node=(lv,int(r),int(c))
    except (ValueError,AttributeError):raise ValueError('올바른 보행 지점이 아닙니다.')
    if not load_navigation().free(*node):raise ValueError('통행 가능한 보행 지점이 아닙니다.')
    return node

@lru_cache(maxsize=1)
def places():return {p['id']:p for p in json.loads((ROOT/'public/data/catalog.json').read_text('utf-8'))}

@lru_cache(maxsize=1)
def photo_views():return json.loads((ROOT/'public/data/photo_views.json').read_text('utf-8'))

@lru_cache(maxsize=1)
def outside_cells():
    nav=load_navigation();seed=tuple(places()['outside_start']['navigation_node']);seen={seed};queue=deque([seed])
    while queue:
        for n,_,portal,_ in nav.neighbours(queue.popleft()):
            if not portal and n not in seen:seen.add(n);queue.append(n)
    return seen

def closest_photo(node,yaw):
    return photo_library.match(node,yaw)

def option_from_result(identity,label,result):
    nav=load_navigation();path,_=nav.assemble(result)
    first=path[0]['position'];last=path[-1]['position']
    direction=next((p['position'] for p in path[1:] if math.dist(p['position'][:2],first[:2])>.1),last)
    heading=math.atan2(direction[1]-first[1],direction[0]-first[0])
    return {'id':identity,'label':label,'distance_m':round(result['distance'],2),
        'destination_node':':'.join(map(str,result['nodes'][-1])),'path':path,'heading':heading,
        'kind':next((p['kind'] for p in path if p['kind'] in ('stairs','door','elevator')),'walk')}

def direct_step(node,yaw):
    nav=load_navigation();origin=nav.position(node);nodes=[node];steps=[];distance=0.
    for d in [i*nav.cell/2 for i in range(1,int(STEP_M*2/nav.cell)+1)]:
        x=origin[0]+math.cos(yaw)*d;y=origin[1]+math.sin(yaw)*d
        c=round((x-nav.origin[0])/nav.cell);r=round((y-nav.origin[1])/nav.cell);n=(node[0],r,c)
        if n==nodes[-1]:continue
        available={other:cost for other,cost,portal,_ in nav.neighbours(nodes[-1]) if portal is None}
        if n not in available:break
        distance+=available[n];nodes.append(n);steps.append((None,False))
    if distance<.6:return None
    return {'nodes':nodes,'steps':steps,'distance':distance}

def route_prefix(node,goal,policy='shortest'):
    if not places()[goal].get('navigation_node'):raise ValueError('목적지 출입구 위치를 확인 중입니다.')
    nav=load_navigation();full=nav.search(node,tuple(places()[goal]['navigation_node']),policy=policy)
    if full is None:return None,None
    nodes=[node];steps=[];distance=0.
    for other,(portal,reverse) in zip(full['nodes'][1:],full['steps']):
        if portal:
            if steps:break
            cost=sum(math.dist(a,b) for a,b in zip(portal['polyline'],portal['polyline'][1:]))
        else:cost=math.dist(nav.position(nodes[-1]),nav.position(other))
        if not portal and distance+cost>STEP_M and steps:break
        nodes.append(other);steps.append((portal,reverse));distance+=cost
        if portal or distance>=STEP_M:break
    return ({'nodes':nodes,'steps':steps,'distance':distance} if steps else None),full['distance']

def snapshot(node,yaw,goal='prime_hall',include_photo=True,policy='shortest'):
    if not math.isfinite(yaw):raise ValueError('바라보는 방향이 올바르지 않습니다.')
    if goal not in places():raise ValueError('등록되지 않은 목적지입니다.')
    yaw=wrap(yaw);nav=load_navigation();position=nav.position(node)
    if not nav.free(*node):raise ValueError('보행 지도 밖입니다.')
    options=[]
    recommended,remaining=route_prefix(node,goal,policy)
    if recommended:
        option=option_from_result('recommended','목적지 방향으로 이동',recommended)
        portal=next((p for p,_ in recommended['steps'] if p),None)
        if portal:
            option['portal_id']=portal['id'];option['label']=portal['name']
        option['recommended']=True;options.append(option)
    for identity,label,delta in [('forward','앞으로',0),('left','왼쪽 통로',math.pi/2),('right','오른쪽 통로',-math.pi/2),('back','뒤로',math.pi)]:
        result=direct_step(node,yaw+delta)
        if result:
            option=option_from_result(identity,label,result);option['relative']=identity;options.append(option)
    for other,_,portal,reverse in nav.portals.get(node,[]):
        if not nav.portal_allowed(portal,policy=policy):continue
        result={'nodes':[node,other],'steps':[(portal,reverse)],'distance':sum(math.dist(a,b) for a,b in zip(portal['polyline'],portal['polyline'][1:]))}
        label=portal['name']
        options.append(option_from_result(portal['id'],label,result))
    zone='입구 앞' if node in outside_cells() else f'{node[0]}층'
    return {'node':':'.join(map(str,node)),'position':position,'eye_height_m':EYE_HEIGHT,'yaw':yaw,'level':node[0],
        'location':f'제1학생회관 · {zone}','goal':places()[goal]['name'],'remaining_m':round(remaining,1) if remaining is not None else None,
        'arrived':remaining is not None and remaining<.65,'options':options,'arrival_note':places()[goal].get('arrival_note'),
        'photo':closest_photo(node,yaw) if include_photo else None,
        'nearby_view':photo_library.match(node,yaw,allow_turn=True) if include_photo else None,
        'navigation_source':'spatial floor grid and explicit portals, not recorded camera sequence',
        'field_verified':False,'is_live_positioning':False,'policy':policy,
        'policy_note':'엘리베이터 운행·문턱·접근성 미검증 · 무장애 보장 아님' if policy=='elevator' else '선택한 연결 내 최단 도보 거리'}

def start(place='outside_start',goal='prime_hall',policy='shortest'):
    if place not in places():raise ValueError('등록된 시작 지점이 아닙니다.')
    node=tuple(places()[place]['navigation_node']);p=load_navigation().position(node)
    if place=='outside_start':target=places()['entrance']['position']
    else:
        prefix,_=route_prefix(node,goal,policy)
        target=load_navigation().position(prefix['nodes'][1]) if prefix else [p[0]-1,p[1],p[2]]
    yaw=math.atan2(target[1]-p[1],target[0]-p[0])
    return snapshot(node,yaw,goal,policy=policy)

def transition(node,yaw,choice,goal='prime_hall',policy='shortest'):
    """공간 지도에서 승인한 한 이동 구간의 사진 샘플. 촬영 순서 연결은 입력하지 않는다."""
    s=snapshot(node,yaw,goal,include_photo=False,policy=policy)
    option=next((o for o in s['options'] if o['id']==choice),None)
    if option is None:raise ValueError('현재 지점에서 선택할 수 없는 이동입니다.')
    output=transition_for_option(option,goal,policy)
    reverse={**option,'path':option['path'][::-1],'destination_node':':'.join(map(str,node)),'heading':yaw}
    output['reverse_samples']=transition_for_option(reverse,goal,policy)['samples']
    return output

def transition_for_option(option,goal='prime_hall',policy='shortest'):
    path=option['path'];lengths=[0.]
    for a,b in zip(path,path[1:]):lengths.append(lengths[-1]+math.dist(a['position'],b['position']))
    total=lengths[-1];samples=[]
    for distance in sorted(set([0.,total]+[i*.75 for i in range(1,math.ceil(total/.75))])):
        index=1
        while index<len(path)-1 and lengths[index]<distance:index+=1
        a,b=path[index-1],path[index];den=lengths[index]-lengths[index-1]
        t=(distance-lengths[index-1])/den if den else 0
        p=[v+(b['position'][j]-v)*t for j,v in enumerate(a['position'])]
        heading=math.atan2(b['position'][1]-a['position'][1],b['position'][0]-a['position'][0])
        on_stairs=option['kind']=='stairs' and distance>.1 and distance<total-.1
        on_elevator=option['kind']=='elevator' and distance>.1 and distance<total-.1
        photo=None;nearby=None;sample_node=None
        if on_elevator:
            # The car and shaft were not scanned. Never replace this ride by a stair/floor photo.
            pass
        elif on_stairs:
            photo=photo_library.stair_match(p,heading);nearby=photo or photo_library.stair_match(p,heading,True)
        else:
            level=(a['level'] if a['level'] in ('1','2') else b['level'])
            if level not in ('1','2'):level=parse_node(option['destination_node'])[0]
            try:sample_node=load_navigation().snap(level,p[:2],maximum=.25)
            except ValueError:sample_node=None
            if sample_node is None and option['kind']=='door':
                endpoint=path[0] if distance<total/2 else path[-1]
                try:sample_node=load_navigation().snap(endpoint['level'],endpoint['position'][:2],maximum=.25)
                except ValueError:pass
            if sample_node:
                photo=photo_library.match(sample_node,heading);nearby=photo or photo_library.match(sample_node,heading,True)
                if option['kind']=='door':
                    for image in [photo,nearby]:
                        if image:image['distance_m']=round(math.dist(image['position'][:2],p[:2]),2);image['context']='door'
        samples.append({'fraction':round(distance/total,5) if total else 1,'position':p,'level':'elevator' if on_elevator else 'stairs' if on_stairs else (sample_node[0] if sample_node else path[-1]['level']),
            'photo':photo,'nearby_view':nearby,'heading':heading,
            'notice':'엘리베이터 연결 시뮬레이션 · 내부 촬영 없음 · 운행 확인 필요' if on_elevator else None})
    end_node=parse_node(option['destination_node'])
    # Face a photographed entrance once on arrival; subsequent manual turns stay free.
    arrival=places()[goal]
    next_heading=arrival.get('arrival_heading',option['heading']) if end_node==tuple(arrival['navigation_node']) else option['heading']
    next_view=snapshot(end_node,next_heading,goal,policy=policy)
    return {'option':option,'next':next_view,'samples':samples,'recording_trajectory_used':False}
