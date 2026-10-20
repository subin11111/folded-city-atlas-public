import {createCampusFrame} from './campus-data.js';
export const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
export function project(p,a,b){const dx=b[0]-a[0],dy=b[1]-a[1],den=dx*dx+dy*dy;const u=den?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)):0;const point=[a[0]+u*dx,a[1]+u*dy];return {point,u,distance:distance(p,point)};}
export function createOutdoorRouter(campus,map){
  const frame=createCampusFrame(campus,map),nodes=new Map(),edges=[];
  const allowed=new Set(['footway','path','pedestrian','residential','service','living_street','steps','track']);
  for(const road of campus.roads){
    if(!allowed.has(road.k)||['no','private'].includes(road.foot)||(['no','private'].includes(road.access)&&!['yes','designated','permissive'].includes(road.foot)))continue;
    const points=road.p.map(p=>frame.toLocal(p).slice(0,2));
    // Shared OSM geometry endpoints only, never connect visual crossings on different layers.
    const ids=road.p.map(p=>`${p[0]},${p[1]},${road.layer||0}`);
    points.forEach((p,i)=>nodes.set(ids[i],p));
    for(let i=1;i<ids.length;i++)if(distance(points[i-1],points[i])>.05)edges.push({a:ids[i-1],b:ids[i],p:points[i-1],q:points[i],road});
  }
  function nearest(p){let best=null;for(const edge of edges){const hit=project(p,edge.p,edge.q);if(!best||hit.distance<best.distance)best={...hit,edge};}return best;}
  function route(start,end){
    const a=nearest(start),b=nearest(end);
    if(!a||!b||a.distance>25||b.distance>60)return {found:false,message:'등록된 보행로에서 멀어 현재 위치 경로를 연결하지 못했어요.'};
    const adjacency=new Map();
    function link(i,j,cost){if(!adjacency.has(i))adjacency.set(i,[]);adjacency.get(i).push([j,cost]);}
    function both(i,j,cost){link(i,j,cost);link(j,i,cost);}
    for(const e of edges){
      const cuts=[{id:e.a,u:0,p:e.p},{id:e.b,u:1,p:e.q}];
      if(e===a.edge)cuts.push({id:'@start',u:a.u,p:a.point});
      if(e===b.edge)cuts.push({id:'@end',u:b.u,p:b.point});
      cuts.sort((x,y)=>x.u-y.u);for(let i=1;i<cuts.length;i++)both(cuts[i-1].id,cuts[i].id,distance(cuts[i-1].p,cuts[i].p));
    }
    const costs=new Map([['@start',0]]),prev=new Map(),pending=new Set(['@start']);
    while(pending.size){let n=null;for(const key of pending)if(n===null||costs.get(key)<costs.get(n))n=key;pending.delete(n);if(n==='@end')break;
      for(const [other,cost] of adjacency.get(n)||[]){const g=costs.get(n)+cost;if(g<(costs.get(other)??Infinity)){costs.set(other,g);prev.set(other,n);pending.add(other);}}
    }
    if(!costs.has('@end'))return {found:false,message:'OSM 보행로 연결이 끊겨 있어 경로를 만들지 못했어요.'};
    const ids=['@end'];while(ids.at(-1)!=='@start')ids.push(prev.get(ids.at(-1)));ids.reverse();
    const points=ids.map(id=>id==='@start'?a.point:id==='@end'?b.point:nodes.get(id));
    return {found:true,points,startGap:a.distance,endGap:b.distance,entry:end.slice(0,2),distance:costs.get('@end'),entryConnected:false,osmIds:[...new Set(edges.filter(e=>ids.includes(e.a)||ids.includes(e.b)).map(e=>e.road.osm_id))],fieldVerified:false};
  }
  return {route,nearest,edges,nodes};
}
export function routeProgress(position,points){
  let total=0,best=null;for(let i=1;i<points.length;i++){const length=distance(points[i-1],points[i]),hit=project(position,points[i-1],points[i]);if(!best||hit.distance<best.crossTrack)best={crossTrack:hit.distance,along:total+hit.u*length,index:i,point:hit.point};total+=length;}
  return best?{...best,total,remaining:Math.max(0,total-best.along)}:null;
}
