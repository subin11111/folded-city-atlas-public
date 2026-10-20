import fs from 'node:fs';
import {createOutdoorRouter,routeProgress} from '../public/outdoor-routing.js';
import {createGeoFrame} from '../public/geo-frame.js';
import {GnssFilter,rtsSmooth} from '../public/gnss-filter.js';
const read=name=>JSON.parse(fs.readFileSync(new URL(`../public/data/${name}.json`,import.meta.url),'utf8'));
const map=read('map'),campus=read('campus_source'),entry=read('catalog').find(p=>p.id==='outside_start').position;
const router=createOutdoorRouter(campus,map),frame=createGeoFrame(map);
const candidates=[...router.nodes.values()].map(p=>({p,r:router.route(p,entry)})).filter(v=>v.r.found&&v.r.distance>90&&v.r.distance<135).sort((a,b)=>a.r.distance-b.r.distance);
if(!candidates.length)throw Error('No reproducible connected OSM approach');
const route=candidates[0].r,points=route.points,lengths=[0];for(let i=1;i<points.length;i++)lengths.push(lengths.at(-1)+Math.hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]));
let seed=20261020;const random=()=>{seed=(1664525*seed+1013904223)>>>0;return (seed+.5)/4294967296;};
const gaussian=()=>Math.sqrt(-2*Math.log(random()))*Math.cos(2*Math.PI*random());
const records=[],truth=[],start=Date.parse('2026-10-20T03:00:00Z');
function at(s){let i=1;while(i<lengths.length-1&&lengths[i]<s)i++;const u=(s-lengths[i-1])/(lengths[i]-lengths[i-1]||1);return points[i-1].map((v,j)=>v+(points[i][j]-v)*Math.max(0,Math.min(1,u)));}
for(let i=0;i<115;i++){
  if(i>=70&&i<88)continue;
  const p=at(Math.max(0,Math.min(lengths.at(-1),i-8))),q=at(Math.max(0,Math.min(lengths.at(-1),i-7)));
  const heading=(Math.atan2((q[0]-p[0])*frame.vector(1,0)[0]+(q[1]-p[1])*frame.vector(1,0)[1],(q[0]-p[0])*frame.vector(0,1)[0]+(q[1]-p[1])*frame.vector(0,1)[1])*180/Math.PI+360)%360;
  const spike=[32,33,55].includes(i),poor=i===61,noise=poor?20:2.2;
  const measured=[p[0]+gaussian()*noise+(spike?60:0),p[1]+gaussian()*noise-(spike?45:0)];
  records.push({timestamp:start+i*1000,coords:{...frame.toGeo(...measured),accuracy:poor?85:8,heading:i<9?null:heading,speed:i<9?0:1},scenario:spike?'injected_jump':poor?'poor_accuracy':i===88?'reacquisition':'normal'});
  truth.push({timestamp:start+i*1000,position:p});
}
const f=new GnssFilter();for(const r of records){const p=frame.fromGeo(r.coords.latitude,r.coords.longitude);f.update({x:p[0],y:p[1],t:r.timestamp/1000,accuracy:r.coords.accuracy});}
const smooth=rtsSmooth(f.history),truthByT=new Map(truth.map(r=>[r.timestamp/1000,r.position]));
const rmse=rows=>Math.sqrt(rows.reduce((sum,r)=>sum+Math.hypot(r.p[0]-truthByT.get(r.t)[0],r.p[1]-truthByT.get(r.t)[1])**2,0)/rows.length);
const metrics={records:records.length,accepted:f.history.filter(r=>r.accepted).length,rejected:records.length-f.history.filter(r=>r.accepted).length,raw_rmse_m:rmse(f.history.map(r=>({t:r.t,p:r.raw}))),kalman_rmse_m:rmse(f.history.map(r=>({t:r.t,p:r.x}))),rts_rmse_m:rmse(smooth.map(r=>({t:r.t,p:r.x}))),evaluation:'synthetic truth only; not field accuracy',route_m:route.distance,entry_gap_m:route.endGap};
const demo={schema:'folded-city-gnss-v1',simulated:true,description:'Seeded synthetic metre-noise around an OSM shortest pedestrian path, with injected jumps, poor accuracy and 18-second gap. Not measured phone data.',seed:20261020,records,truth,route,metrics};
fs.writeFileSync(new URL('../public/data/gnss_demo.json',import.meta.url),JSON.stringify(demo,null,2)+'\n');
fs.mkdirSync(new URL('../../docs/assets/gnss-navigation-20261020/',import.meta.url),{recursive:true});
fs.writeFileSync(new URL('../../docs/assets/gnss-navigation-20261020/filter_results.json',import.meta.url),JSON.stringify({metrics,history:f.history,smooth,truth,route},null,2)+'\n');
console.log(JSON.stringify(metrics,null,2));
