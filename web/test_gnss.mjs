import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {GnssFilter,rtsSmooth,circularHeading,mul,inverse} from './public/gnss-filter.js';
import {createGeoFrame} from './public/geo-frame.js';
import {createOutdoorRouter,routeProgress} from './public/outdoor-routing.js';
const read=name=>JSON.parse(fs.readFileSync(new URL(`./public/data/${name}.json`,import.meta.url),'utf8'));
test('circular heading crosses north without averaging to south',()=>{assert.equal(circularHeading(359,1,.5),0);assert.equal(circularHeading(null,null),null);});
test('filter rejects invalid, nonmonotonic, imprecise and jumping fixes',()=>{
 const f=new GnssFilter();assert.equal(f.update({x:0,y:0,t:1,accuracy:0}).reason,'invalid');
 assert.ok(f.update({x:0,y:0,t:1,accuracy:5}).accepted);assert.equal(f.update({x:0,y:0,t:1,accuracy:5}).reason,'time_order');
 assert.equal(f.update({x:0,y:0,t:2,accuracy:80}).reason,'accuracy');
 assert.equal(f.update({x:90,y:40,t:3,accuracy:5}).accepted,false);assert.ok(f.update({x:2,y:0,t:4,accuracy:5}).accepted);
 assert.ok(f.update({x:30,y:0,t:30,accuracy:5}).segment);assert.equal(f.history.at(-1).x[2],0);
});
test('Joseph covariance stays symmetric and finite; RTS never bridges outages',()=>{
 const f=new GnssFilter();for(let i=0;i<35;i++)f.update({x:i+(i%2?1:-1),y:0,t:i,accuracy:8});
 const before=rtsSmooth(f.history);f.update({x:200,y:90,t:60,accuracy:8});const after=rtsSmooth(f.history);
 assert.deepEqual(before.at(-1).position,after[34].position);
 for(const r of f.history)for(let i=0;i<4;i++)for(let j=0;j<4;j++){assert.ok(Number.isFinite(r.P[i][j]));assert.ok(Math.abs(r.P[i][j]-r.P[j][i])<1e-8);}
 for(const r of after)for(let i=0;i<4;i++)assert.ok(r.P[i][i]>=0&&r.P[i][i]<=f.history[after.indexOf(r)].P[i][i]+1e-7);
});
test('matrix inverse identity',()=>{const a=[[5,2],[2,3]],p=mul(a,inverse(a));assert.ok(Math.abs(p[0][0]-1)<1e-9);assert.ok(Math.abs(p[1][0])<1e-9);});
test('GNSS frame matches campus ENU and has unit scale',()=>{
 const map=read('map'),f=createGeoFrame(map),g=map.georeference;
 const p=f.fromGeo(g.origin_latitude_deg,g.origin_longitude_deg),v=f.vector(-g.t_map_xy_to_EN_m[0]-2,-g.t_map_xy_to_EN_m[1]+2);assert.ok(Math.hypot(p[0]-v[0],p[1]-v[1])<1e-7);
 assert.ok(Math.abs(Math.hypot(...f.headingVector(123))-1)<1e-9);
});
test('outdoor routing respects access and split positions on the same segment',()=>{
 const map=read('map'),campus=read('campus_source'),router=createOutdoorRouter(campus,map),edge=router.edges[0];
 const a=edge.p.map((v,i)=>v*.8+edge.q[i]*.2),b=edge.p.map((v,i)=>v*.2+edge.q[i]*.8),r=router.route(a,b);
 assert.ok(r.found);assert.ok(Math.abs(r.distance-Math.hypot(b[0]-a[0],b[1]-a[1]))<1e-6);
 assert.ok(!router.edges.some(e=>e.road.foot==='no'));
 assert.equal(router.route([10000,10000],a).found,false);
 assert.equal(routeProgress(a,[a,b]).along,0);
});
test('deterministic replay rejects injected jumps and RTS improves this synthetic case',()=>{
 const demo=read('gnss_demo'),f=new GnssFilter(),frame=createGeoFrame(read('map'));
 for(const r of demo.records){const xy=frame.fromGeo(r.coords.latitude,r.coords.longitude);const v=f.update({x:xy[0],y:xy[1],t:r.timestamp/1000,accuracy:r.coords.accuracy});if(r.scenario==='injected_jump')assert.equal(v.accepted,false);}
 const truth=new Map(demo.truth.map(r=>[r.timestamp/1000,r.position]));
 const rmse=rows=>Math.sqrt(rows.reduce((s,r)=>s+(r.p[0]-truth.get(r.t)[0])**2+(r.p[1]-truth.get(r.t)[1])**2,0)/rows.length);
 const raw=rmse(f.history.map(r=>({t:r.t,p:r.raw}))),kalman=rmse(f.history.map(r=>({t:r.t,p:r.x}))),rts=rmse(rtsSmooth(f.history).map(r=>({t:r.t,p:r.x})));
 assert.equal(demo.simulated,true);assert.ok(rts<kalman&&kalman<raw);
 for(const [actual,expected] of [[raw,demo.metrics.raw_rmse_m],[kalman,demo.metrics.kalman_rmse_m],[rts,demo.metrics.rts_rmse_m]])assert.ok(Math.abs(actual-expected)<1e-9);
});
