import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {StepDetector,IndoorGrid,PedestrianTracker,indoorProgress,requestWalkingSensors} from './public/indoor-position.js';
const nav=JSON.parse(fs.readFileSync(new URL('./public/data/navigation.json',import.meta.url),'utf8'));
const grid=new IndoorGrid(nav);
function simpleGrid(){const bytes=new Uint8Array(200);bytes.fill(255);return {cell_m:1,origin_xy:[0,0],shape:[40,40],levels:{'1':{z:0,walkable_bits:Buffer.from(bytes).toString('base64')},'2':{z:4,walkable_bits:Buffer.from(bytes).toString('base64')}},portals:[]};}
test('stationary data, tiny tremors, null sensors and stale gaps do not create walking',()=>{
  const d=new StepDetector();for(let t=0;t<6000;t+=20)assert.equal(d.sample(9.81+.04*Math.sin(t/80),t),null);
  assert.equal(d.sample(null,8000),null);assert.equal(d.sample(9.81,12000),null);
  const p=new PedestrianTracker(simpleGrid(),[10,10,0]);assert.equal(p.motion({accelerationIncludingGravity:{x:null,y:null,z:null}},1000).reason,'no_motion');
});
test('a full accelerometer cycle produces bounded steps rather than double integration',()=>{
  const d=new StepDetector(),steps=[];for(let t=0;t<11000;t+=20){const r=d.sample(9.81+(t<1200?0:2*Math.sin((t-1200)/500*2*Math.PI)),t);if(r)steps.push(r);}
  assert.ok(steps.length>=15&&steps.length<=20,steps.length);assert.ok(steps.every(s=>s.length>=.48&&s.length<=.85));
});
test('movement comes from live steps and heading, not from the requested route',()=>{
  const p=new PedestrianTracker(simpleGrid(),[10,10,0]);assert.equal(p.step(.7,2000).reason,'heading');
  p.heading([1,0],2000,10);const a=p.step(.7,2100);assert.equal(a.moved,true);assert.ok(Math.abs(a.position[0]-10.7)<1e-8);assert.equal(a.position[1],10);
  assert.equal(p.step(.7,7000).reason,'heading');assert.equal(p.snapshot().level,'1');
  p.reset([10,10,4],'2');assert.equal(p.snapshot().position[2],4.03);
});
test('walls and diagonal blocked corners cannot be crossed or snapped through',()=>{
  const data=simpleGrid(),bits=Buffer.from(data.levels['1'].walkable_bits,'base64');
  for(let r=0;r<40;r++){const i=r*40+12;bits[i>>3]&=~(1<<(i&7));}data.levels['1'].walkable_bits=bits.toString('base64');
  const p=new PedestrianTracker(data,[10,10,0]);p.heading([1,0],2000,10);
  for(let i=0;i<15;i++){p.heading([1,0],2100+i*500,10);p.step(.7,2200+i*500);}
  assert.ok(p.snapshot().position[0]<11.6);assert.ok(p.snapshot().blocked>0);
});
test('turning the phone while stationary does not translate position, and compass jumps pause steps',()=>{
  const p=new PedestrianTracker(simpleGrid(),[10,10,0]);p.heading([1,0],2000,10);
  assert.equal(p.heading([-1,0],2100,10),false);assert.equal(p.step(.7,2200).reason,'heading');
  assert.equal(p.heading([-1,0],2400,10),false);assert.equal(p.heading([-1,0],3000,10),true);
  assert.deepEqual(p.snapshot().position,[10,10,.03]);
});
test('real entry and all registered place anchors lie on the corresponding level',()=>{
  const places=JSON.parse(fs.readFileSync(new URL('./public/data/catalog.json',import.meta.url),'utf8'));
  for(const p of places)assert.ok(grid.free(p.level==='outside'?'1':p.level,p.position),p.id);
  assert.equal(grid.free('3',[0,0]),false);
});
test('route progress never mistakes the same XY on another floor for arrival',()=>{
  const route={route:[{position:[0,0,0],level:'1',kind:'walk'},{position:[5,0,0],level:'1',kind:'walk'},{position:[5,0,4],level:'2',kind:'stairs'},{position:[0,0,4],level:'2',kind:'walk'}],segments:[{kind:'stairs',start_index:1,end_index:2}]};
  assert.equal(indoorProgress({position:[0,0,0],level:'1'},route).arrived,false);
  assert.ok(indoorProgress({position:[4,0,0],level:'1'},route).transition);
  assert.equal(indoorProgress({position:[0,0,4],level:'2'},route).arrived,true);
});
test('both iOS permissions are requested in the tap call stack, including denial',async()=>{
  const calls=[];let resolveMotion;const motion=new Promise(resolve=>resolveMotion=resolve);
  const promise=requestWalkingSensors({DeviceMotionEvent:{requestPermission(){calls.push('motion');return motion;}},DeviceOrientationEvent:{requestPermission(absolute){assert.equal(absolute,true);calls.push('orientation');return Promise.resolve('denied');}}});
  assert.deepEqual(calls,['motion','orientation']);resolveMotion('granted');assert.deepEqual(await promise,{motion:true,orientation:false});
  assert.deepEqual(await requestWalkingSensors({}),{motion:false,orientation:false});
});
