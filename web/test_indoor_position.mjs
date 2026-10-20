import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {StepDetector,IndoorGrid,PedestrianTracker,indoorProgress,requestWalkingSensors} from './public/indoor-position.js';
import {attitudeForward,WalkingHeading,sensorEventTime,strideCalibration} from './public/walking-sensors.js';
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

test('time-resampled walking cycles stay stable at 30/50/60/100 Hz with jitter',()=>{
  const counts=[];
  for(const hz of [30,50,60,100]){
    const d=new StepDetector();let count=0;
    for(let i=0;i<hz*15;i++){
      const t=i/hz*1000+(i?Math.sin(i*2.3)*1.5:0);
      if(d.sample(9.81+(t<1500?0:2*Math.sin((t-1500)*Math.PI*4/1000)),t))count++;
    }
    counts.push(count);
  }
  assert.ok(Math.max(...counts)-Math.min(...counts)<=1,JSON.stringify(counts));
  assert.ok(counts.every(n=>n>=24&&n<=27),JSON.stringify(counts));
});

test('missing input is not extrapolated and high-frequency tremor is not walking',()=>{
  const d=new StepDetector();let steps=0;
  for(let t=0;t<8000;t+=10)if(d.sample(9.81+2*Math.sin(t/1000*2*Math.PI*9),t))steps++;
  assert.equal(steps,0);assert.equal(d.sample(9.81,10000),null);
  for(let t=10020;t<10900;t+=20)assert.equal(d.sample(9.81+2*Math.sin(t/80),t),null);
  const p=new PedestrianTracker(simpleGrid(),[10,10,0]);p.heading([1,0],1000,10);
  assert.equal(p.motion({acceleration:{x:1,y:1,z:1}},1100).reason,'no_motion');
});

test('upright forward uses the rear-screen axis, while a flat screen uses its top',()=>{
  assert.equal(attitudeForward(0,0,0).bearing,0);
  assert.equal(attitudeForward(0,0,0,90).bearing,90);
  assert.equal(attitudeForward(0,90,90).bearing,270);
  assert.equal(attitudeForward(null,0,0),null);
  assert.equal(attitudeForward(10,90,0).axis,'rear');
});

test('Safari relative angles alone cannot establish an absolute walking direction',()=>{
  const h=new WalkingHeading();
  assert.equal(h.sample({alpha:0,beta:80,gamma:0,absolute:false,webkitCompassHeading:180,webkitCompassAccuracy:10},1000),null);
  assert.equal(h.sample({alpha:0,beta:0,gamma:0,webkitCompassHeading:-1,webkitCompassAccuracy:10},1100),null);
  assert.equal(h.sample({alpha:0,beta:0,gamma:0,webkitCompassHeading:180,webkitCompassAccuracy:-1},1200),null);
  const a=h.sample({alpha:20,beta:0,gamma:0,webkitCompassHeading:110,webkitCompassAccuracy:10},1300);
  assert.ok(Math.abs(a.bearing-110)<1e-8);
  assert.equal(a.source,'flat_compass_reference');
});

test('confirmed door direction removes arbitrary attitude yaw without a route',()=>{
  for(const origin of [0,37,120]){
    const h=new WalkingHeading(190),first={alpha:origin,beta:80,gamma:0};
    assert.equal(h.sample(first,1000),null);assert.equal(h.sample(first,1400),null);
    const aligned=h.sample(first,1800);assert.ok(Math.abs(aligned.bearing-190)<1e-7);
    const turned=h.sample({...first,alpha:origin+90},3800);assert.ok(Math.abs(turned.bearing-100)<.5);
    assert.equal(turned.source,'confirmed_entrance');
    h.reset();assert.equal(h.sample(first,4000),null);
  }
});

test('sensor event time handles relative/epoch timestamps and rejects delayed batches',()=>{
  assert.equal(sensorEventTime({timeStamp:500},1000500,1000000),1000500);
  assert.equal(sensorEventTime({timeStamp:1790000000500},1790000000500,1000),1790000000500);
  assert.equal(sensorEventTime({timeStamp:500},1005000,1000000),null);
  assert.equal(sensorEventTime({},1000500,1000000),1000500);
});

test('stride fitting needs an independently measured segment and enough cycles',()=>{
  assert.equal(strideCalibration(10,[4,4]),null);
  assert.equal(strideCalibration(500,new Array(16).fill(4)),null);
  assert.equal(strideCalibration(10,new Array(16).fill(NaN)),null);
  assert.equal(strideCalibration(11.2,[...new Array(8).fill(1.2),...new Array(8).fill(30)]),null);
  const result=strideCalibration(10,new Array(16).fill(4));
  assert.equal(result.steps,16);assert.ok(Math.abs(result.coefficient-10/(16*Math.pow(4,.25)))<1e-10);
});

test('guidance pauses for real stride calibration and re-anchors afterwards',()=>{
  const source=fs.readFileSync(new URL('./public/location-controller.js',import.meta.url),'utf8');
  assert.ok(source.includes('strideCalibration(strideSession.distance,strideSession.amplitudes)'));
  assert.ok(source.includes('stop();notify(message);chooseIndoor()'));
  assert.ok(source.includes("p=>p.id==='main_door'"));
  assert.ok(source.includes('sensorEventTime(event,now)'));
});
