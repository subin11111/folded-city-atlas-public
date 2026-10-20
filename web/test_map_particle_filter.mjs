import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {MapParticleTracker,PARTICLE_SETTINGS} from './public/map-particle-filter.js';
import {GravityHeading} from './public/gravity-heading.js';
import {IndoorGrid} from './public/indoor-position.js';
const nav=JSON.parse(fs.readFileSync(new URL('./public/data/navigation.json',import.meta.url),'utf8'));
const catalog=JSON.parse(fs.readFileSync(new URL('./public/data/catalog.json',import.meta.url),'utf8'));
function room(){const bits=Buffer.alloc(200,255);return {cell_m:.5,origin_xy:[0,0],shape:[40,40],
  levels:{'1':{z:0,walkable_bits:bits.toString('base64')},'2':{z:4,walkable_bits:bits.toString('base64')}},portals:[]};}
const event=(z=20)=>({accelerationIncludingGravity:{x:0,y:0,z:-9.81},acceleration:{x:0,y:0,z:0},rotationRate:{alpha:z,beta:0,gamma:0}});
test('gravity-projected yaw uses deg/s and does not turn pitch into horizontal rotation',()=>{
  const h=new GravityHeading();h.anchor(90);h.sample(event(),1000);
  for(let t=1020;t<=2000;t+=20)h.sample(event(),t);
  assert.ok(Math.abs(h.bearing-70)<1e-8);
  const before=h.bearing;for(let t=2020;t<=3000;t+=20)h.sample({...event(0),rotationRate:{alpha:0,beta:50,gamma:0}},t);
  assert.ok(Math.abs(h.bearing-before)<1e-8);
});
test('gyro gaps and absent rates do not silently integrate missing movement',()=>{
  const h=new GravityHeading();h.anchor(10);h.sample(event(),1000);
  assert.equal(h.sample(event(),3000),null);assert.equal(h.bearing,null);
  assert.equal(h.sample(event(),3020),null);h.anchor(55);h.sample(event(0),3040);assert.equal(h.bearing,55);
  assert.equal(h.sample({...event(),rotationRate:{alpha:null,beta:0,gamma:0}},3060),null);
});
test('a stationary particle tracker neither moves with headings nor invents steps',()=>{
  const p=new MapParticleTracker(room(),[5,5,0]);
  const before=p.snapshot().position;
  for(let t=1000;t<3000;t+=20){p.heading([Math.cos(t/1000),Math.sin(t/1000)],t);p.motion(event(0),t);}
  assert.deepEqual(p.snapshot().position,before);assert.equal(p.snapshot().steps,0);
  assert.equal(p.step(.7,9000).reason,'heading');
});
test('all displayed steps and ancestry are wall-feasible; no route is supplied',()=>{
  const n=room(),bits=Buffer.from(n.levels['1'].walkable_bits,'base64');
  for(let r=0;r<40;r++){const i=r*40+20;bits[i>>3]&=~(1<<(i&7));}n.levels['1'].walkable_bits=bits.toString('base64');
  const p=new MapParticleTracker(n,[7,6,0],'1',{recordAncestry:true}),grid=new IndoorGrid(n);let before=p.snapshot().position;
  for(let i=0;i<25;i++){
    p.heading([1,0],1000+i*500);p.step(.65,1100+i*500);const after=p.snapshot().position;
    assert.ok(after[0]<9.75);assert.ok(grid.clear('1',before,after));before=after;
  }
  const path=p.ancestry();for(let i=1;i<path.length;i++)assert.ok(grid.clear('1',path[i-1].position,path[i].position));
});
test('particle proposals remain finite and within configured particle bounds',()=>{
  const p=new MapParticleTracker(room(),[5,5,0]);
  for(let i=0;i<20;i++){p.heading([1,0],1000+i*500);p.step(.6,1100+i*500);}
  const s=p.snapshot();assert.ok(s.position.every(Number.isFinite));assert.ok(s.uncertainty>=1.5);
  assert.ok(s.particleCount>=PARTICLE_SETTINGS.minParticles&&s.particleCount<=PARTICLE_SETTINGS.maxParticles);assert.deepEqual(p.ancestry(),[]);
});
test('confirmed floor resets height, never lets a 1F step become 2F automatically',()=>{
  const p=new MapParticleTracker(room(),[5,5,0]);p.heading([1,0],1000);p.step(.6,1100);
  assert.equal(p.snapshot().level,'1');p.reset([5,5,4],'2');assert.equal(p.snapshot().position[2],4.03);
  assert.equal(p.step(.6,2000).reason,'heading');
});
test('all registered anchors initialize particles only on their corresponding floor',()=>{
  for(const place of catalog){const level=place.level==='outside'?'1':place.level,p=new MapParticleTracker(nav,place.position,level);
    assert.ok(p.particles.every(q=>p.grid.free(level,[q.x,q.y])));}
});
test('production uses particle PDR and gyro front-end, without recorded position input',()=>{
  const text=fs.readFileSync(new URL('./public/location-controller.js',import.meta.url),'utf8');
  assert.match(text,/new MapParticleTracker\(/);assert.match(text,/gravityHeading.sample\(event,time\)/);
  assert.doesNotMatch(text,/observations\.json|comparison\.json|ancestry\(/);
});
