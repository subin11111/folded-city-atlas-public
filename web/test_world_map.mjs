import assert from 'node:assert/strict';
import {test} from 'node:test';
import fs from 'node:fs';
import {createWorldRegistration,mercatorPoint,displayElevationOffset} from './public/world-registration.js';
import {createGeoFrame} from './public/geo-frame.js';
import {insideRing,ringCentre} from './public/campus-data.js';
const read=name=>JSON.parse(fs.readFileSync(new URL(`./public/data/${name}.json`,import.meta.url),'utf8'));
const map=read('map'),campus=read('campus_source'),style=read('world_style'),registration=createWorldRegistration(map),geo=createGeoFrame(map);
test('실내 표시는 1층을 배경 평면에 맞추고 2층 높이와 계단 길이를 유지한다',()=>{
  const first=map.levels.find(l=>l.id==='1').z,second=map.levels.find(l=>l.id==='2').z;
  const offset=displayElevationOffset(map,{mode:'indoor',campusLift:13.7328125});
  assert.ok(Math.abs(first+offset-.08)<1e-12);
  assert.ok(Math.abs((second+offset)-(first+offset)-4.15)<1e-12);
  assert.equal(displayElevationOffset(map,{mode:'campus',campusLift:13.7328125}),13.7328125);
  const nav=read('navigation');
  for(const portal of nav.portals){
    const shifted=portal.polyline.map(([x,y,z])=>[x,y,z+offset]);
    const length=points=>points.slice(1).reduce((s,p,i)=>s+Math.hypot(...p.map((v,k)=>v-points[i][k])),0);
    assert.ok(Math.abs(length(shifted)-length(portal.polyline))<1e-10,portal.id);
  }
  const scene=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  assert.ok(scene.includes('world?.setSceneMode(mode)'));
});
test('전역 배경은 지역 bounds와 3D 건물 레이어를 포함하지 않는다',()=>{
  assert.equal(style.version,8);assert.match(style.sources.openmaptiles.url,/openfreemap\.org\/planet$/);
  assert.ok(style.layers.length>8);assert.ok(!style.layers.some(l=>l.type==='fill-extrusion'));
  assert.ok(!('bounds' in style.sources.openmaptiles));
  assert.ok(campus.buildings.every(b=>campus.campus.some(r=>insideRing(ringCentre(b.o),r))));
});
test('교내 SLAM 좌표와 배경지도 좌표를 축척 변형 없이 왕복한다',()=>{
  for(const point of [[0,0],[-40,-30],[30,40],[-500,-400],[500,400]]){
    const [lon,lat]=registration.toLngLat(point),back=geo.fromGeo(lat,lon);
    assert.ok(Math.hypot(back[0]-point[0],back[1]-point[1])<.002);
  }
  const a=mercatorPoint(registration.toLngLat([0,0])),b=mercatorPoint(registration.toLngLat([30,40]));
  assert.ok(Math.abs(Math.hypot(b[0]-a[0],b[1]-a[1])/registration.scale-50)<.12);
});
test('지역 3D 투영 행렬은 같은 지리 기준점에 대응한다',()=>{
  const m=registration.matrix;
  for(const [x,y] of [[-35,-30],[25,35],[200,-200]]){
    const exact=mercatorPoint(registration.toLngLat([x,y]));
    const affine=[m[0]*x+m[4]*y+m[12],m[1]*x+m[5]*y+m[13]];
    assert.ok(Math.hypot(exact[0]-affine[0],exact[1]-affine[1])/registration.scale<.08);
  }
});

test('캠퍼스 바닥을 DEM 최저점으로 띄우지 않고 평면 배경과 같은 기준에 둔다',()=>{
  const source=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  assert.ok(source.includes('terrainHeight:()=>metricFrame.datum'));
  assert.ok(source.includes('const displayBase=frame.datum;'));
  assert.ok(source.includes('const lift=.08;'));
  assert.ok(!source.includes('frame.datum-Math.min(...surface.heights)'));
  const z=displayElevationOffset(map,{mode:'indoor'})+map.levels.find(l=>l.id==='1').z;
  assert.ok(Math.abs(z-.08)<1e-12);
});
