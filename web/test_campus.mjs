import assert from 'node:assert/strict';
import {test} from 'node:test';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {createCampusFrame,insideRing,applyCampusLabels,campusBuildingMatches} from './public/campus-data.js';
import {createDisplayTerrain} from './public/campus-terrain.js';
import {appearanceFor,buildingAppearances} from './public/campus-appearance.js';
const read=name=>JSON.parse(fs.readFileSync(new URL(`./public/data/${name}.json`,import.meta.url),'utf8'));
const campus=read('campus_source'),map=read('map'),provenance=read('campus_provenance');
const frame=createCampusFrame(campus,map);

test('수의학관의 지도명과 검색 별칭을 보완하고 OSM 원본은 보존한다',()=>{
  const before=JSON.stringify(campus),labelled=applyCampusLabels(campus);
  const original=campus.buildings.find(b=>b.id==='relation/16467729'),vet=labelled.buildings.find(b=>b.id===original.id);
  assert.equal(original.n,'');assert.equal(vet.n,'수의학관 · 수의과대학');
  for(const query of ['수의학관','수의과대학','수의대','수의 대','15번 건물'])assert.equal(campusBuildingMatches(vet,query),true);
  assert.equal(campusBuildingMatches(vet,''),false);
  assert.equal(campusBuildingMatches(vet,'학생회관'),false);
  assert.deepEqual(vet.o,original.o);assert.equal(vet.h,original.h);assert.equal(vet.g,original.g);
  assert.match(vet.name_source,/konkuk\.ac\.kr/);assert.equal(JSON.stringify(campus),before);
});

test('OSM 원본에서 직접 생성한 데이터와 해시가 일치한다',()=>{
  assert.equal(provenance.third_party_application_dataset_used,false);
  assert.equal(provenance.third_party_application_code_used,false);
  assert.match(provenance.api_source||provenance.source,/openstreetmap|OSM|overpass/i);
  assert.equal(crypto.createHash('sha256').update(fs.readFileSync(new URL('./public/data/campus_source.json',import.meta.url))).digest('hex'),provenance.sha256);
});
test('같은 학생회관 OSM 윤곽의 좌표 변환이 기존 지도와 일치한다',()=>{
  const old=map.vectors.osm.find(b=>b.id==='osm_way_367253036').xy_m;
  const differences=frame.student.o.map(p=>{const q=frame.toLocal(p);return Math.min(...old.map(r=>Math.hypot(r[0]-q[0],r[1]-q[1])));});
  assert.ok(Math.max(...differences)<.15,`최대 차이 ${Math.max(...differences)}m`);
});
test('임의 축척 조정 없이 미터 길이를 보존한다',()=>{
  const a=frame.toLocal([0,0]),b=frame.toLocal([30,40]);
  assert.equal(frame.scale,1);assert.ok(Math.abs(Math.hypot(a[0]-b[0],a[1]-b[1])-50)<.001);
});
test('공개 고도와 실내 바닥 기준을 분리한다',()=>{
  assert.ok(Math.abs(frame.toLocal([0,0],frame.student.g)[2]+1.55)<1e-8);
  assert.ok(Number.isFinite(frame.terrainHeight(0,0)));
  assert.equal(campus.terrain.dm.length,campus.terrain.nx*campus.terrain.nz);
});
test('배경 고도의 급상승을 완화하고 원본과 측위 좌표는 보존한다',()=>{
  const before=JSON.stringify(campus),surface=createDisplayTerrain(campus),g=campus.terrain;
  assert.equal(surface.diagnostics.rawMaxStep,62);
  assert.ok(surface.diagnostics.displayMaxStep<=g.step*.35+1e-8);
  assert.ok(surface.diagnostics.changedVertices>0);
  assert.equal(JSON.stringify(campus),before);
  for(let j=0;j<g.nz-1;j++)for(let i=0;i<g.nx-1;i++){
    const a=j*g.nx+i;
    for(const b of [a+g.nx+1,a+g.nx-1])if(b%g.nx>=0&&Math.abs((b%g.nx)-i)===1)assert.ok(Math.abs(surface.heights[a]-surface.heights[b])<=g.step*.35*Math.SQRT2+1e-8);
  }
  assert.equal(frame.scale,1);
});
test('수면과 호숫가를 같은 표시용 고도장에 연결한다',()=>{
  const surface=createDisplayTerrain(campus),g=campus.terrain;
  assert.equal(surface.waters.length,1);assert.equal(surface.waters[0].height,22);
  let count=0;
  for(let j=0;j<g.nz;j++)for(let i=0;i<g.nx;i++){
    const p=[g.x0+i*g.step,g.z0+j*g.step],r=surface.waters[0].land.r;
    if(insideRing(p,r[0])&&!r.slice(1).some(h=>insideRing(p,h))){assert.ok(surface.heights[j*g.nx+i]<22);count++;}
  }
  assert.ok(count>1000);
  const scene=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  assert.doesNotMatch(scene,/xyToHeight|waters\.some\(r=>insideRing\(\[x\+tg\.step/);
});
test('실사 참고 재질과 외관 미확인 건물을 구분한다',()=>{
  assert.equal(appearanceFor(frame.student).id,'student-brick-accent');
  assert.ok(new Set(Object.values(buildingAppearances).map(s=>s.pattern)).size>=3);
  assert.ok(Object.values(buildingAppearances).every(s=>s.evidence.length>0&&s.note));
  assert.equal(appearanceFor({id:'unobserved'}).id,'unverified-neutral');
});
test('배경 표면을 길찾기 연결로 사용하지 않는다',()=>{
  const navigation=read('navigation');
  assert.equal(navigation.recording_trajectory_used_for_routing,false);
  assert.deepEqual(navigation.portals.map(p=>p.id).sort(),['main_door','main_stairs','sub_stairs','sub_vestibule','elevator'].sort());
  assert.ok(navigation.portals.every(p=>p.field_verified===false));
  assert.ok(!navigation.portals.some(p=>p.id.startsWith('osm')));
});
test('게임 출처와 링크가 화면 및 현재 데이터에 남아 있지 않다',()=>{
  for(const name of ['index.html','app.js','data/campus_provenance.json']) {
    const text=fs.readFileSync(new URL(`./public/${name}`,import.meta.url),'utf8');
    assert.doesNotMatch(text,/hong-iron|smash.konkuk/i);
  }
  assert.ok(campus.buildings.length>40);assert.ok(campus.roads.length>100);
  assert.equal(insideRing([1,1],[[0,0],[2,0],[2,2],[0,2]]),true);
});
