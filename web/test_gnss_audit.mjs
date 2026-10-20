import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {GnssFilter,rtsSmooth} from './public/gnss-filter.js';
import {createGeoFrame} from './public/geo-frame.js';
import {loadMeasuredRecording,MEASURED_SESSIONS} from './public/gnss-recordings.js';
const read=n=>JSON.parse(fs.readFileSync(new URL(`./public/data/${n}.json`,import.meta.url),'utf8'));
const audit=read('gnss_slam_comparison'),frame=createGeoFrame(read('map'));

test('measured playback accepts only the five recorded DB sources and defaults to outside',async()=>{
 let requested='';
 const mock=id=>async url=>{requested=url;return {ok:true,json:async()=>read(`gnss_${id}_recorded`)};};
 const outside=await loadMeasuredRecording(undefined,mock('outside'));
 assert.equal(requested,'/data/gnss_outside_recorded.json');assert.equal(outside.records.length,338);
 for(const id of Object.keys(MEASURED_SESSIONS)){const value=await loadMeasuredRecording(id,mock(id));assert.equal(value.source,MEASURED_SESSIONS[id].source);assert.equal(value.simulated,false);}
 await assert.rejects(loadMeasuredRecording('__proto__',mock('outside')));
 await assert.rejects(loadMeasuredRecording('outside',async()=>({ok:true,json:async()=>read('gnss_demo')})));
 await assert.rejects(loadMeasuredRecording('outside',mock('lobby')));
 const code=fs.readFileSync(new URL('./public/location-controller.js',import.meta.url),'utf8');
 assert.match(code,/syntheticGnssTools/);assert.match(code,/recordingSource/);assert.match(code,/if\(state\.mode==='log'\)\{state\.route=null/);
});
test('five measured sessions keep unique fix times and explicit reference provenance',()=>{
 assert.equal(Object.keys(audit.sessions).length,5);assert.equal(audit.original_selected_pose_csv_available,false);
 let fixes=0,paired=0,photos=0;
 for(const [id,s] of Object.entries(audit.sessions)){
  const replay=read(`gnss_${id}_recorded`);assert.equal(replay.simulated,false);
  assert.equal(new Set(s.records.map(r=>r.gnss_stamp)).size,s.records.length);
  fixes+=s.records.length;paired+=s.records.filter(r=>r.reference).length;photos+=s.metrics.published_photo_checks;
  assert.ok(s.metrics.published_photo_max_difference_m<.0001);
  assert.equal(s.metrics.per_session_translation_applied,false);
  for(let i=0;i<s.records.length;i++)assert.ok(Math.abs(replay.records[i].timestamp/1000-s.records[i].gnss_stamp)<1e-6);
 }
 assert.equal(fixes,1099);assert.equal(paired,1098);assert.equal(photos,189);
});
test('replay filters reproduce all published common-epoch metrics, including poor indoor results',()=>{
 for(const s of Object.values(audit.sessions)){
  const f=new GnssFilter();for(const r of s.records){const xy=frame.fromGeo(r.latitude,r.longitude);assert.ok(Math.hypot(xy[0]-r.raw[0],xy[1]-r.raw[1])<1e-6);f.update({x:xy[0],y:xy[1],t:r.gnss_stamp,accuracy:r.reported_accuracy,accuracyConvention:'unspecified_radius'});}
  const sm=rtsSmooth(f.history),refs=new Map(s.records.filter(r=>r.reference).map(r=>[r.gnss_stamp,r.reference]));
  const rows=f.history.map((h,i)=>({h,sm:sm[i],ref:refs.get(h.t)})).filter(r=>r.ref);
  for(const [key,get] of [['raw_common_residual_m',r=>r.h.raw],['kalman_residual_m',r=>r.h.x],['rts_residual_m',r=>r.sm.position]]){
   const rmse=Math.sqrt(rows.reduce((a,r)=>{const p=get(r);return a+(p[0]-r.ref[0])**2+(p[1]-r.ref[1])**2;},0)/rows.length);
   assert.ok(Math.abs(rmse-s.metrics[key].rmse)<1e-7);
  }
 }
 assert.ok(audit.sessions.floor2.metrics.kalman_residual_m.rmse>audit.sessions.floor2.metrics.raw_common_residual_m.rmse);
});
test('global frames agree and no GNSS floor or fabricated accuracy is published',()=>{
 assert.ok(audit.coordinate_checks.python_vs_production_from_geo_max_m<1e-6);
 assert.ok(audit.coordinate_checks.campus_vs_geo_enu_max_m<1e-8);
 assert.ok(Math.abs(audit.coordinate_checks.rotation_determinant-1)<1e-10);
 assert.equal(audit.gnss_altitude_used,false);assert.equal(audit.georeference.indoor_gps_used,false);
 assert.equal(audit.coordinate_checks.unit_scale,1);
});
