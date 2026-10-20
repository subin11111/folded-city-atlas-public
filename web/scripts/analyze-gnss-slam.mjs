// Run exactly the production filter, coordinate frames and RTS on every measured session.
import fs from 'node:fs';
import {GnssFilter,rtsSmooth} from '../public/gnss-filter.js';
import {createGeoFrame} from '../public/geo-frame.js';
import {createCampusFrame} from '../public/campus-data.js';
const out=new URL('../../docs/assets/gnss-slam-audit-20261020/',import.meta.url);
const audit=JSON.parse(fs.readFileSync(new URL('paired-input.json',out),'utf8'));
const read=name=>JSON.parse(fs.readFileSync(new URL(`../public/data/${name}.json`,import.meta.url),'utf8'));
const map=read('map'),campus=read('campus_source'),frame=createGeoFrame(map),cf=createCampusFrame(campus,map);
const norm=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
function stats(values){
  const a=values.filter(Number.isFinite).sort((a,b)=>a-b),n=a.length;
  const quantile=q=>{const p=(n-1)*q,i=Math.floor(p);return a[i]+(a[Math.min(i+1,n-1)]-a[i])*(p-i);};
  return n?{n,median:quantile(.5),p90:quantile(.9),max:a.at(-1),rmse:Math.sqrt(a.reduce((s,x)=>s+x*x,0)/n)}:{n:0};
}
function fromEN(x,y){const g=map.georeference,R=g.R_map_xy_to_EN,t=g.t_map_xy_to_EN_m,o=map.manual_preview_offset_en_m||[2,-2];return [R[0][0]*(x-t[0]-o[0])+R[1][0]*(y-t[1]-o[1]),R[0][1]*(x-t[0]-o[0])+R[1][1]*(y-t[1]-o[1])];}
let crossFrame=0,pythonFrame=0,roundTrip=0;
const summary={reference:audit.reference,original_selected_pose_csv_available:false,absolute_survey_rmse:null,manual_preview_offset_en_m:audit.manual_preview_offset_en_m,sessions:{}};
const features=[];
for(const [id,s] of Object.entries(audit.sessions)){
  const f=new GnssFilter(),statuses=[],rows=s.records;
  for(const r of rows){
    const p=frame.fromGeo(r.latitude,r.longitude);pythonFrame=Math.max(pythonFrame,norm(p,r.raw));
    const inv=frame.toGeo(...r.raw);roundTrip=Math.max(roundTrip,norm(frame.fromGeo(inv.latitude,inv.longitude),r.raw));
    const state=f.update({x:p[0],y:p[1],t:r.gnss_stamp,accuracy:r.reported_accuracy,accuracyConvention:'unspecified_radius'});
    statuses.push({t:r.gnss_stamp,accepted:state.accepted,reason:state.reason});
  }
  const smooth=rtsSmooth(f.history),states=new Map(f.history.map((h,i)=>[h.t,{h,rts:smooth[i]}]));
  const common=[],held=[],accept={};let last=null;
  for(const r of rows){
    const state=states.get(r.gnss_stamp),status=statuses.find(v=>v.t===r.gnss_stamp);
    r.accepted=status.accepted;r.reason=status.reason;
    if(state){r.kalman=state.h.x.slice(0,2);r.rts=state.rts.position;r.nis=state.h.nis;r.segment=state.h.segment;}
    if(status.accepted&&state)last=r.kalman;
    r.ui_position=last;
    if(r.reference&&state)common.push(r);
    if(r.reference&&last)held.push(norm(last,r.reference));
    accept[status.reason]=(accept[status.reason]||0)+1;
  }
  const metrics={...s.metrics,filter_common_epochs:common.length,filter_status_counts:accept,
    raw_common_residual_m:stats(common.map(r=>norm(r.raw,r.reference))),
    kalman_residual_m:stats(common.map(r=>norm(r.kalman,r.reference))),
    rts_residual_m:stats(common.map(r=>norm(r.rts,r.reference))),
    ui_last_good_residual_m:stats(held),
    reference_provenance:'DB Admin.opt_poses + portfolio O_FULL gauge + selected rigid placement + keyframe translation interpolation; verified against all 189 web photograph positions; not original selected CSV'};
  s.metrics=metrics;summary.sessions[id]={label:s.label,source:s.source,sha256:s.sha256,...metrics};
  features.push({type:'Feature',properties:{session:id,kind:'original_GNSS',independent_ground_truth:false},geometry:{type:'LineString',coordinates:rows.map(r=>[r.longitude,r.latitude])}});
  features.push({type:'Feature',properties:{session:id,kind:'SLAM_web_reference_coarse_georeference',independent_ground_truth:false},geometry:{type:'LineString',coordinates:s.reference_trajectory.map(r=>{const p=frame.toGeo(...r.position);return [p.longitude,p.latitude];})}});
  console.log(JSON.stringify({session:id,n:common.length,raw:metrics.raw_common_residual_m.rmse,kalman:metrics.kalman_residual_m.rmse,rts:metrics.rts_residual_m.rmse,status:accept}));
}
for(const p of [[0,0],[100,0],[0,100],[-250,350]]){const a=cf.toLocal([p[0],-p[1]]),b=fromEN(...p);crossFrame=Math.max(crossFrame,norm(a,b));}
summary.coordinate_checks={python_vs_production_from_geo_max_m:pythonFrame,campus_vs_geo_enu_max_m:crossFrame,
  current_approximate_inverse_roundtrip_max_m:roundTrip,
  rotation_determinant:map.georeference.R_map_xy_to_EN[0][0]*map.georeference.R_map_xy_to_EN[1][1]-map.georeference.R_map_xy_to_EN[0][1]*map.georeference.R_map_xy_to_EN[1][0],
  unit_scale:map.georeference.scale,preview_offset_length_m:Math.hypot(...audit.manual_preview_offset_en_m)};
if(pythonFrame>1e-6||crossFrame>1e-8)throw Error('Inconsistent coordinate implementation');
audit.coordinate_checks=summary.coordinate_checks;
fs.writeFileSync(new URL('summary.json',out),JSON.stringify(summary,null,2)+'\n');
fs.writeFileSync(new URL('comparison.json',out),JSON.stringify(audit,null,2)+'\n');
fs.writeFileSync(new URL('comparison.geojson',out),JSON.stringify({type:'FeatureCollection',features})+'\n');
// Compact derived evidence served by the independent comparison page, never used as live input.
fs.writeFileSync(new URL('../public/data/gnss_slam_comparison.json',import.meta.url),JSON.stringify(audit)+'\n');
console.log(JSON.stringify(summary.coordinate_checks));
