// Offline audit only: import production code; never alter operational data.
import fs from 'node:fs';
import crypto from 'node:crypto';
import {GnssFilter,rtsSmooth,FILTER_SETTINGS} from '../public/gnss-filter.js';

const repo=new URL('../../',import.meta.url);
const input=new URL('docs/assets/gnss-slam-audit-20261020/comparison.json',repo);
const target=new URL('docs/assets/critical-review-20261021/gnss-settings-replay.json',repo);
const source=JSON.parse(fs.readFileSync(input,'utf8'));
const result={source_sha256:crypto.createHash('sha256').update(fs.readFileSync(input)).digest('hex'),
  filter_sha256:crypto.createHash('sha256').update(fs.readFileSync(new URL('../public/gnss-filter.js',import.meta.url))).digest('hex'),
  settings:FILTER_SETTINGS,reference:'Same-epoch web SLAM pose proxy; no surveyed ground truth',
  interpretation:'unspecified_radius is the iOS source convention; browser_95_assumption is sensitivity analysis, not a new browser field trial',sessions:{}};
for(const [name,s] of Object.entries(source.sessions)){
  result.sessions[name]={};
  for(const convention of ['unspecified_radius','browser_95_assumption']){
    const filter=new GnssFilter();
    for(const r of s.records)filter.update({x:r.raw[0],y:r.raw[1],t:r.gnss_stamp,
      accuracy:r.reported_accuracy,accuracyConvention:convention});
    const smooth=rtsSmooth(filter.history);
    const lookup=new Map(s.records.map(r=>[r.gnss_stamp,r]));
    const rows=filter.history.filter(h=>lookup.get(h.t)?.reference).map(h=>{
      const i=filter.history.indexOf(h),r=lookup.get(h.t);
      return {t:h.t,raw:r.raw,reference:r.reference.slice(0,2),kalman:h.x.slice(0,2),rts:smooth[i].position};
    });
    const rms=a=>Math.sqrt(a.reduce((s,v)=>s+v*v,0)/a.length);
    const metrics={};
    for(const kind of ['raw','kalman','rts']){
      const errors=rows.map(r=>r[kind].map((v,i)=>v-r.reference[i]));
      const bias=[0,1].map(i=>errors.reduce((s,e)=>s+e[i],0)/errors.length);
      const centered=errors.map(e=>Math.hypot(e[0]-bias[0],e[1]-bias[1]));
      const velocityErrors=[];
      for(let i=1;i<rows.length;i++){
        const dt=rows[i].t-rows[i-1].t;
        if(dt>0&&dt<=5)velocityErrors.push(Math.hypot(...[0,1].map(j=>
          ((rows[i][kind][j]-rows[i-1][kind][j])-(rows[i].reference[j]-rows[i-1].reference[j]))/dt)));
      }
      metrics[kind]={n:errors.length,relative_rmse_m:rms(errors.map(e=>Math.hypot(...e))),
        mean_delta_map_xy_m:bias,bias_norm_m:Math.hypot(...bias),
        mean_removed_rmse_m:rms(centered),relative_velocity_rmse_m_s:rms(velocityErrors),
        velocity_intervals:velocityErrors.length,bias_subtraction_used_for_diagnostics_only:true};
    }
    result.sessions[name][convention]={accepted:filter.history.filter(h=>h.accepted).length,
      history:filter.history.length,metrics};
  }
}
fs.mkdirSync(new URL('.',target),{recursive:true});
fs.writeFileSync(target,JSON.stringify(result,null,2)+'\n');
for(const [name,s] of Object.entries(result.sessions))console.log(JSON.stringify({name,...s}));
