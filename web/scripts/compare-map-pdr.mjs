// Reproducible current-map comparisons. No destination, reference path or GNSS fit.
import fs from 'node:fs';
import crypto from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {performance} from 'node:perf_hooks';
import {StepDetector,WalkingHeading} from '../public/walking-sensors.js';
import {GravityHeading} from '../public/gravity-heading.js';
import {MapParticleTracker} from '../public/map-particle-filter.js';
import {PedestrianTracker,IndoorGrid} from '../public/indoor-position.js';
import {createGeoFrame} from '../public/geo-frame.js';
const arg=name=>{const i=process.argv.indexOf(name);return i<0?null:process.argv[i+1];};
const repo=new URL('../../',import.meta.url),out=new URL('docs/assets/map-pdr-20261021/',repo);
fs.mkdirSync(out,{recursive:true});
const read=p=>JSON.parse(fs.readFileSync(p,'utf8')),hash=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const nav=read(new URL('web/public/data/navigation.json',repo)),map=read(new URL('web/public/data/map.json',repo));
const places=read(new URL('web/public/data/catalog.json',repo)),frame=createGeoFrame(map),grid=new IndoorGrid(nav);
const entry=places.find(p=>p.id==='entrance'),stairs=nav.portals.find(p=>p.id==='main_stairs'),prime=places.find(p=>p.id==='prime_hall');
const bearing=v=>{const e=frame.headingVector(90),n=frame.headingVector(0);return (Math.atan2(v[0]*e[0]+v[1]*e[1],v[0]*n[0]+v[1]*n[1])*180/Math.PI+360)%360;};
const door=nav.portals.find(p=>p.id==='main_door').polyline;
const entryBearing=bearing([door.at(-1)[0]-door[0][0],door.at(-1)[1]-door[0][1]]);
const stairBearing=bearing([stairs.polyline.at(-1)[0]-stairs.polyline.at(-2)[0],stairs.polyline.at(-1)[1]-stairs.polyline.at(-2)[1]]);
const observationsURL=new URL('observations.json',out);
if(arg('--native')){
  const input=read(arg('--native')),scenarios=[];
  for(const floorStart of [66,68.841865234375,70.295]){
    const heading=new WalkingHeading(entryBearing),gyro=new GravityHeading();let detector=new StepDetector(),phase=0,next=0;
    const rows=[];
    for(const row of input.samples){
      const [t,x,y,z,lx,ly,lz,wx,wy,wz,alpha,beta,gamma]=row;
      if(t<30.617)continue;
      const pose=heading.sample({alpha,beta,gamma,absolute:false},t*1000);
      if(pose)gyro.anchor(pose.bearing);
      const motion={accelerationIncludingGravity:{x,y,z},acceleration:{x:lx,y:ly,z:lz},rotationRate:{alpha:wz,beta:wx,gamma:wy}};
      const gh=gyro.sample(motion,t*1000);
      if(t>=floorStart&&phase!==2){phase=2;detector=new StepDetector();}
      else if(t>=48.5757434&&phase!==2)phase=1;
      const step=phase===1?null:detector.sample(Math.hypot(x,y,z),t*1000);
      if(t<next&&!step)continue;next=t+.05;
      rows.push([t,pose?.bearing??null,gh?.bearing??null,step?.length??null,phase]);
    }
    scenarios.push({floorStart,rows});
  }
  fs.writeFileSync(observationsURL,JSON.stringify({source_sha256:input.source_sha256,native_recording_not_safari:true,
    columns:['seconds','entry_referenced_attitude_bearing_deg','gravity_gyro_bearing_deg','detected_step_m_or_null','phase_0_1f_1_stairs_2_2f'],
    entry_s:30.617,stairs_start_s:48.5757434,entryBearing,stairBearing,
    floor_start_note:'Timing sensitivity, inferred from barometer and photos; not recorded button presses.',scenarios},null,0)+'\n');
}
const observations=read(observationsURL);
const configs=[
  {id:'ekf_map',name:'보행 EKF + 벽 제약',factory:(p,l)=>new PedestrianTracker(nav,p,l)},
  {id:'bootstrap_pf',name:'고정 입자 PF',factory:(p,l,seed)=>new MapParticleTracker(nav,p,l,{seed,recordAncestry:true,particles:384,adaptive:false,estimateBias:false,clearanceWeight:false})},
  {id:'adaptive_pf',name:'지도 보조 적응형 PF',factory:(p,l,seed)=>new MapParticleTracker(nav,p,l,{seed,recordAncestry:true,particles:512,minParticles:256,maxParticles:768})},
  {id:'wide_pf',name:'넓은 방향 사전 PF',factory:(p,l,seed)=>new MapParticleTracker(nav,p,l,{seed,recordAncestry:true,particles:512,minParticles:256,maxParticles:768,initialHeadingSigmaDeg:32,headingNoiseDeg:4})},
  {id:'dense_pf',name:'고밀도 적응형 PF',factory:(p,l,seed)=>new MapParticleTracker(nav,p,l,{seed,recordAncestry:true,particles:1536,minParticles:768,maxParticles:2048})},
];
const median=values=>{const a=[...values].sort((x,y)=>x-y),m=Math.floor(a.length/2);return a.length%2?a[m]:(a[m-1]+a[m])/2;};
const runs=[];
for(const scenario of observations.scenarios)for(const frontend of ['attitude','gyro','gyro_stair_anchor'])for(const cfg of configs)for(const seed of [193,893,20261021,872341]){
  const tracker=cfg.factory(entry.position,'1',seed),trace=[],timings=[];let floor=false,headingOffset=0;
  const phase={first:{steps:0,moved:0,rejected:0},second:{steps:0,moved:0,rejected:0}},failures={};
  for(const [t,att,gyro,length,mode]of scenario.rows){
    let head=frontend==='attitude'?att:gyro;
    if(mode===2&&!floor){
      tracker.reset(stairs.polyline.at(-1),'2');floor=true;
      if(frontend==='gyro_stair_anchor'&&head!==null)headingOffset=observations.stairBearing-head;
    }
    if(head!==null)tracker.heading(frame.headingVector(head+headingOffset),t*1000,15);
    if(length===null||mode===1)continue;
    const before=performance.now(),r=tracker.step(length,t*1000);timings.push(performance.now()-before);
    const s=tracker.snapshot(),p=floor?phase.second:phase.first;p.steps++;p.moved+=Number(r.moved);p.rejected+=Number(!r.moved);
    failures[r.reason]=(failures[r.reason]||0)+1;
    trace.push({t,position:s.position,level:s.level,uncertainty:s.uncertainty,moved:r.moved,reason:r.reason,
      yaw:s.yaw,particles:s.particleCount??null,strideScale:s.estimatedStrideScale??null});
  }
  const second=trace.filter(p=>p.level==='2'),positions=second.map(p=>p.position);
  const extent=positions.length?Math.hypot(Math.max(...positions.map(p=>p[0]))-Math.min(...positions.map(p=>p[0])),Math.max(...positions.map(p=>p[1]))-Math.min(...positions.map(p=>p[1]))):0;
  let wallCrossings=0,jumps=0;for(let i=1;i<second.length;i++){
    if(!grid.clear('2',second[i-1].position,second[i].position))wallCrossings++;
    if(Math.hypot(second[i].position[0]-second[i-1].position[0],second[i].position[1]-second[i-1].position[1])>2)jumps++;
  }
  const photo=second.reduce((a,b)=>!a||Math.abs(b.t-70.295)<Math.abs(a.t-70.295)?b:a,null);
  const ancestry=tracker.ancestry?.().map(p=>({t:p.time===null?scenario.floorStart:p.time/1000,position:p.position}))??[];
  const paused=second.filter(p=>p.uncertainty>7).length;
  const usable=second.filter(p=>p.moved&&p.uncertainty<=7).length;
  runs.push({config:cfg.id,name:cfg.name,frontend,seed,floorStart:scenario.floorStart,phase,failures,extent,wallCrossings,jumps,paused,usable,
    medianStepMs:median(timings),maxStepMs:Math.max(...timings),final:tracker.snapshot(),
    prime_photo_diagnostic:photo?{t:photo.t,distance_to_registered_photo_point_m:Math.hypot(photo.position[0]-prime.position[0],photo.position[1]-prime.position[1]),not_survey_error:true}:null,
    trace,ancestry,uncertainty_mean:second.reduce((s,p)=>s+p.uncertainty,0)/second.length});
  console.log(JSON.stringify({config:cfg.id,frontend,seed,floorStart:scenario.floorStart,second:phase.second,extent,wallCrossings,jumps}));
}
const result={source_sha256:observations.source_sha256,selection_uses_destination:false,ground_truth_available:false,
  protected_map_sha256:Object.fromEntries(['map','navigation','catalog'].map(n=>[n,hash(new URL('web/public/data/'+n+'.json',repo))])),
  source_sha256_by_module:Object.fromEntries(['map-particle-filter','gravity-heading','walking-sensors','indoor-position'].map(n=>[n,hash(new URL('web/public/'+n+'.js',repo))])),
  runs,groups:[]};
for(const cfg of configs)for(const frontend of ['attitude','gyro','gyro_stair_anchor']){
  const g=runs.filter(r=>r.config===cfg.id&&r.frontend===frontend);
  result.groups.push({config:cfg.id,name:cfg.name,frontend,n:g.length,medianRejected:median(g.map(r=>r.phase.second.rejected)),
    medianMoved:median(g.map(r=>r.phase.second.moved)),medianExtent:median(g.map(r=>r.extent)),
    medianUncertainty:median(g.map(r=>r.uncertainty_mean)),maxWallCrossings:Math.max(...g.map(r=>r.wallCrossings)),
    maxJumps:Math.max(...g.map(r=>r.jumps)),medianStepMs:median(g.map(r=>r.medianStepMs)),
    medianUsable:median(g.map(r=>r.usable)),minUsable:Math.min(...g.map(r=>r.usable)),
    // Photo location is an after-the-fact diagnostic, not a selection objective.
    medianPhotoDistance:median(g.map(r=>r.prime_photo_diagnostic?.distance_to_registered_photo_point_m??Infinity))});
}
fs.writeFileSync(new URL('comparison.json',out),JSON.stringify(result)+'\n');
console.log(JSON.stringify(result.groups,null,2));
