// Map-aided PDR: sample walking direction, drift and stride, reject wall crossings.
// Independent JS implementation informed by APF_PDR (ISPRS IJGI 2025, 14, 476).
// Accepts referenced attitude or gravity-projected gyro heading from a separate front-end.
// Does not reproduce the original MATLAB cross-entropy adaptation criterion.
// Neither destinations nor a requested route are inputs to this estimator.
import {IndoorGrid} from './indoor-position.js';
import {StepDetector} from './walking-sensors.js';
const finite=Number.isFinite,rad=Math.PI/180;
const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const bound=(v,a,b)=>Math.max(a,Math.min(b,v));
export const PARTICLE_SETTINGS=Object.freeze({particles:1536,minParticles:768,maxParticles:2048,
  anchorSigma:.35,initialHeadingSigmaDeg:18,headingNoiseDeg:3,strideNoise:.10,
  biasSigmaDegPerSec:.12,biasWalkDegPerSec:.025,strideScaleSigma:.08,
  adaptive:true,estimateBias:true,clearanceWeight:true,recordAncestry:false,seed:872341});

function randomSource(seed){let state=seed>>>0;return ()=>{state=(state+0x6D2B79F5)>>>0;let x=state;
  x=Math.imul(x^(x>>>15),x|1);x^=x+Math.imul(x^(x>>>7),x|61);return ((x^(x>>>14))>>>0)/4294967296;};}

export class MapParticleTracker {
  constructor(navigation,position,level='1',options={}){
    this.settings={...PARTICLE_SETTINGS,...options};this.random=randomSource(this.settings.seed);
    this.grid=new IndoorGrid(navigation);this.detector=new StepDetector(options);
    this.sensorYaw=null;this.particles=[];this.fields={};this.reset(position,level);
  }
  normal(){const u=Math.max(1e-12,this.random());return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*this.random());}
  anchor(position,level){
    for(let i=0;i<40;i++){
      const p=[position[0]+this.normal()*this.settings.anchorSigma,position[1]+this.normal()*this.settings.anchorSigma];
      if(this.grid.clear(level,position,p))return p;
    }return position.slice(0,2);
  }
  reset(position,level){
    if(!position?.slice(0,2).every(finite)||!this.grid.free(level,position))throw Error('이 위치에서 안내를 시작할 수 없어요. 가까운 입구나 장소를 선택해 주세요.');
    const old=this.particles,previousYaw=this.stepSensorYaw;
    this.level=level;this.position=position.slice(0,2);this.steps=0;this.walked=0;this.blocked=0;this.rejected=0;
    this.lastMotionAt=0;this.stepAt=null;this.headingAt=0;this.headingReady=false;this.lost=false;
    this.stepSensorYaw=this.sensorYaw;this.detector.reset();this.estimate=null;
    this.particles=Array.from({length:this.settings.particles},(_,i)=>{
      const p=old.length?old[i%old.length]:null,[x,y]=this.anchor(position,level);
      return {x,y,yaw:p&&previousYaw!==null?wrap(this.sensorYaw+wrap(p.yaw-previousYaw)):null,
        bias:p?.bias??this.normal()*this.settings.biasSigmaDegPerSec*rad,
        scale:p?.scale??bound(1+this.normal()*this.settings.strideScaleSigma,.65,1.35),weight:1/this.settings.particles,
        node:this.settings.recordAncestry?{position:[x,y,this.grid.data.levels[level].z+.03],time:null,parent:null}:null};
    });
  }
  heading(vector,time,accuracy=15){
    if(!vector?.every(finite)||!finite(time)||!finite(accuracy)||accuracy<0||accuracy>30||time<this.headingAt)return false;
    const yaw=Math.atan2(vector[1],vector[0]);
    if(this.sensorYaw===null){
      this.sensorYaw=this.stepSensorYaw=yaw;
      for(const p of this.particles)p.yaw=wrap(yaw+this.normal()*this.settings.initialHeadingSigmaDeg*rad);
    }else if(!this.headingReady){
      const d=wrap(yaw-this.sensorYaw);
      for(const p of this.particles)p.yaw=p.yaw===null?wrap(yaw+this.normal()*this.settings.initialHeadingSigmaDeg*rad):wrap(p.yaw+d);
      this.stepSensorYaw=yaw;
    }
    this.sensorYaw=yaw;this.headingAt=time;this.headingReady=true;return true;
  }
  motion(event,time){
    const a=event.accelerationIncludingGravity;
    if(!a||![a.x,a.y,a.z].every(finite))return {moved:false,reason:'no_motion'};
    this.lastMotionAt=time;const step=this.detector.sample(Math.hypot(a.x,a.y,a.z),time);
    return step?this.step(step.length,time):{moved:false,reason:'waiting_step'};
  }
  clearance(level,p){
    // Local free-cell support, not attraction to a destination or corridor route.
    if(!this.settings.clearanceWeight)return 1;
    const [r,c]=this.grid.cellAt(p);let n=0;
    for(const [dr,dc]of [[0,1],[0,-1],[1,0],[-1,0],[1,1],[-1,-1],[1,-1],[-1,1]])if(this.grid.freeCell(level,r+dr,c+dc))n++;
    return .6+.4*n/8;
  }
  resample(items,count){
    const sum=items.reduce((s,p)=>s+p.weight,0),output=[],step=sum/count;
    let mark=this.random()*step,index=0,cumulative=items[0].weight;
    for(let i=0;i<count;i++,mark+=step){while(index<items.length-1&&mark>cumulative)cumulative+=items[++index].weight;
      output.push({...items[index],weight:1/count});
    }return output;
  }
  posterior(items){
    const sum=items.reduce((s,p)=>s+p.weight,0);
    let x=0,y=0,c=0,s=0,bias=0,scale=0;
    for(const p of items){const w=p.weight/sum;x+=w*p.x;y+=w*p.y;c+=w*Math.cos(p.yaw);s+=w*Math.sin(p.yaw);bias+=w*p.bias;scale+=w*p.scale;}
    let variance=0;for(const p of items)variance+=p.weight/sum*((p.x-x)**2+(p.y-y)**2);
    // A posterior mean may sit in a wall between two surviving hypotheses.
    // Display a real high-support particle near the mean, never that wall point.
    let best=items[0],bestCost=Infinity;
    for(const p of items){const cost=(p.x-x)**2+(p.y-y)**2;
      if(cost<bestCost){bestCost=cost;best=p;}}
    return {mean:[x,y],representative:best,yaw:Math.atan2(s,c),variance,
      headingSpread:Math.sqrt(Math.max(0,-2*Math.log(Math.max(1e-9,Math.hypot(c,s))))),bias,scale};
  }
  step(length,time){
    if(!finite(length)||length<.3||length>1.1||!finite(time))return {moved:false,reason:'invalid'};
    if(!this.headingReady||time-this.headingAt>4000)return {moved:false,reason:'heading'};
    const s=this.settings,change=wrap(this.sensorYaw-this.stepSensorYaw),dt=this.stepAt===null?.55:bound((time-this.stepAt)/1000,.1,2);
    const proposed=[];
    for(const p of this.particles){
      const bias=s.estimateBias?bound(p.bias+this.normal()*s.biasWalkDegPerSec*rad,-1.5*rad,1.5*rad):0;
      const scale=s.estimateBias?bound(p.scale+this.normal()*.008,.65,1.35):1;
      const yaw=wrap(p.yaw+change+bias*dt+this.normal()*s.headingNoiseDeg*rad);
      const distance=bound(length*scale+this.normal()*s.strideNoise,.30,1.1);
      const x=p.x+distance*Math.cos(yaw),y=p.y+distance*Math.sin(yaw);
      if(!this.grid.clear(this.level,[p.x,p.y],[x,y]))continue;
      proposed.push({x,y,yaw,bias,scale,weight:p.weight*this.clearance(this.level,[x,y]),
        node:s.recordAncestry?{position:[x,y,this.grid.data.levels[this.level].z+.03],time,parent:p.node}:null});
    }
    this.steps++;this.stepAt=time;this.stepSensorYaw=this.sensorYaw;
    this.rejected+=this.particles.length-proposed.length;
    if(!proposed.length){this.blocked++;this.lost=true;
      // Keep the last posterior position, propagate only orientation. A recovery
      // never teleports particles through a wall or invents a zero-length step.
      for(const p of this.particles)p.yaw=wrap(p.yaw+change+this.normal()*8*rad);
      return {moved:false,reason:'wall'};
    }
    const estimate=this.posterior(proposed),previous=this.position;
    this.estimate=estimate;let display=estimate.representative;
    const reachable=proposed.filter(p=>distance(previous,[p.x,p.y])<=length*1.8+.4&&this.grid.clear(this.level,previous,[p.x,p.y]));
    if(reachable.length){display=reachable.reduce((a,b)=>distance([a.x,a.y],estimate.mean)<distance([b.x,b.y],estimate.mean)?a:b);this.position=[display.x,display.y];this.blocked=0;}
    else this.blocked++;
    this.walked+=distance(previous,this.position);this.lost=this.blocked>=3;
    let count=s.particles;
    if(s.adaptive)count=estimate.variance<.35&&estimate.headingSpread<.15?s.minParticles:s.maxParticles;
    const weightSum=proposed.reduce((v,p)=>v+p.weight,0);
    this.effectiveParticles=1/proposed.reduce((v,p)=>v+(p.weight/weightSum)**2,0);
    this.particles=this.resample(proposed,count);
    return {moved:distance(previous,this.position)>.01,reason:this.lost?'uncertain':'step',...this.snapshot()};
  }
  snapshot(){
    const estimate=this.estimate;
    return {position:[...this.position,this.grid.data.levels[this.level].z+.03],level:this.level,
      yaw:estimate?.yaw??this.sensorYaw??0,steps:this.steps,walked:this.walked,
      uncertainty:Math.max(1.5,2.45*Math.sqrt(estimate?.variance??.36)),
      headingReady:this.headingReady,headingAt:this.headingAt,lastMotionAt:this.lastMotionAt,blocked:this.blocked,
      method:'map_aided_particle_pdr',particleCount:this.particles.length,effectiveParticles:this.effectiveParticles??this.particles.length,
      estimatedHeadingBiasDegPerSec:(estimate?.bias??0)/rad,estimatedStrideScale:estimate?.scale??1,
      lost:this.lost,rejectedParticles:this.rejected};
  }
  ancestry(){
    if(!this.particles.length)return [];
    const path=[];let node=this.posterior(this.particles).representative.node;
    for(;node;node=node.parent)path.push({time:node.time,position:node.position});
    return path.reverse();
  }
}
