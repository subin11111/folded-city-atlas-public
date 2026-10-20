// Camera-free pedestrian estimates. A confirmed landmark supplies position;
// live steps and a referenced device heading supply motion, never the planned route.
import {mul,add,transpose} from './gnss-filter.js';
import {StepDetector} from './walking-sensors.js';
export {StepDetector} from './walking-sensors.js';
const finite=Number.isFinite,rad=Math.PI/180;
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const dist=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);

export class IndoorGrid {
  constructor(data){
    this.data=data;this.cell=data.cell_m;this.origin=data.origin_xy;[this.rows,this.cols]=data.shape;
    this.grids=Object.fromEntries(Object.entries(data.levels).map(([lv,v])=>[lv,Uint8Array.from(atob(v.walkable_bits),c=>c.charCodeAt(0))]));
  }
  cellAt(p){return [Math.round((p[1]-this.origin[1])/this.cell),Math.round((p[0]-this.origin[0])/this.cell)];}
  freeCell(level,r,c){const i=r*this.cols+c;return !!(r>=0&&r<this.rows&&c>=0&&c<this.cols&&this.grids[level]?.[i>>3]&(1<<(i&7)));}
  onDoor(level,p){
    return this.data.portals.some(portal=>portal.kind==='door'&&portal.a[0]===level&&portal.b[0]===level&&portal.polyline.some((b,i,pts)=>{
      if(!i)return false;const a=pts[i-1],dx=b[0]-a[0],dy=b[1]-a[1],d=dx*dx+dy*dy;
      const u=d?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d)):0;
      return Math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)<.28;
    }));
  }
  free(level,p){return this.freeCell(level,...this.cellAt(p))||this.onDoor(level,p);}
  clear(level,a,b){
    const count=Math.max(1,Math.ceil(dist(a,b)/(this.cell*.4)));let previous=this.cellAt(a);
    for(let i=0;i<=count;i++){
      const p=[a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count],cell=this.cellAt(p);
      if(!this.free(level,p))return false;
      if(cell[0]!==previous[0]&&cell[1]!==previous[1]&&!this.onDoor(level,p)){
        if(!this.freeCell(level,cell[0],previous[1])||!this.freeCell(level,previous[0],cell[1]))return false;
      }
      previous=cell;
    }return true;
  }
  constrain(level,previous,proposed){
    if(this.clear(level,previous,proposed))return proposed;
    const [r,c]=this.cellAt(proposed),candidates=[],radius=3;
    for(let dr=-radius;dr<=radius;dr++)for(let dc=-radius;dc<=radius;dc++){
      if(!this.freeCell(level,r+dr,c+dc))continue;
      const p=[this.origin[0]+(c+dc)*this.cell,this.origin[1]+(r+dr)*this.cell];
      if(dist(p,proposed)<=.36&&dist(p,previous)>.1&&this.clear(level,previous,p))candidates.push(p);
    }
    candidates.sort((a,b)=>dist(a,proposed)-dist(b,proposed));return candidates[0]||null;
  }
}

export class PedestrianTracker {
  constructor(navigation,position,level='1',options={}){
    this.grid=new IndoorGrid(navigation);this.detector=new StepDetector(options);this.reset(position,level);
  }
  reset(position,level){
    if(!this.grid.grids[level]||!position?.slice(0,2).every(finite)||!this.grid.free(level,position))throw Error('이 위치에서 안내를 시작할 수 없어요. 가까운 입구나 장소를 선택해 주세요.');
    this.x=[position[0],position[1],0];this.P=[[.36,0,0],[0,.36,0],[0,0,(20*rad)**2]];
    this.level=level;this.headingAt=0;this.headingReady=false;this.headingCandidate=null;this.lastMotionAt=0;this.steps=0;this.blocked=0;this.walked=0;this.detector.reset();
  }
  heading(vector,time,accuracy=15){
    if(!vector?.every(finite)||!finite(time)||!finite(accuracy)||accuracy<0||accuracy>30)return false;
    const yaw=Math.atan2(vector[1],vector[0]);
    if(!this.headingReady){
      if(this.headingAt){
        if(!this.headingCandidate||Math.abs(wrap(yaw-this.headingCandidate.yaw))>15*rad){this.headingCandidate={yaw,at:time};return false;}
        if(time-this.headingCandidate.at<700)return false;
      }
      this.x[2]=yaw;this.headingReady=true;this.headingCandidate=null;
    }
    const variance=(Math.max(8,accuracy)*rad)**2,innovation=wrap(yaw-this.x[2]);
    // Compass disturbances must not immediately swing the walking direction.
    if(time-this.headingAt<1500&&Math.abs(innovation)>75*rad){this.headingReady=false;this.headingCandidate={yaw,at:time};return false;}
    const K=this.P[2][2]/(this.P[2][2]+variance);
    this.x[2]=wrap(this.x[2]+K*innovation);
    // Repeated compass events are correlated. Do not gain fictitious position
    // accuracy or translate the pedestrian merely because the phone rotates.
    this.P[2][2]=Math.max((8*rad)**2,(1-K)**2*this.P[2][2]+K*K*variance);
    this.P[0][2]=this.P[1][2]=this.P[2][0]=this.P[2][1]=0;
    this.headingAt=time;return true;
  }
  motion(event,time){
    const a=event.accelerationIncludingGravity;
    if(!a||![a.x,a.y,a.z].every(finite))return {moved:false,reason:'no_motion'};
    this.lastMotionAt=time;
    const step=this.detector.sample(Math.hypot(a.x,a.y,a.z),time);
    return step?this.step(step.length,time):{moved:false,reason:'waiting_step'};
  }
  step(length,time){
    if(!finite(length)||length<.3||length>1.1||!finite(time))return {moved:false,reason:'invalid'};
    if(!this.headingReady||time-this.headingAt>4000)return {moved:false,reason:'heading'};
    if(this.snapshot().uncertainty>7)return {moved:false,reason:'uncertainty'};
    const [x,y,yaw]=this.x,c=Math.cos(yaw),s=Math.sin(yaw);
    const F=[[1,0,-length*s],[0,1,length*c],[0,0,1]];
    const Q=[[.0225,0,0],[0,.0225,0],[0,0,(4*rad)**2]];
    this.P=add(mul(mul(F,this.P),transpose(F)),Q);this.steps++;
    const next=this.grid.constrain(this.level,[x,y],[x+length*c,y+length*s]);
    if(!next){this.blocked++;return {moved:false,reason:'wall'};}
    this.x[0]=next[0];this.x[1]=next[1];this.walked+=dist([x,y],next);this.blocked=0;
    return {moved:true,reason:'step',...this.snapshot()};
  }
  snapshot(){
    const [a,b]=this.P[0],c=this.P[1][1],eigen=(a+c+Math.sqrt((a-c)**2+4*b*b))/2;
    return {position:[this.x[0],this.x[1],this.grid.data.levels[this.level].z+.03],level:this.level,
      yaw:this.x[2],steps:this.steps,walked:this.walked,uncertainty:Math.max(1.5,2.45*Math.sqrt(eigen)+.025*this.steps),
      headingReady:this.headingReady,headingAt:this.headingAt,lastMotionAt:this.lastMotionAt,blocked:this.blocked};
  }
}

export function indoorProgress(snapshot,route){
  const nodes=route?.route||[],level=snapshot.level;let total=0,best=null;
  for(let i=1;i<nodes.length;i++){
    const a=nodes[i-1],b=nodes[i],length=Math.hypot(...b.position.map((v,j)=>v-a.position[j]));
    if(a.level===level&&b.level===level&&!['stairs','elevator'].includes(b.kind)){
      const dx=b.position[0]-a.position[0],dy=b.position[1]-a.position[1],den=dx*dx+dy*dy;
      const u=den?Math.max(0,Math.min(1,((snapshot.position[0]-a.position[0])*dx+(snapshot.position[1]-a.position[1])*dy)/den)):0;
      const point=[a.position[0]+u*dx,a.position[1]+u*dy],cross=dist(point,snapshot.position);
      if(!best||cross<best.cross)best={index:i,cross,along:total+u*length};
    }total+=length;
  }
  const transition=(route?.segments||[]).find(segment=>['stairs','elevator'].includes(segment.kind)&&nodes[segment.start_index]?.level===level&&dist(snapshot.position,nodes[segment.start_index].position)<=2);
  const end=nodes.at(-1),arrived=!!end&&end.level===level&&dist(snapshot.position,end.position)<=1.5&&(!best||best.cross<=2);
  return {remaining:best?Math.max(0,total-best.along):null,crossTrack:best?.cross??Infinity,index:best?.index??0,transition,arrived};
}

// Call both iOS permission methods synchronously from the tap handler, before
// any network request/await can consume transient user activation.
export async function requestWalkingSensors(host=globalThis){
  const requests=['DeviceMotionEvent','DeviceOrientationEvent'].map(name=>{
    const type=host[name];if(!type)return Promise.resolve(false);
    try{return typeof type.requestPermission==='function'?Promise.resolve(name==='DeviceOrientationEvent'?type.requestPermission(true):type.requestPermission()).then(v=>v==='granted',()=>false):Promise.resolve(true);}catch{return Promise.resolve(false);}
  });
  const [motion,orientation]=await Promise.all(requests);return {motion,orientation};
}
