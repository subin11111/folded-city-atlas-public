// Constant-velocity Kalman filter in a local metre frame. No road or floor snapping.
export const transpose=a=>a[0].map((_,j)=>a.map(r=>r[j]));
export const mul=(a,b)=>a.map(r=>b[0].map((_,j)=>r.reduce((s,v,k)=>s+v*b[k][j],0)));
export const add=(a,b,sign=1)=>a.map((r,i)=>r.map((v,j)=>v+sign*b[i][j]));
export const eye=n=>Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>+(i===j)));
export function inverse(a){
  const n=a.length,m=a.map((r,i)=>[...r,...eye(n)[i]]);
  for(let k=0;k<n;k++){
    let p=k;for(let i=k+1;i<n;i++)if(Math.abs(m[i][k])>Math.abs(m[p][k]))p=i;
    [m[k],m[p]]=[m[p],m[k]];const d=m[k][k];if(Math.abs(d)<1e-12)throw Error('Singular covariance');
    m[k]=m[k].map(v=>v/d);
    for(let i=0;i<n;i++)if(i!==k){const f=m[i][k];m[i]=m[i].map((v,j)=>v-f*m[k][j]);}
  }return m.map(r=>r.slice(n));
}
const column=x=>x.map(v=>[v]);
const flat=a=>a.map(r=>r[0]);
const H=[[1,0,0,0],[0,1,0,0]];
const copy=a=>a.map(r=>[...r]);
export const FILTER_SETTINGS=Object.freeze({accelerationSigma:1.2,gateChi2:9.21034,maxAccuracy:50,maxGap:15,maxSpeed:4,staleSeconds:10});
export class GnssFilter {
  constructor(settings={}){this.settings={...FILTER_SETTINGS,...settings};this.reset();}
  reset(){this.x=null;this.P=null;this.t=null;this.history=[];this.rejected=0;this.anchor=null;}
  update(fix){
    const {x,y,t,accuracy}=fix,s=this.settings;
    const bad=![x,y,t,accuracy].every(Number.isFinite)||accuracy<=0;
    if(bad)return {accepted:false,reason:'invalid'};
    if(this.t!==null&&t<=this.t)return {accepted:false,reason:'time_order'};
    if(accuracy>s.maxAccuracy)return {accepted:false,reason:'accuracy',raw:[x,y],accuracy,t};
    // accuracy is a 95% horizontal radius. Isotropic-Gaussian assumption: r95 = 2.4477 sigma.
    const variance=Math.max(1.5,accuracy/(fix.accuracyConvention==='unspecified_radius'?1:2.44774683))**2,R=[[variance,0],[0,variance]];
    const restart=!this.x||t-this.t>s.maxGap||t-this.anchor.t>s.maxGap;
    let F=eye(4),xp,Pp,nis=0,accepted=true,reason=restart?'initial':'updated';
    if(restart){this.x=[x,y,0,0];this.P=[[variance,0,0,0],[0,variance,0,0],[0,0,4,0],[0,0,0,4]];}
    else {
      const dt=t-this.t;F=[[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]];
      const q=s.accelerationSigma**2,a=dt**4/4*q,b=dt**3/2*q,c=dt**2*q;
      const Q=[[a,0,b,0],[0,a,0,b],[b,0,c,0],[0,b,0,c]];
      xp=flat(mul(F,column(this.x)));Pp=add(mul(mul(F,this.P),transpose(F)),Q);
      const innovation=[[x-xp[0]],[y-xp[1]]],S=add(mul(mul(H,Pp),transpose(H)),R),Si=inverse(S);
      nis=mul(mul(transpose(innovation),Si),innovation)[0][0];
      const elapsed=t-this.anchor.t;
      const jump=Math.hypot(x-this.anchor.x,y-this.anchor.y)>s.maxSpeed*elapsed+accuracy+this.anchor.accuracy;
      accepted=nis<=s.gateChi2&&!jump;reason=accepted?'updated':jump?'speed_gate':'innovation_gate';
      if(accepted){
        const K=mul(mul(Pp,transpose(H)),Si);this.x=flat(add(column(xp),mul(K,innovation)));
        // Joseph form preserves covariance symmetry and positive semidefiniteness.
        const A=add(eye(4),mul(K,H),-1);
        this.P=add(mul(mul(A,Pp),transpose(A)),mul(mul(K,R),transpose(K)));
      }else{this.x=xp;this.P=Pp;this.rejected++;}
    }
    this.t=t;if(accepted)this.anchor={x,y,t,accuracy};
    const row={t,raw:[x,y],accuracy,accepted,reason,nis,segment:restart,x:[...this.x],P:copy(this.P),F,xp:xp||[...this.x],Pp:Pp||copy(this.P)};
    this.history.push(row);
    return {...row,position:accepted?this.x.slice(0,2):null,rejected:this.rejected};
  }
}
// Fixed-interval Rauch–Tung–Striebel smoother, separately within each gap-delimited segment.
export function rtsSmooth(history){
  const result=history.map(h=>({t:h.t,position:h.x.slice(0,2),x:[...h.x],P:copy(h.P)}));
  for(let i=history.length-2;i>=0;i--){
    const h=history[i],next=history[i+1];if(next.segment)continue;
    const C=mul(mul(h.P,transpose(next.F)),inverse(next.Pp));
    result[i].x=flat(add(column(h.x),mul(C,add(column(result[i+1].x),column(next.xp),-1))));
    result[i].P=add(h.P,mul(mul(C,add(result[i+1].P,next.Pp,-1)),transpose(C)));
    result[i].position=result[i].x.slice(0,2);
  }return result;
}
export function circularHeading(previous,next,weight=.3){
  if(!Number.isFinite(next))return null;
  if(!Number.isFinite(previous))return (next%360+360)%360;
  const delta=((next-previous+540)%360)-180;return (previous+weight*delta+360)%360;
}
