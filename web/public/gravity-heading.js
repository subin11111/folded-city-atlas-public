// Integrate angular rate about the gravity-defined vertical, in degrees/s.
// DeviceMotion rotationRate: beta=x, gamma=y, alpha=z. No double integration.
const finite=Number.isFinite,rad=Math.PI/180,wrap=a=>(a%360+360)%360;
export class GravityHeading {
  constructor(){this.reset();}
  reset(){this.bearing=null;this.at=null;this.active=false;this.gravity=null;this.bias=0;this.stationary=[];}
  anchor(bearing){if(finite(bearing)&&this.bearing===null)this.bearing=wrap(bearing);}
  sample(event,time){
    if(this.bearing===null||!finite(time))return null;
    const a=event.accelerationIncludingGravity,l=event.acceleration,w=event.rotationRate;
    if(!a||!w||![a.x,a.y,a.z,w.beta,w.gamma,w.alpha].every(finite)){this.active=false;return null;}
    const hasLinear=l&&[l.x,l.y,l.z].every(finite),inc=[a.x,a.y,a.z];
    if(hasLinear)this.gravity=inc.map((v,i)=>v-[l.x,l.y,l.z][i]);
    else if(!this.gravity)this.gravity=inc;
    else{const dt=this.at===null?.02:Math.max(0,Math.min(.25,(time-this.at)/1000)),gain=1-Math.exp(-dt/.7);
      this.gravity=this.gravity.map((v,i)=>v+gain*(inc[i]-v));}
    const norm=Math.hypot(...this.gravity);if(norm<5||norm>14){this.active=false;return null;}
    const rate=-[w.beta,w.gamma,w.alpha].reduce((s,v,i)=>s+v*this.gravity[i]/norm,0);
    if(Math.abs(rate)>240){this.at=time;this.active=false;return null;}
    const dt=this.at===null?0:(time-this.at)/1000;this.at=time;
    if(dt<0||dt>.25){this.bearing=null;this.active=false;return null;}
    if(hasLinear&&Math.hypot(l.x,l.y,l.z)<.15&&Math.hypot(w.beta,w.gamma,w.alpha)<2){
      this.stationary.push({time,rate});this.stationary=this.stationary.filter(r=>time-r.time<=1500);
      if(this.stationary.length>=20&&time-this.stationary[0].time>=800)this.bias=this.stationary.reduce((s,r)=>s+r.rate,0)/this.stationary.length;
    }else this.stationary=[];
    // Positive right-handed rotation about up decreases the clockwise bearing.
    this.bearing=wrap(this.bearing-(rate-this.bias)*dt);this.active=true;
    return {bearing:this.bearing,accuracy:15,source:'gravity_projected_gyro',time,biasDegPerSec:this.bias};
  }
}
