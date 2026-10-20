// Sensor-frame conversion and causal walking cycles. No map or destination input.
const finite=Number.isFinite,rad=Math.PI/180;
const wrap=a=>(a%360+360)%360;
const delta=(a,b)=>((a-b+540)%360)-180;

export const STEP_SETTINGS=Object.freeze({sampleHz:50,highPassHz:.65,lowPassHz:3,
  minimumPeak:.65,minimumTrough:-.25,minimumAmplitude:1.2,minimumIntervalMs:320,
  maximumCycleMs:1600,warmupMs:1000,maximumGapMs:250,strideCoefficient:.55});

class Biquad {
  constructor(kind,frequency,rate,initial=0){
    const w=2*Math.PI*frequency/rate,c=Math.cos(w),a=Math.sin(w)/Math.SQRT2,den=1+a;
    const sign=kind==='high'?1:-1,b=(1+sign*c)/2;
    this.b0=b/den;this.b1=-sign*2*b/den;this.b2=b/den;
    this.a1=-2*c/den;this.a2=(1-a)/den;
    this.x1=this.x2=initial;this.y1=this.y2=kind==='high'?0:initial;
  }
  sample(x){const y=this.b0*x+this.b1*this.x1+this.b2*this.x2-this.a1*this.y1-this.a2*this.y2;
    this.x2=this.x1;this.x1=x;this.y2=this.y1;this.y1=y;return y;}
}

export class StepDetector {
  constructor(options={}){this.settings={...STEP_SETTINGS,...options};this.reset();}
  reset(){this.at=null;this.previous=null;this.next=null;this.started=null;this.filtered=0;
    this.peak=0;this.trough=0;this.phase='waiting';this.cycleAt=0;this.lastStep=-Infinity;}
  sample(magnitude,time){
    const s=this.settings;
    if(!finite(magnitude)||!finite(time)||magnitude<0||magnitude>35)return null;
    if(this.at!==null&&time<=this.at)return null;
    if(this.at===null||time-this.at>s.maximumGapMs){
      this.reset();this.at=this.started=time;this.previous=magnitude;this.next=time+1000/s.sampleHz;
      this.high=new Biquad('high',s.highPassHz,s.sampleHz,magnitude);
      this.low=new Biquad('low',s.lowPassHz,s.sampleHz);return null;
    }
    let step=null;
    // Resample only inside the interval of two received samples; never extrapolate
    // a missing interval. The filter and cycle timing are independent of event Hz.
    for(;this.next<=time+1e-7;this.next+=1000/s.sampleHz){
      const u=Math.min(1,(this.next-this.at)/(time-this.at));
      const v=this.low.sample(this.high.sample(this.previous+(magnitude-this.previous)*u));
      this.filtered=v;if(this.next-this.started<s.warmupMs)continue;
      if(this.phase!=='waiting'&&this.next-this.cycleAt>s.maximumCycleMs){this.phase='waiting';this.peak=this.trough=0;}
      if(v<=s.minimumTrough){
        if(this.phase==='peak'){
          const amplitude=this.peak-Math.min(this.trough,v);
          if(this.next-this.lastStep>=s.minimumIntervalMs&&amplitude>=s.minimumAmplitude){
            this.lastStep=this.next;
            step={time:this.next,amplitude,length:Math.max(.48,Math.min(.85,s.strideCoefficient*Math.pow(amplitude,.25)))};
          }
          this.phase='trough';this.cycleAt=this.next;this.peak=0;this.trough=v;
        }else{
          if(this.phase==='waiting'){this.phase='trough';this.cycleAt=this.next;}
          this.trough=Math.min(this.trough,v);
        }
      }else if(this.phase==='trough'&&v>=s.minimumPeak){this.phase='peak';this.peak=v;}
      else if(this.phase==='peak')this.peak=Math.max(this.peak,v);
    }
    this.previous=magnitude;this.at=time;return step;
  }
}

// A real, independently measured straight distance is required. A planned route
// or nearest destination is not a calibration measurement.
export function strideCalibration(distance,amplitudes){
  if(!finite(distance)||distance<5||distance>30||!Array.isArray(amplitudes)||amplitudes.length<8||
      !amplitudes.every(v=>finite(v)&&v>=STEP_SETTINGS.minimumAmplitude))return null;
  const coefficient=distance/amplitudes.reduce((sum,v)=>sum+Math.pow(v,.25),0);
  const average=distance/amplitudes.length;
  if(coefficient<.3||coefficient>.8||average<.45||average>.9)return null;
  const boundedDistance=amplitudes.reduce((sum,v)=>sum+Math.max(.48,Math.min(.85,coefficient*Math.pow(v,.25))),0);
  if(Math.abs(boundedDistance-distance)>Math.max(.3,distance*.03))return null;
  return {version:1,coefficient,distance,steps:amplitudes.length};
}

// Z-X'-Y'' device attitude as defined by W3C. Returns a horizontal body-forward
// bearing in the attitude's reference frame, which may be arbitrary on Safari.
export function attitudeForward(alpha,beta,gamma,screenAngle=0){
  if(![alpha,beta,gamma,screenAngle].every(finite))return null;
  const [a,b,g,o]=[alpha,beta,gamma,screenAngle].map(v=>v*rad);
  const ca=Math.cos(a),sa=Math.sin(a),cb=Math.cos(b),sb=Math.sin(b),cg=Math.cos(g),sg=Math.sin(g);
  const x=[ca*cg-sa*sb*sg,sa*cg+ca*sb*sg],y=[-cb*sa,cb*ca];
  const top=[Math.sin(o)*x[0]+Math.cos(o)*y[0],Math.sin(o)*x[1]+Math.cos(o)*y[1]];
  const rear=[-ca*sg-sa*sb*cg,-sa*sg+ca*sb*cg];
  const tn=Math.hypot(...top),rn=Math.hypot(...rear),w=Math.max(0,Math.min(1,(.85-Math.abs(cb*cg))/.2));
  if((w<1&&tn<.2)||(w>0&&rn<.2))return null;
  const v=[(1-w)*top[0]/Math.max(tn,1e-9)+w*rear[0]/Math.max(rn,1e-9),
    (1-w)*top[1]/Math.max(tn,1e-9)+w*rear[1]/Math.max(rn,1e-9)];
  if(Math.hypot(...v)<.5)return null;
  return {bearing:wrap(Math.atan2(v[0],v[1])/rad),topBearing:wrap(Math.atan2(y[0],y[1])/rad),
    compassCalibrationSafe:Math.abs(beta)<60&&Math.abs(gamma)<45,axis:w>.5?'rear':'screen_top'};
}

export class WalkingHeading {
  constructor(referenceHeading=null){this.reset(referenceHeading);}
  reset(referenceHeading=null){this.reference=finite(referenceHeading)?wrap(referenceHeading):null;
    this.offset=null;this.candidate=null;this.at=null;this.value=null;this.source=null;}
  sample(event,time,screenAngle=0){
    if(!finite(time)||(this.at!==null&&time<=this.at))return null;
    const body=attitudeForward(event.alpha,event.beta,event.gamma,screenAngle);
    if(!body)return null;
    let value,accuracy=15,source;
    if(this.reference!==null){
      if(!this.candidate||Math.abs(delta(body.bearing,this.candidate.bearing))>12){this.candidate={bearing:body.bearing,time};return null;}
      if(time-this.candidate.time<700)return null;
      this.offset=delta(this.reference,body.bearing);this.reference=null;this.source='confirmed_entrance';
    }
    if(this.offset!==null){value=wrap(body.bearing+this.offset);source=this.source;}
    else if(event.absolute===true){value=body.bearing;source='absolute_attitude';}
    else if(finite(event.webkitCompassHeading)&&event.webkitCompassHeading>=0&&
      finite(event.webkitCompassAccuracy)&&event.webkitCompassAccuracy>=0&&event.webkitCompassAccuracy<=30&&body.compassCalibrationSafe){
      this.offset=delta(event.webkitCompassHeading,body.topBearing);
      this.source='flat_compass_reference';value=wrap(body.bearing+this.offset);source=this.source;accuracy=Math.max(15,event.webkitCompassAccuracy);
    }else return null;
    const weight=this.at===null?1:1-Math.exp(-Math.min(1,(time-this.at)/1000)/.18);
    this.value=this.value===null?value:wrap(this.value+weight*delta(value,this.value));this.at=time;
    return {bearing:this.value,accuracy,axis:body.axis,source};
  }
}

export function sensorEventTime(event,now=Date.now(),origin=globalThis.performance?.timeOrigin){
  if(!finite(event?.timeStamp)||event.timeStamp<=0)return now;
  const time=event.timeStamp>1e12?event.timeStamp:finite(origin)?origin+event.timeStamp:now;
  return time>now+250||now-time>1000?null:time;
}
