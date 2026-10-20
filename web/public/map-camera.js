const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const angleDifference=(a,b)=>((a-b+540)%360)-180;
export function orbitCamera(start,dx,dy){
  return {bearing:angleDifference(start.bearing-dx*.35,0),pitch:clamp(start.pitch+dy*.22,0,70)};
}
export function followChanged(previous,data){
  if(!previous)return true;
  const moved=Math.hypot(data.position[0]-previous.position[0],data.position[1]-previous.position[1])>=2;
  const turned=data.headingUp&&Number.isFinite(data.headingDegrees)&&
    (!Number.isFinite(previous.headingDegrees)||Math.abs(angleDifference(data.headingDegrees,previous.headingDegrees))>=3);
  return moved||turned||data.headingUp!==previous.headingUp;
}
