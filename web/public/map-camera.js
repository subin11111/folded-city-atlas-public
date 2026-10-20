const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export function campusFraming(width,height){
  const compact=width<=650||height<480;
  return {minZoom:compact?14.5:13.6,pitch:compact?42:50,bearing:-32,
    padding:{top:compact?Math.min(60,height*.16):100,bottom:compact?22:45,left:compact?14:25,right:compact?14:25}};
}
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
