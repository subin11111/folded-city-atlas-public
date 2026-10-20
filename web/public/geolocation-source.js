// Acquisition is separate from navigation quality gates. A coarse fix is still a fix.
export function inspectBrowserPosition(position,now=Date.now()){
  const c=position?.coords,t=position?.timestamp;
  if(!c||![c.latitude,c.longitude,c.accuracy,t].every(Number.isFinite)||c.accuracy<=0||Math.abs(c.latitude)>90||Math.abs(c.longitude)>180)return {valid:false,reason:'invalid'};
  if(now-t>15000||t-now>2000)return {valid:false,reason:'stale'};
  return {valid:true,latitude:c.latitude,longitude:c.longitude,accuracy:c.accuracy,timestamp:t};
}

export function watchBrowserPosition(geolocation,onPosition,onError){
  let active=true,watch=null;
  const receive=p=>{if(active)onPosition(p);},fail=e=>{if(active)onError(e);};
  if(!geolocation){fail({code:2,unsupported:true});return ()=>{active=false;};}
  // First obtain a possibly coarse fix while the high-accuracy watch warms up.
  try{watch=geolocation.watchPosition(receive,fail,{enableHighAccuracy:true,maximumAge:0,timeout:20000});}
  catch(e){fail({code:e.code===1?1:2});}
  try{geolocation.getCurrentPosition(receive,e=>{if(e.code===1)fail(e);},{enableHighAccuracy:false,maximumAge:0,timeout:8000});}
  catch{}
  return ()=>{active=false;if(watch!==null)geolocation.clearWatch(watch);};
}
