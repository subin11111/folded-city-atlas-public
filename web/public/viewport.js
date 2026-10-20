// Match the app frame to Safari's visible area, including toolbar/keyboard changes.
// Keep pinch zoom native: do not resize the layout around a magnified viewport.
export function viewportMetrics(host){
  const v=host.visualViewport;
  if(v&&Number.isFinite(v.scale)&&Math.abs(v.scale-1)>.02)return null;
  const positive=(a,b)=>Number.isFinite(a)&&a>0?a:b;
  return {
    height:positive(v?.height,host.innerHeight),
    width:positive(v?.width,host.innerWidth),
    top:Math.max(0,Number.isFinite(v?.offsetTop)?v.offsetTop:0),
    left:Math.max(0,Number.isFinite(v?.offsetLeft)?v.offsetLeft:0)
  };
}

export function installViewport(host,root){
  let frame=0;
  const update=()=>{
    frame=0;const metrics=viewportMetrics(host);if(!metrics)return;
    for(const [key,value] of Object.entries(metrics))root.style.setProperty(`--app-${key}`,`${value}px`);
  };
  const schedule=()=>{if(!frame)frame=host.requestAnimationFrame(update);};
  const events=[[host,'resize'],[host,'orientationchange'],[host,'pageshow'],
    [host.visualViewport,'resize'],[host.visualViewport,'scroll']].filter(([target])=>target);
  for(const [target,event] of events)target.addEventListener(event,schedule,{passive:true});
  update();
  return ()=>{for(const [target,event] of events)target.removeEventListener(event,schedule);if(frame)host.cancelAnimationFrame(frame);};
}

if(typeof window!=='undefined')installViewport(window,document.documentElement);
