// Give the first vertical swipe to the browser, not the map's drag recognizer.
// Once the document has scrolled, restore normal one-finger map interaction.
export function browserScrollMode({mobile,standalone,scrollY,previous='page'}){
  if(!mobile||standalone)return 'map';
  if(scrollY<=4)return 'page';
  if(scrollY>=64)return 'map';
  return previous;
}

export function installBrowserScroll(host,root){
  const mobile=host.matchMedia('(max-width:650px), (max-height:500px) and (pointer:coarse)');
  const standalone=host.matchMedia('(display-mode:standalone)');
  let fingers=0,frame=0,origin=host.scrollY||0;
  const reserve=()=>root.style.setProperty('--browser-scroll-runway',`${Math.ceil(origin)+240}px`);
  const update=()=>{
    frame=0;if(fingers)return;
    if((host.scrollY||0)<=4&&origin>0){origin=0;reserve();}
    const next=browserScrollMode({mobile:mobile.matches,standalone:standalone.matches,
      scrollY:(host.scrollY||0)-origin,previous:root.dataset.browserScroll});
    if(next===root.dataset.browserScroll)return;
    root.dataset.browserScroll=next;root.dispatchEvent(new Event('browser-scroll-change'));
  };
  const schedule=()=>{if(!frame)frame=host.requestAnimationFrame(update);};
  // Reload/back may restore a document offset while Safari has opened its bars.
  // Reserve more native scroll space without programmatically scrolling the page.
  const pageShow=()=>{origin=host.scrollY||0;reserve();schedule();};
  const touch=event=>{fingers=event.touches.length;if(!fingers)schedule();};
  const touchEvents=['touchstart','touchend','touchcancel'];
  for(const name of touchEvents)host.addEventListener(name,touch,{passive:true,capture:true});
  for(const name of ['scroll','resize'])host.addEventListener(name,schedule,{passive:true});
  host.addEventListener('pageshow',pageShow,{passive:true});
  mobile.addEventListener('change',schedule);standalone.addEventListener('change',schedule);
  reserve();update();
  return ()=>{
    for(const name of touchEvents)host.removeEventListener(name,touch,true);
    for(const name of ['scroll','resize'])host.removeEventListener(name,schedule);
    host.removeEventListener('pageshow',pageShow);
    mobile.removeEventListener('change',schedule);standalone.removeEventListener('change',schedule);
    if(frame)host.cancelAnimationFrame(frame);
    delete root.dataset.browserScroll;
    root.style.removeProperty('--browser-scroll-runway');
  };
}

if(typeof window!=='undefined')installBrowserScroll(window,document.documentElement);
