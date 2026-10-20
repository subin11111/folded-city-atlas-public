export function sheetDetents(availableHeight,bottomInset=0){
  const height=Math.max(120,availableHeight),inset=Math.max(0,bottomInset);
  const collapsed=Math.min(height-48,56+inset);
  const expanded=Math.max(collapsed+24,height-70);
  return [collapsed,Math.min(expanded-12,Math.max(collapsed+12,Math.round(height*.44))),expanded];
}
export function nearestDetent(height,detents,velocity=0){
  const projected=height+Math.max(-2,Math.min(2,velocity))*140;
  return detents.reduce((best,value,index)=>Math.abs(value-projected)<Math.abs(detents[best]-projected)?index:best,0);
}

export function createBottomSheet(){
  const workspace=document.querySelector('.workspace'),sidebar=document.querySelector('.sidebar');
  const grip=document.getElementById('sheetGrip'),scroll=document.getElementById('sheetScroll');
  const mobile=window.matchMedia('(max-width:650px), (max-height:500px) and (pointer:coarse)');
  let index=1,drag=null,ignoreClick=false;
  const names=['접힘 · 지도 전체 보기','중간 높이','펼침 · 목록 스크롤'];
  const heights=()=>sheetDetents(workspace.getBoundingClientRect().height,parseFloat(getComputedStyle(sidebar).paddingBottom)||0);
  function paint(height){workspace.style.setProperty('--place-sheet-height',`${Math.round(height)}px`);}
  function set(next){
    index=Math.max(0,Math.min(2,next));drag=null;sidebar.classList.remove('sheet-dragging');
    workspace.dataset.sheet=['collapsed','medium','expanded'][index];
    grip.setAttribute('aria-valuenow',String(index));grip.setAttribute('aria-valuetext',names[index]);
    scroll.inert=mobile.matches&&index===0;
    if(mobile.matches)paint(heights()[index]);else workspace.style.removeProperty('--place-sheet-height');
  }
  function begin(y,time){
    drag={y,start:sidebar.getBoundingClientRect().height,lastY:y,lastTime:time,velocity:0,moved:false};
  }
  function update(y,time){
    if(!drag)return;
    const delta=drag.y-y;if(Math.abs(delta)>6)drag.moved=true;
    drag.velocity=(drag.lastY-y)/Math.max(1,time-drag.lastTime);drag.lastY=y;drag.lastTime=time;
    sidebar.classList.add('sheet-dragging');
    const points=heights();paint(Math.max(points[0],Math.min(points[2],drag.start+delta)));
  }
  function finish(){
    if(!drag)return;
    const moved=drag.moved,velocity=drag.velocity;
    const next=nearestDetent(sidebar.getBoundingClientRect().height,heights(),velocity);
    ignoreClick=moved;set(next);
  }
  grip.addEventListener('pointerdown',event=>{
    if(!mobile.matches)return;event.preventDefault();begin(event.clientY,event.timeStamp);grip.setPointerCapture(event.pointerId);
  });
  grip.addEventListener('pointermove',event=>{if(drag){event.preventDefault();update(event.clientY,event.timeStamp);}});
  grip.addEventListener('pointerup',finish);grip.addEventListener('pointercancel',()=>set(index));
  grip.addEventListener('click',()=>{if(ignoreClick){ignoreClick=false;return;}set(index===2?0:index+1);});
  grip.addEventListener('keydown',event=>{
    const next={ArrowUp:index+1,ArrowRight:index+1,ArrowDown:index-1,ArrowLeft:index-1,Home:0,End:2,' ':index===2?0:index+1,Enter:index===2?0:index+1}[event.key];
    if(next===undefined)return;event.preventDefault();set(next);
  });
  let touch=null;
  scroll.addEventListener('touchstart',event=>{
    if(!mobile.matches||event.touches.length!==1||event.target.closest('input,select,button,summary,a'))return;
    const point=event.touches[0];touch={y:point.clientY,x:point.clientX,time:event.timeStamp,top:scroll.scrollTop,claimed:false};
  },{passive:true});
  scroll.addEventListener('touchmove',event=>{
    if(!touch||event.touches.length!==1)return;
    const point=event.touches[0],dy=point.clientY-touch.y;
    if(!touch.claimed&&Math.abs(dy)>8&&Math.abs(dy)>Math.abs(point.clientX-touch.x)){
      if((dy<0&&index<2)||(dy>0&&touch.top<=0&&scroll.scrollTop<=0)){
        touch.claimed=true;begin(touch.y,touch.time);
      }else{touch=null;return;}
    }
    if(touch.claimed){event.preventDefault();update(point.clientY,event.timeStamp);}
  },{passive:false});
  scroll.addEventListener('touchend',()=>{if(touch?.claimed)finish();touch=null;});
  scroll.addEventListener('touchcancel',()=>{touch=null;set(index);});
  window.addEventListener('resize',()=>set(index));mobile.addEventListener('change',()=>set(index));
  // visualViewport updates can change the workspace without window.resize.
  new ResizeObserver(()=>{if(!drag)set(index);}).observe(workspace);
  set(1);
  return {expand(){set(2);},reveal(){if(index===0)set(1);},set};
}
