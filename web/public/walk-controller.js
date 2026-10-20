const $=id=>document.getElementById(id);
const ns='http://www.w3.org/2000/svg';
import {createPhotoViewer} from './photo-viewer.js?v=journey-2';

export function createWalkController({navigation,ensureScene,getScene,getGoal,getPolicy=()=> 'shortest',onError,onStart=()=>{}}) {
  let current=null,goal='prime_hall',busy=false,lookPending=false,history=[],request=0,lookTimer=null;
  const workspace=document.querySelector('.workspace');
  let photoMode=true,lastTransitionSample=-1,policy='shortest';
  const sheet=document.querySelector('.walk-bottom-sheet');
  const syncSheetHeight=()=>workspace.style.setProperty('--walk-sheet-height',`${Math.ceil(sheet.getBoundingClientRect().height)}px`);
  if(typeof ResizeObserver!=='undefined')new ResizeObserver(syncSheetHeight).observe(sheet);
  function toggleSettings(open){$('walkSettings').hidden=!open;$('walkSettingsToggle').setAttribute('aria-expanded',String(open));if(open)toggleMap(false);}
  function toggleMap(open){$('walkMapPanel').hidden=!open;$('walkMapToggle').setAttribute('aria-expanded',String(open));if(open)toggleSettings(false);}
  const viewer=createPhotoViewer({onTurn(delta){if(current&&!busy)getScene()?.turnWalk(delta);},on3D:()=>presentation(false),onNearest:nearestDirection});
  function presentation(value){
    photoMode=value;workspace.classList.toggle('photo-mode',value&&!!current);viewer.show(value&&!!current);
    $('walkViewToggle').textContent=value?'3D 공간 보기':'실사 공간 보기';
    $('walkViewDescription').textContent=value?'사진으로 둘러보기':'3D로 둘러보기';
    $('walkPhotoClose').textContent=value?'실사 탐색으로 돌아가기':'3D 시점으로 돌아가기';
    $('walkHelp').textContent=value?'드래그로 둘러보고, 화살표로 이동하세요.':'화면을 드래그해 3D 공간을 둘러보세요.';
    if(value&&current)viewer.display(current.photo);else if(current)getScene()?.loadWalkModels();
  }
  async function nearestDirection(){
    if(!current||busy||!current.nearby_view)return;
    getScene().setWalkYaw(current.nearby_view.heading);onLook();
  }
  async function api(path,params) {
    const response=await fetch(`/api/walk/${path}?${new URLSearchParams({...params,policy})}`);
    const data=await response.json();if(!response.ok)throw new Error(data.error||'보행 지도를 읽지 못했습니다.');return data;
  }
  function setBusy(value){busy=value;for(const b of $('walkHUD').querySelectorAll('button'))b.disabled=value;render();}
  function svg(tag,attrs,parent){const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);parent.appendChild(e);return e;}
  function miniMap(){
    if(!current)return;const root=$('walkMiniMap');root.replaceChildren();
    const level=navigation.levels[current.level];if(!level)return;
    const [x,y]=current.position;root.setAttribute('viewBox',`${x-13} ${-y-11} 26 22`);
    const rings=items=>items.map(r=>r.map((p,i)=>`${i?'L':'M'}${p[0]},${-p[1]}`).join(' ')+'Z').join(' ');
    svg('path',{d:rings(level.floor_rings),fill:'#e8ecef','fill-rule':'evenodd'},root);
    svg('path',{d:rings(level.walkable_rings),fill:'#dce9dc','fill-rule':'evenodd'},root);
    svg('path',{d:rings(level.obstacle_rings),fill:'#8d969e','fill-rule':'evenodd'},root);
    for(const p of navigation.portals.filter(p=>p.kind==='stairs'))svg('path',{d:p.polyline.map((v,i)=>`${i?'L':'M'}${v[0]},${-v[1]}`).join(' '),fill:'none',stroke:'#c39147','stroke-width':.45},root);
    const next=current.options.find(o=>o.recommended);
    if(next)svg('path',{d:next.path.map((p,i)=>`${i?'L':'M'}${p.position[0]},${-p.position[1]}`).join(' '),fill:'none',stroke:'#006b3f','stroke-width':.3},root);
    svg('circle',{cx:x,cy:-y,r:.65,fill:'white',stroke:'#006b3f','stroke-width':.2},root);
    svg('path',{d:'M 1.3 0 L -.65 -.6 L -.35 0 L -.65 .6 Z',fill:'#006b3f',transform:`translate(${x} ${-y}) rotate(${-current.yaw*180/Math.PI})`},root);
  }
  function render(){
    if(!current)return;
    $('walkLocation').textContent=current.location;$('walkFloor').textContent=`${current.level}F`;
    $('walkGoal').textContent=current.goal;$('walkMiniFloor').textContent=`${current.level}F`;
    $('walkRemaining').textContent=current.arrived?'도착':current.remaining_m===null?'경로 없음':`${current.remaining_m.toFixed(1)} m 남음`;
    $('walkInstruction').textContent=busy?'선택한 보행 경로를 따라 이동 중…':current.arrived?'목적지 접근 지점에 도착했습니다.':current.policy==='elevator'?'계단 제외 경로 · 엘리베이터 운행·접근성 확인 필요':'이동할 방향을 선택하세요';
    $('walkHistory').disabled=busy||lookPending||!history.length;$('walkPhotoButton').disabled=busy||lookPending||!current.photo;
    $('walkPhotoButton').textContent=current.photo?'사진 크게 보기':'이 방향의 사진 없음';
    $('photoNearestDirection').disabled=!current.nearby_view||busy;
    $('walkExit').disabled=busy;
    const options=$('walkOptions'),secondary=$('walkSecondaryOptions');options.replaceChildren();secondary.replaceChildren();
    const recommended=current.options.find(o=>o.recommended);
    for(const option of current.options){
      if(option.id!=='recommended'&&!option.relative&&recommended?.kind===option.kind)continue;
      if(current.arrived&&option.recommended)continue;
      const button=document.createElement('button');button.disabled=busy||lookPending;button.dataset.move=option.id;
      button.className=option.recommended?'recommended':'';
      const delta=Math.atan2(Math.sin(option.heading-current.yaw),Math.cos(option.heading-current.yaw));
      const recommendedArrow=Math.abs(delta)<Math.PI/4?'↑':Math.abs(delta)>3*Math.PI/4?'↓':delta>0?'←':'→';
      const icon={recommended:recommendedArrow,forward:'↑',left:'←',right:'→',back:'↓'}[option.id]||'↗';
      const label=document.createElement('span');label.textContent=`${icon} ${option.label}`;
      const distance=document.createElement('small');distance.textContent=`${option.distance_m.toFixed(1)} m`;
      button.append(label,distance);button.addEventListener('click',()=>move(option));(option.recommended?options:secondary).appendChild(button);
    }
    options.hidden=!options.childElementCount;$('walkOtherDirections').hidden=!secondary.childElementCount;
    syncSheetHeight();
    for(const id of ['lookLeft','lookRight'])$(id).disabled=busy;
    if(!$('walkPhotoPanel').hidden)updatePhoto();miniMap();if(photoMode)viewer.display(current.photo);
  }
  function updatePhoto(){
    const p=current?.photo;
    $('walkPhotoImage').hidden=!p;
    if(p)$('walkPhotoImage').src=p.url;
    else $('walkPhotoImage').removeAttribute('src');
    $('walkPhotoCaption').textContent=p?'현재 둘러보는 지점 주변에서 촬영한 사진입니다.':'이 방향의 사진이 없습니다.';
  }
  async function refresh(){
    if(!current||busy)return;const token=++request;
    try{const next=await api('options',{node:current.node,yaw:getScene().getWalkYaw(),goal});if(token!==request||!current)return;current=next;}
    catch(error){onError(error.message);}
    finally{if(token===request){lookPending=false;render();}}
  }
  async function start(place='outside_start'){
    onStart();
    if(busy)return;busy=true;const token=++request;
    try{
      await ensureScene();if(!getScene())throw new Error('3D 보기를 먼저 시작해야 합니다.');goal=getGoal()||'prime_hall';policy=getPolicy();lookPending=false;
      const next=await api('start',{place,goal});if(token!==request)return;
      current=next;history=[];workspace.classList.add('walking');document.body.classList.add('is-walking');$('walkHUD').hidden=false;$('walkPhotoPanel').hidden=true;
      toggleSettings(false);toggleMap(false);$('walkOtherDirections').open=false;
      viewer.clear();getScene().enterWalk(next);presentation(true);render();
    }catch(error){onError(error.message);}finally{setBusy(false);}
  }
  async function move(option,backward=false,returnFrames=null){
    if(busy||lookPending||!current)return;const before=current;request++;toggleSettings(false);toggleMap(false);$('walkOtherDirections').open=false;setBusy(true);$('walkPhotoPanel').hidden=true;
    try{
      // 서버에서 검증한 보행 지점을 사용하고, 선택된 경로의 굴곡을 따라 이동한다.
      let transition=null;
      if(!backward)transition=await api('move',{node:before.node,yaw:before.yaw,choice:option.id,goal});
      else if(returnFrames)transition={samples:returnFrames,next:null};
      const next=transition?.next||await api('options',{node:option.destination_node,yaw:option.heading,goal});
      if(transition)viewer.preloadSequence(transition.samples);
      lastTransitionSample=-1;
      await getScene().moveWalk(option,{onProgress(fraction){
        if(!transition)return;
        let index=0;while(index<transition.samples.length-1&&transition.samples[index+1].fraction<=fraction)index++;
        if(index===lastTransitionSample)return;lastTransitionSample=index;
        const frame=transition.samples[index];if(photoMode)viewer.display(frame.photo);
        $('walkFloor').textContent=frame.level==='elevator'?'승강기':frame.level==='stairs'?'층간':`${frame.level}F`;
        if(frame.notice)$('walkInstruction').textContent=frame.notice;
        if(before.remaining_m!==null)$('walkRemaining').textContent=`${Math.max(0,before.remaining_m-option.distance_m*fraction).toFixed(1)} m 남음`;
      }});
      if(!backward)history.push({before,option,samples:transition?.samples,reverseSamples:transition?.reverse_samples});
      current=next;
    }catch(error){onError(error.message);}finally{setBusy(false);if(current&&Math.abs(current.yaw-getScene().getWalkYaw())>.02)onLook();}
  }
  async function goBack(){
    if(busy||lookPending||!history.length)return;const entry=history.pop();
    const reversed={...entry.option,path:[...entry.option.path].reverse(),destination_node:entry.before.node,heading:entry.before.yaw};
    const frames=entry.reverseSamples||entry.samples?.map(frame=>({...frame,fraction:1-frame.fraction,
      photo:frame.photo?{...frame.photo,heading_delta_deg:Math.round(Math.abs(Math.atan2(Math.sin(frame.photo.heading-entry.before.yaw),Math.cos(frame.photo.heading-entry.before.yaw)))*180/Math.PI)}:null})).reverse();
    await move(reversed,true,frames);
  }
  function exit(){
    if(busy)return;request++;current=null;history=[];clearTimeout(lookTimer);
    lookPending=false;viewer.show(false);workspace.classList.remove('walking','photo-mode');document.body.classList.remove('is-walking');$('walkHUD').hidden=true;$('walkPhotoPanel').hidden=true;toggleSettings(false);toggleMap(false);getScene()?.showCampus();
  }
  function onLook(){if(!current||busy)return;lookPending=true;current.yaw=getScene().getWalkYaw();render();clearTimeout(lookTimer);lookTimer=setTimeout(refresh,140);}
  $('walkExit').addEventListener('click',exit);
  $('walkViewToggle').addEventListener('click',()=>presentation(!photoMode));
  $('walkHistory').addEventListener('click',goBack);
  $('lookLeft').addEventListener('click',()=>getScene()?.turnWalk(Math.PI/4));
  $('lookRight').addEventListener('click',()=>getScene()?.turnWalk(-Math.PI/4));
  $('walkPhotoButton').addEventListener('click',()=>{$('walkPhotoPanel').hidden=false;updatePhoto();});
  $('walkPhotoClose').addEventListener('click',()=>$('walkPhotoPanel').hidden=true);
  $('walkSettingsToggle').addEventListener('click',()=>toggleSettings($('walkSettings').hidden));
  $('walkSettingsClose').addEventListener('click',()=>{toggleSettings(false);$('walkSettingsToggle').focus();});
  $('walkMapToggle').addEventListener('click',()=>toggleMap($('walkMapPanel').hidden));
  $('walkMapClose').addEventListener('click',()=>{toggleMap(false);$('walkMapToggle').focus();});
  document.addEventListener('keydown',e=>{
    if(!current||busy||e.target.closest('input,select,textarea,dialog')||e.ctrlKey||e.metaKey)return;
    if(e.key==='Escape'){if(!$('walkPhotoPanel').hidden)$('walkPhotoPanel').hidden=true;else if(!$('walkSettings').hidden)toggleSettings(false);else if(!$('walkMapPanel').hidden)toggleMap(false);else exit();return;}
    if(e.key==='ArrowLeft'||e.key==='ArrowRight'){e.preventDefault();getScene().turnWalk(e.key==='ArrowLeft'?Math.PI/4:-Math.PI/4);}
    if(e.key==='ArrowUp'||e.key==='ArrowDown'){e.preventDefault();const option=current.options.find(o=>o.id===(e.key==='ArrowUp'?'forward':'back'));if(option)move(option);}
  });
  return {start,exit,onLook,isActive:()=>!!current};
}
