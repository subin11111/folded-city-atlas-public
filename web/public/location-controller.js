import {GnssFilter} from './gnss-filter.js';
import {createGeoFrame} from './geo-frame.js';
import {createOutdoorRouter,distance} from './outdoor-routing.js';
import {navigationEstimate} from './live-navigation.js';
import {indoorProgress,requestWalkingSensors} from './indoor-position.js';
import {MapParticleTracker} from './map-particle-filter.js';
import {GravityHeading} from './gravity-heading.js';
import {dockMobileControl} from './mobile-layout.js';
import {WalkingHeading,StepDetector,strideCalibration,sensorEventTime} from './walking-sensors.js';
const $=id=>document.getElementById(id),finite=Number.isFinite;

export function createLocationController({map,campus,entry,navigation,places,getScene,setView,onIndoor,onLevel,notify,collapseSheet}){
  const frame=createGeoFrame(map),router=createOutdoorRouter(campus,map),filter=new GnssFilter();
  const state={mode:'off',fix:null,heading:null,headingAt:0,follow:true,headingUp:false,goal:'',goalName:'',indoorDistance:0,route:null,indoorRoute:null,watch:null};
  const walkingHeading=new WalkingHeading(),gravityHeading=new GravityHeading();let strideSession=null,strideSetting=null;
  try{const saved=JSON.parse(localStorage.getItem('folded-city-stride-v1')||'null');if(saved?.version===1&&finite(saved.coefficient)&&saved.coefficient>=.3&&saved.coefficient<=.8)strideSetting=saved;}catch{}
  let lastGoodAt=0,tracker=null,generation=0,sensorsAttached=false,headingAttached=false,lastDraw=0,error='',wakeLock=null,pendingTransition=null;
  const controls=document.createElement('div');controls.className='location-controls';
  controls.innerHTML='<button id="locateButton" aria-label="현재 위치 사용"><svg aria-hidden="true" viewBox="0 0 24 24"><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3m0 14v3M2 12h3m14 0h3"/></svg><span>내 위치</span></button><button id="headingButton" aria-label="휴대폰 방향 사용">➤ <span>휴대폰 방향</span></button>';
  document.querySelector('.map-shell').append(controls);
  dockMobileControl($('headingButton'));
  const panel=document.createElement('section');panel.className='location-card';panel.id='locationCard';panel.hidden=true;panel.setAttribute('aria-label','현재 위치 안내');
  panel.innerHTML='<div class="location-summary"><span id="locationMode">현재 위치</span><button id="stopLocation" aria-label="현재 위치 안내 종료">×</button></div><div class="live-navigation-summary" id="liveSummary" hidden><span id="liveDestination"></span><strong id="liveRemaining"></strong></div><strong id="locationInstruction">위치를 찾고 있어요</strong><p id="locationQuality" role="status"></p><p id="headingStatus"></p><div class="location-actions"><button id="followLocation" aria-pressed="true">위치 따라가기</button><button id="confirmEntrance" hidden>입구에서 실내 안내</button><button id="confirmFloor" hidden></button><button id="correctIndoor" hidden>현재 위치 수정</button><button id="retryWalkingSensors" hidden>동작 센서 허용</button><button id="rerouteIndoor" hidden>여기서 경로 다시 찾기</button></div><details><summary>이용 안내</summary><p id="locationHelp">위치 권한을 허용해 주세요. 실내에서는 입구를 확인한 뒤 걸음과 방향 센서로 위치를 추정합니다.</p></details>';
  document.querySelector('.map-shell').append(panel);
  const strideTools=document.createElement('details');strideTools.className='stride-settings';
  strideTools.innerHTML='<summary>보폭 맞추기</summary><p>줄자 등으로 직접 잰 직선 구간을 걸어 주세요. 측정 중에는 위치 안내를 멈추고, 끝난 뒤 시작 위치를 다시 선택합니다.</p><label>직접 잰 거리 (m) <input id="strideDistance" type="number" min="5" max="30" step="0.5" value="10" inputmode="decimal"></label><button id="strideMeasure" type="button">걸음 측정 시작</button><p id="strideStatus" role="status"></p>';
  panel.append(strideTools);
  const dialog=document.createElement('dialog');dialog.className='location-anchor-dialog';dialog.setAttribute('aria-labelledby','anchorTitle');
  dialog.innerHTML='<form method="dialog"><button class="dialog-close" aria-label="닫기">×</button></form><h2 id="anchorTitle">지금 어디에 있나요?</h2><p>눈앞에 보이는 장소를 선택해 주세요.</p><select id="anchorPlace" aria-label="현재 있는 장소"></select><img id="anchorPhoto" alt="선택한 장소의 모습"><p id="anchorName"></p><button id="confirmAnchor" class="primary">이 위치에서 안내 시작</button><p class="anchor-help">휴대폰 위쪽을 걷는 방향으로 들고 화면을 켜 두세요. 층을 이동하면 도착한 층을 확인해 주세요.</p>';
  document.body.append(dialog);
  for(const p of places.filter(p=>p.level==='1'||p.level==='2')){const option=document.createElement('option');option.value=p.id;option.textContent=`${p.name} · ${p.level}층`;$('anchorPlace').append(option);}
  const layer=document.createElementNS('http://www.w3.org/2000/svg','g');layer.id='locationLayer';$('mapSvg').append(layer);
  function heading(){return Date.now()-state.headingAt<=4000?state.heading:null;}
  function draw(data){
    getScene()?.updateLocation?.(data);layer.replaceChildren();if(!data)return;
    const shape=(tag,attrs)=>{const e=document.createElementNS(layer.namespaceURI,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);layer.append(e);};
    const [x,y]=data.position;
    shape('circle',{cx:x,cy:-y,r:data.accuracy,fill:'#2789e918',stroke:'#2789e9','stroke-width':.12});
    if(data.heading){const [hx,hy]=data.heading;shape('path',{d:`M${x},${-y} L${x+hx*3-hy},${-(y+hy*3+hx)} L${x+hx*3+hy},${-(y+hy*3-hx)} Z`,fill:'#2789e955'});}
    shape('circle',{cx:x,cy:-y,r:.48,fill:data.stale?'#8a9690':'#2789e9',stroke:'white','stroke-width':.15});
    if(data.follow&&$('map3d').hidden)$('mapSvg').setAttribute('viewBox',`${x-12} ${-y-12} 24 24`);
  }
  function render(){
    if(state.mode==='off'){draw(null);return;}
    panel.hidden=false;panel.dataset.mode=state.mode;$('locateButton').setAttribute('aria-pressed','true');
    for(const id of ['confirmFloor','correctIndoor','retryWalkingSensors','rerouteIndoor'])$(id).disabled=!!strideSession;
    $('strideDistance').disabled=!!strideSession;
    $('liveSummary').hidden=!state.goal;$('liveDestination').textContent=state.goalName||'선택한 목적지';
    for(const id of ['confirmFloor','correctIndoor','retryWalkingSensors','rerouteIndoor'])$(id).hidden=true;
    $('confirmEntrance').hidden=state.mode!=='live'||!state.goal;
    if(state.mode==='indoor'){
      $('confirmEntrance').hidden=true;$('locationMode').textContent='실내 위치 추정';$('correctIndoor').hidden=false;
      if(!tracker){$('locationInstruction').textContent='실내 안내를 준비하고 있어요';$('locationQuality').textContent=error;return;}
      const fix=tracker.snapshot(),now=Date.now(),progress=indoorProgress(fix,state.indoorRoute),deg=heading();
      const sensorStale=now-fix.lastMotionAt>4000,headingStale=now-fix.headingAt>4000||!fix.headingReady;
      const paused=document.hidden||sensorStale||headingStale||fix.uncertainty>7||fix.blocked>=3;
      pendingTransition=progress.transition||null;
      $('locationInstruction').textContent=progress.arrived?'목적지 근처예요. 출입문을 확인해 주세요.':pendingTransition?`${pendingTransition.kind==='elevator'?'엘리베이터':'계단'}로 이동한 뒤 도착한 층을 확인해 주세요.`:fix.blocked>=3?'통로와 추정 위치가 맞지 않아요. 현재 위치를 수정해 주세요.':progress.crossTrack>2.5?'경로에서 벗어났어요. 여기서 경로를 다시 찾을 수 있어요.':nextInstruction(fix,progress);
      if(sensorStale)$('locationInstruction').textContent='동작 센서를 기다리고 있어요';
      else if(headingStale)$('locationInstruction').textContent=walkingHeading.reference!==null?'입구 안쪽을 향한 채 잠시 멈춰 주세요':'휴대폰을 잠깐 평평하게 들어 방향을 맞춰 주세요';
      else if(fix.uncertainty>7)$('locationInstruction').textContent='주변 장소를 선택해 현재 위치를 맞춰 주세요';
      if(document.hidden)$('locationInstruction').textContent='화면으로 돌아오면 안내를 이어갑니다';
      if(strideSession)$('locationInstruction').textContent='보폭을 측정하고 있어요. 직접 잰 구간 끝에서 완료를 눌러 주세요.';
      $('locationQuality').textContent=error||`${paused?'위치 갱신 대기':'센서와 지도로 위치 추정'} · ${fix.level}층`;
      if(strideSession)$('locationQuality').textContent='보폭 측정 중 · 위치 안내 일시 정지';
      $('headingStatus').textContent=headingStale?'방향 센서 대기 중':`${Math.round(deg??0)}° · ${fix.steps}걸음 감지`;
      $('liveRemaining').textContent=progress.remaining===null?'경로를 확인해 주세요':`약 ${Math.round(progress.remaining)}m 남음`;
      $('locationHelp').textContent='입구에서는 안쪽을 향해 시작 방향을 맞춥니다. 다른 장소에서는 휴대폰을 잠깐 평평하게 들어 나침반 기준을 맞춘 뒤 세워 들어도 됩니다. 휴대폰은 걷는 방향을 향하게 들고 화면을 켜 두세요. 위치가 어긋나면 주변 장소를 다시 선택하고, 층 이동 후 도착한 층을 확인해 주세요.';
      $('retryWalkingSensors').hidden=sensorsAttached&&headingAttached&&!sensorStale&&!headingStale;$('rerouteIndoor').hidden=progress.crossTrack<=2.5;
      if(pendingTransition){const node=state.indoorRoute.route[pendingTransition.end_index];$('confirmFloor').hidden=false;$('confirmFloor').textContent=`${node.level}층에 도착했어요`;}
      strideTools.hidden=false;draw({indoor:true,position:fix.position,raw:fix.position,accuracy:fix.uncertainty,heading:headingStale?null:[Math.cos(fix.yaw),Math.sin(fix.yaw)],headingDegrees:deg,headingUp:false,stale:paused||!!strideSession,follow:state.follow});return;
    }
    strideTools.hidden=true;$('locationMode').textContent='현재 위치';const fix=state.fix,stale=!lastGoodAt||Date.now()-lastGoodAt>10000,deg=heading();
    $('locationHelp').textContent='실외 위치는 휴대폰의 위치 서비스를 사용합니다. 건물에 들어가면 입구에서 실내 안내를 선택해 주세요. 위치·동작 정보는 이 화면에서 처리하며 서버로 보내지 않습니다.';
    $('locationQuality').textContent=error||(!fix?'위치 권한을 허용해 주세요.':stale?'위치 신호를 기다리고 있어요.':`위치 수신 범위 약 ${Math.round(fix.accuracy)}m`);
    $('headingStatus').textContent=deg===null?'방향을 보려면 화살표 버튼을 눌러 주세요.':`${Math.round(deg)}° · 휴대폰 방향`;
    if(!fix){$('locationInstruction').textContent='위치를 찾고 있어요';return;}
    const estimate=navigationEstimate({position:fix.position,route:state.route,entry,indoorDistance:state.indoorDistance,accuracy:fix.accuracy,stale,rejected:state.rejected});
    $('liveRemaining').textContent=estimate.completeConnection?`약 ${Math.round(estimate.knownRemaining)}m · ${estimate.estimatedMinutes}분`:state.goal?`입구 이후 약 ${Math.round(state.indoorDistance)}m`:'목적지를 선택해 주세요';
    let instruction=state.goal?'경로를 따라 학생회관 입구로 이동해 주세요.':'목적지를 선택하면 길을 안내합니다.';
    if(estimate.nearEntry&&state.goal)instruction='입구 앞인가요? 입구에서 실내 안내를 선택해 주세요.';
    else if(estimate.progress&&estimate.qualityOk){const next=state.route.points[estimate.progress.index],vector=[next[0]-fix.position[0],next[1]-fix.position[1]],north=frame.headingVector(0),east=frame.headingVector(90);const bearing=(Math.atan2(vector[0]*east[0]+vector[1]*east[1],vector[0]*north[0]+vector[1]*north[1])*180/Math.PI+360)%360;const diff=deg===null?0:((bearing-deg+540)%360)-180;instruction=`${deg===null?'경로를 따라':Math.abs(diff)<35?'앞으로':Math.abs(diff)>145?'뒤쪽 경로로':diff>0?'오른쪽으로':'왼쪽으로'} 약 ${Math.round(distance(fix.position,next))}m 이동하세요.`;if(estimate.progress.remaining<10)instruction='입구까지 주변 안내와 지도를 확인해 주세요.';}
    if(!estimate.qualityOk)instruction='위치 신호를 확인하고 있어요. 주변 표지도 함께 봐 주세요.';
    $('locationInstruction').textContent=instruction;
    draw({position:[...fix.position,0],raw:[...fix.raw,0],accuracy:fix.accuracy,heading:deg===null?null:frame.headingVector(deg),headingDegrees:deg,headingUp:false,stale,follow:state.follow,route:state.route?.found?state.route:null});
  }
  function nextInstruction(fix,progress){const nodes=state.indoorRoute?.route||[],target=nodes[Math.min(nodes.length-1,progress.index+8)];if(!target||target.level!==fix.level)return '표시된 경로를 따라 이동하세요.';const yaw=Math.atan2(target.position[1]-fix.position[1],target.position[0]-fix.position[0]),turn=Math.atan2(Math.sin(yaw-fix.yaw),Math.cos(yaw-fix.yaw));return `${Math.abs(turn)<.6?'앞으로':Math.abs(turn)>2.5?'뒤쪽 통로로':turn>0?'왼쪽으로':'오른쪽으로'} 이동하세요.`;}
  function updateRoute(){if(state.mode!=='live'||!state.fix||!state.goal)return;state.route=router.route(state.fix.position,entry);}
  function ingest(position){
    if(state.mode!=='live')return;const c=position.coords;
    if(!finite(c.latitude)||!finite(c.longitude)||Math.abs(c.latitude)>90||Math.abs(c.longitude)>180||Math.abs(Date.now()-position.timestamp)>15000)return;
    const xy=frame.fromGeo(c.latitude,c.longitude),result=filter.update({x:xy[0],y:xy[1],t:position.timestamp/1000,accuracy:c.accuracy,accuracyConvention:'browser_r95'});state.rejected=!result.accepted;
    if(result.accepted){state.fix={position:result.position,raw:xy,accuracy:c.accuracy};lastGoodAt=Date.now();error='';updateRoute();}render();
  }
  function orientation(event){
    if(!['live','indoor'].includes(state.mode))return;const now=Date.now(),time=sensorEventTime(event,now);
    if(time===null)return;const value=walkingHeading.sample(event,time,screen.orientation?.angle??window.orientation??0);
    if(!value)return;
    if(state.mode==='indoor'){
      gravityHeading.anchor(value.bearing);
      if(gravityHeading.active&&time-gravityHeading.at<500)return;
    }
    state.heading=value.bearing;state.headingAt=time;
    tracker?.heading(frame.headingVector(state.heading),time,value.accuracy);if(now-lastDraw>150){lastDraw=now;render();}
  }
  function motion(event){
    if(state.mode!=='indoor'||!tracker||document.hidden)return;const now=Date.now(),time=sensorEventTime(event,now);if(time===null)return;
    const gyro=gravityHeading.sample(event,time);
    if(gyro){state.heading=gyro.bearing;state.headingAt=time;tracker.heading(frame.headingVector(gyro.bearing),time,gyro.accuracy);}
    if(strideSession){const a=event.accelerationIncludingGravity;if(a&&[a.x,a.y,a.z].every(finite)){
      const step=strideSession.detector.sample(Math.hypot(a.x,a.y,a.z),time);if(step){strideSession.amplitudes.push(step.amplitude);$('strideStatus').textContent=`${strideSession.amplitudes.length}걸음 감지 · 구간 끝에서 측정 완료를 눌러 주세요`;}
    }tracker.lastMotionAt=time;return;}
    if(pendingTransition){tracker.lastMotionAt=time;return;}const result=tracker.motion(event,time);if(result.moved||now-lastDraw>250){lastDraw=now;render();}
  }
  async function sensors(permissionPromise){
    const token=generation,permission=await permissionPromise;if(token!==generation||state.mode==='off')return;
    if(permission.orientation&&!headingAttached){window.addEventListener('deviceorientation',orientation);window.addEventListener('deviceorientationabsolute',orientation);headingAttached=true;}
    if(permission.motion&&!sensorsAttached){window.addEventListener('devicemotion',motion);sensorsAttached=true;}
    error=permission.orientation&&(state.mode!=='indoor'||permission.motion)?'':'동작·방향 접근이 필요해요. Safari에서 이 사이트의 접근을 허용해 주세요.';render();
  }
  function stopSensors(){if(sensorsAttached)window.removeEventListener('devicemotion',motion);if(headingAttached){window.removeEventListener('deviceorientation',orientation);window.removeEventListener('deviceorientationabsolute',orientation);}sensorsAttached=headingAttached=false;wakeLock?.release().catch(()=>{});wakeLock=null;}
  function stop(){generation++;if(state.watch!==null)navigator.geolocation?.clearWatch(state.watch);state.watch=null;stopSensors();tracker=null;pendingTransition=null;strideSession=null;walkingHeading.reset();gravityHeading.reset();state.mode='off';state.heading=null;state.headingAt=0;panel.hidden=true;$('locateButton').setAttribute('aria-pressed','false');draw(null);}
  async function startLive(){
    stop();state.mode='live';state.follow=true;state.fix=null;state.route=null;filter.reset();error='';lastGoodAt=0;
    if(navigator.geolocation)state.watch=navigator.geolocation.watchPosition(ingest,e=>{if(state.mode!=='live')return;error=e.code===1?'위치 접근을 허용해 주세요. 이미 건물 안이라면 실내 안내를 선택해 주세요.':'위치 신호를 기다리고 있어요. 입구에서는 실내 안내를 시작할 수 있어요.';render();},{enableHighAccuracy:true,maximumAge:1000,timeout:15000});
    else error='이 브라우저에서 위치를 받을 수 없어요. 실내 시작점을 선택해 주세요.';
    render();await setView('3d');if(state.mode==='live'){getScene()?.showCampus();collapseSheet();render();}
  }
  function startNavigation(id,details={}){setGoal(id,details);return startLive();}
  function anchorPhoto(){const p=places.find(p=>p.id===$('anchorPlace').value);if(!p)return;$('anchorPhoto').src='/photos_corrected/'+p.photo.split('/').at(-1);$('anchorName').textContent=`${p.name} · ${p.level}층`;
    dialog.querySelector('.anchor-help').textContent=p.id==='entrance'?'입구 밖에서 건물 안쪽을 향해 휴대폰을 들고 시작해 주세요. 시작 후 약 1초간 방향을 맞춘 다음 걸어 주세요.':'시작 후 휴대폰을 잠깐 평평하게 들어 방향을 맞추고, 걷는 방향을 향하게 들어 주세요. 화면을 켜 두고 층 이동 후 도착한 층을 확인해 주세요.';
  }
  function chooseIndoor(place='entrance'){if(!state.goal){notify('먼저 목적지를 선택해 주세요.');return;}$('anchorPlace').value=place;anchorPhoto();if(!dialog.open)dialog.showModal();}
  async function beginIndoor(){
    const permission=requestWalkingSensors(),place=places.find(p=>p.id===$('anchorPlace').value);dialog.close();stop();const token=generation;state.mode='indoor';state.follow=true;error='';render();
    try{const route=await onIndoor(place.id);if(token!==generation||state.mode!=='indoor')return;if(!route?.found)throw Error('이 위치에서 목적지까지 경로를 찾지 못했어요.');
      tracker=new MapParticleTracker(navigation,route.route[0].position,place.level,strideSetting?{strideCoefficient:strideSetting.coefficient}:{});
      if(place.id==='entrance'){
        const door=navigation.portals.find(p=>p.id==='main_door'),a=door?.polyline[0],b=door?.polyline.at(-1);
        if(a&&b){const dx=b[0]-a[0],dy=b[1]-a[1],east=frame.headingVector(90),north=frame.headingVector(0);
          walkingHeading.reset(Math.atan2(dx*east[0]+dy*east[1],dx*north[0]+dy*north[1])*180/Math.PI);}
      }
      state.indoorRoute=route;$('strideStatus').textContent=strideSetting?'직접 측정한 보폭 설정을 사용합니다.':'기본 보폭 설정을 사용합니다.';
      $('strideMeasure').textContent='걸음 측정 시작';onLevel(place.level);await sensors(permission);if(token!==generation)return;
      await setView('3d');getScene()?.showIndoor(false);collapseSheet();render();
      if(navigator.wakeLock)navigator.wakeLock.request('screen').then(lock=>{if(token!==generation){lock.release().catch(()=>{});return;}wakeLock=lock;}).catch(()=>{});
    }catch(e){if(token===generation){stop();notify(e.message);}}
  }
  function confirmFloor(){if(!pendingTransition||!tracker||strideSession)return;const end=state.indoorRoute.route[pendingTransition.end_index];tracker.reset(end.position,end.level);if(heading()!==null)tracker.heading(frame.headingVector(state.heading),Date.now(),20);pendingTransition=null;onLevel(end.level);notify(`${end.level}층에서 안내를 이어갑니다.`);render();}
  function setGoal(id,details={}){state.goal=id;if(details.name)state.goalName=details.name;if(finite(details.indoorDistance))state.indoorDistance=details.indoorDistance;updateRoute();}
  $('locateButton').onclick=()=>startLive();$('headingButton').onclick=()=>{if(state.mode==='off'){notify('먼저 내 위치를 켜 주세요.');return;}sensors(requestWalkingSensors());};$('stopLocation').onclick=stop;
  $('confirmEntrance').onclick=()=>chooseIndoor();$('correctIndoor').onclick=()=>chooseIndoor(tracker?.level==='2'?'elevator_2f':'entrance');$('confirmAnchor').onclick=beginIndoor;$('anchorPlace').onchange=anchorPhoto;$('confirmFloor').onclick=confirmFloor;
  $('retryWalkingSensors').onclick=()=>sensors(requestWalkingSensors());$('rerouteIndoor').onclick=async()=>{try{const r=await onIndoor(null);if(r?.found){state.indoorRoute=r;notify('현재 추정 위치에서 경로를 다시 찾았어요.');render();}}catch(e){notify(e.message);}};
  $('strideMeasure').onclick=()=>{
    if(state.mode!=='indoor'||!tracker)return;
    if(!strideSession){const distance=Number($('strideDistance').value);if(!finite(distance)||distance<5||distance>30){$('strideStatus').textContent='직접 잰 5~30m 구간의 거리를 입력해 주세요.';return;}
      strideSession={distance,detector:new StepDetector(),amplitudes:[]};$('strideMeasure').textContent='구간 끝 · 측정 완료';$('strideStatus').textContent='1초간 멈춘 뒤 직선 구간을 걸어 주세요. 끝에서도 1초 멈춘 뒤 완료를 눌러 주세요.';render();return;}
    const result=strideCalibration(strideSession.distance,strideSession.amplitudes);strideSession=null;
    if(result){strideSetting=result;try{localStorage.setItem('folded-city-stride-v1',JSON.stringify(result));}catch{}}
    const message=result?'보폭을 저장했어요. 지금 있는 장소를 다시 선택해 주세요.':'측정 조건이 맞지 않아요. 8걸음 이상 걸었는지 확인하고, 지금 있는 장소를 다시 선택해 주세요.';
    stop();notify(message);chooseIndoor();
  };
  $('followLocation').onclick=()=>{state.follow=!state.follow;$('followLocation').setAttribute('aria-pressed',String(state.follow));$('followLocation').textContent=state.follow?'위치 따라가기':'내 위치로 돌아가기';render();};
  document.addEventListener('visibilitychange',()=>{if(strideSession){stop();notify('화면이 가려져 보폭 측정을 취소했어요. 현재 위치를 다시 선택해 주세요.');chooseIndoor();return;}if(tracker)tracker.detector.reset();walkingHeading.reset();gravityHeading.reset();state.heading=null;state.headingAt=0;if(document.hidden)wakeLock?.release().catch(()=>{});render();});window.addEventListener('pagehide',stop);
  const timer=setInterval(()=>{if(state.mode!=='off')render();},1000);
  return {state,frame,filter,router,startLive,startNavigation,chooseIndoor,stop,ingest,render,setGoal,
    indoorSnapshot(){return state.mode==='indoor'&&tracker?tracker.snapshot():null;},setIndoorRoute(route){if(tracker&&state.mode==='indoor'){state.indoorRoute=route;pendingTransition=null;render();}},
    pauseFollow(){if(state.mode==='off')return;state.follow=false;$('followLocation').setAttribute('aria-pressed','false');$('followLocation').textContent='내 위치로 돌아가기';},
    destroy(){stop();clearInterval(timer);panel.remove();controls.remove();dialog.remove();layer.remove();}};
}
