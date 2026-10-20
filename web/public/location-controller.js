import {GnssFilter,rtsSmooth,circularHeading,FILTER_SETTINGS} from './gnss-filter.js';
import {createGeoFrame} from './geo-frame.js';
import {createOutdoorRouter,routeProgress,distance} from './outdoor-routing.js';
import {MEASURED_SESSIONS,loadMeasuredRecording} from './gnss-recordings.js';
import {navigationEstimate} from './live-navigation.js';
const $=id=>document.getElementById(id);
const finite=Number.isFinite;
export function createLocationController({map,campus,entry,getScene,setView,onIndoor,notify,collapseSheet}){
  const frame=createGeoFrame(map),router=createOutdoorRouter(campus,map),filter=new GnssFilter();
  const state={mode:'off',fix:null,heading:null,headingSource:null,headingAt:0,follow:true,headingUp:true,navigating:false,goalName:'',indoorDistance:0,route:null,goal:'',watch:null,records:[],raw:[],accepted:[],replayIndex:0,playing:false,offRoute:0,recording:null};
  let lastGoodAt=0,compass=null,clock=()=>Date.now(),replayTimer=null,orientationEnabled=false,headingGeneration=0;
  const controls=document.createElement('div');controls.className='location-controls';
  controls.innerHTML='<button id="locateButton" title="현재 위치 사용" aria-label="현재 위치 사용">◎ <span>내 위치</span></button><button id="headingButton" title="휴대폰 방향 사용" aria-label="휴대폰 방향 사용">➤</button>';
  document.querySelector('.map-shell').append(controls);
  const panel=document.createElement('section');panel.className='location-card';panel.id='locationCard';panel.hidden=true;panel.setAttribute('aria-label','현재 위치 안내');
  panel.innerHTML='<div class="location-summary"><span id="locationMode">실시간 위치</span><button id="stopLocation" aria-label="현재 위치 종료">×</button></div><strong id="locationInstruction">위치를 찾고 있어요</strong><p id="locationQuality" role="status"></p><p id="headingStatus"></p><div class="location-actions"><button id="followLocation" aria-pressed="true">위치 따라가기</button><button id="confirmEntrance" hidden>입구 도착 확인 →</button></div><details><summary>위치 정보</summary><p id="locationDetails"></p><p>위치는 휴대폰 OS가 GNSS·Wi-Fi·기지국 정보를 결합해 제공합니다. 실내 층과 호실은 이 정보만으로 판별하지 않습니다.</p><p id="locationAccuracyNote">정확도 원은 브라우저 원 수신 위치의 95% 반경입니다. 지도 자체도 약 14m 수준의 거친 GNSS 배치이며, 필터 공분산은 지도 정합 오차를 포함하지 않습니다.</p></details>';
  document.querySelector('.map-shell').append(panel);
  const navigationSummary=document.createElement('div');navigationSummary.className='live-navigation-summary';navigationSummary.hidden=true;
  navigationSummary.innerHTML='<span id="liveDestination"></span><strong id="liveRemaining"></strong><small id="liveDistanceNote"></small>';
  panel.insertBefore(navigationSummary,$('locationInstruction'));
  const tools=document.createElement('section');tools.className='location-lab';
  tools.innerHTML='<h3>GNSS 재현·로그</h3><p>수신 위치와 보정 위치를 나란히 비교합니다. 로그는 이 브라우저 메모리에만 보관하며 서버에 전송하지 않습니다.</p><button id="startGnssDemo">합성 GNSS 재현 시작</button><button id="startRecordedGnss">실측 야외 GNSS 로그 재생</button><div id="replayControls" hidden><button id="pauseGnssDemo">일시 정지</button><label>재현 시점 <input id="gnssReplayProgress" type="range" min="0" max="100" value="0"></label></div><button id="exportGnssLog">원본·칼만·RTS 로그 저장</button><label class="log-file">저장 로그 불러와 후보정 <input id="importGnssLog" type="file" accept=".json,application/json"></label><p id="gnssLabStatus"></p><p>상태 [x,y,vx,vy], 등속도 모델, 정확도 기반 R, 2차원 혁신 χ²(99%) 게이트, Joseph 공분산 갱신, RTS 고정구간 평활화. 필터는 RTK나 다중경로 오차의 절대 보정을 대신하지 않습니다.</p>';
  $('projectInfo').append(tools);
  const measuredSelector=document.createElement('label');measuredSelector.textContent='실측 촬영 구간 ';
  const measuredSession=document.createElement('select');measuredSession.id='recordedGnssSession';
  for(const [id,definition] of Object.entries(MEASURED_SESSIONS)){const option=document.createElement('option');option.value=id;option.textContent=definition.label;measuredSession.append(option);}
  measuredSelector.append(measuredSession);tools.insertBefore(measuredSelector,$('startRecordedGnss'));
  $('startRecordedGnss').textContent='학생회관 계측 GNSS 재생';
  tools.insertBefore(measuredSelector,$('startGnssDemo'));tools.insertBefore($('startRecordedGnss'),$('startGnssDemo'));
  tools.querySelector('h3').textContent='학생회관 실측 GNSS 기록';
  tools.querySelector('p').textContent='직접 계측한 다섯 DB의 저장 위치를 재생합니다. 기본 구간은 학생회관 입구 주변 실외입니다. 주황색은 원본, 파란색은 칼만 처리이며 촬영 궤적을 길찾기 경로로 사용하지 않습니다.';
  const syntheticTools=document.createElement('details');syntheticTools.id='syntheticGnssTools';
  const syntheticSummary=document.createElement('summary');syntheticSummary.textContent='필터 시험용 합성 자료 · 실측 아님';
  const syntheticNote=document.createElement('p');syntheticNote.textContent='약 100m 떨어진 OSM 보행로에서 시작하는 가상 입력입니다. 학생회관 계측 위치나 실측 보정 결과가 아닙니다.';
  syntheticTools.append(syntheticSummary,syntheticNote,$('startGnssDemo'));tools.append(syntheticTools);
  $('startGnssDemo').textContent='합성 입력으로 필터 시험';
  const recordingSource=document.createElement('p');recordingSource.id='recordingSource';recordingSource.hidden=true;
  panel.insertBefore(recordingSource,$('locationQuality'));
  const recordingProgress=document.createElement('p');recordingProgress.id='recordingProgress';recordingProgress.hidden=true;
  panel.insertBefore(recordingProgress,$('headingStatus'));
  const recordingControls=document.createElement('div');recordingControls.className='recording-controls';recordingControls.hidden=true;
  recordingControls.innerHTML='<button id="pauseMeasuredReplay">일시 정지</button><label>수집 시점 <input id="measuredReplayProgress" type="range" min="0" max="100" value="0"></label>';
  panel.insertBefore(recordingControls,panel.querySelector('details'));
  const comparison=document.createElement('p'),comparisonLink=document.createElement('a');comparisonLink.href='/gnss-audit.html';comparisonLink.textContent='실측 GNSS와 동시각 SLAM 위치 대조 →';comparison.append(comparisonLink);tools.append(comparison);
  const svg=document.createElementNS('http://www.w3.org/2000/svg','g');svg.id='locationLayer';$('mapSvg').append(svg);
  function heading(){if(clock()-state.headingAt>5000)return null;return state.heading;}
  function render(){
    const scene=getScene(),fix=state.fix;
    if(state.mode==='off'){svg.replaceChildren();scene?.updateLocation?.(null);return;}
    const stale=!lastGoodAt||clock()-lastGoodAt>10000;
    $('locationMode').textContent=state.mode==='demo'?'합성 필터 시험 · 실측 아님':state.mode==='log'?'실측 기록 재생 · 현재 위치 아님':'실시간 위치';
    $('locationCard').dataset.mode=state.mode;
    const measured=state.mode==='log'&&state.recording?.kind==='measured';
    navigationSummary.hidden=!(state.mode==='live'&&state.goal);
    $('liveDestination').textContent=state.goalName||'선택한 목적지';
    recordingSource.hidden=!measured;recordingProgress.hidden=!measured;recordingControls.hidden=!measured;
    if(measured){
      recordingSource.textContent=state.recording.source;
      const timestamp=replay[state.replayIndex]?.timestamp;
      recordingProgress.textContent=`${state.replayIndex+1} / ${replay.length} · ${new Date(timestamp).toLocaleString('ko-KR',{timeZone:'Asia/Seoul',hour12:false})} · 주황: 원본 / 파랑: 칼만`;
      $('pauseMeasuredReplay').textContent=state.playing?'일시 정지':state.replayIndex>=replay.length-1?'처음부터':'재생';
      $('measuredReplayProgress').value=Math.round(state.replayIndex/Math.max(1,replay.length-1)*100);
    }
    let message='수신 대기 중';
    if(fix){
      message=stale?'위치 수신이 늦어요. 마지막 위치를 표시합니다.':fix.accuracy>25?'정확도가 낮아요. 주변 표지와 위치를 확인해 주세요.':state.lastRejected?'위치 튐을 감지해 마지막 정상 위치를 유지합니다.':'칼만 보정 위치를 표시하고 있어요.';
      $('locationQuality').textContent=`${message} · 수신 반경 ${Math.round(fix.accuracy)}m`;
      const deg=heading();$('headingStatus').hidden=measured&&deg===null;$('headingStatus').textContent=deg===null?'헤딩 없음 · 방향 사용을 허용하거나 이동해 주세요.':`${Math.round(deg)}° · ${state.headingSource}`;
      $('locationAccuracyNote').textContent=(fix.accuracyConvention==='unspecified_radius'?'정확도 원은 저장된 iOS 보고 반경이며 신뢰수준은 확정하지 않습니다.':'정확도 원은 브라우저 원 수신 위치의 95% 반경입니다.')+' 지도 자체도 약 14m 수준의 거친 GNSS 배치이며 필터 공분산은 지도 정합 오차를 포함하지 않습니다.';
      const scope=$('routeDistanceScope');if(scope)scope.textContent='입구 도착을 확인한 뒤 이어지는 실내 경로';
      const age=Math.max(0,(clock()-lastGoodAt)/1000);
      $('locationDetails').textContent=`마지막 정상 수신 ${age.toFixed(0)}초 전 / 이상치 ${filter.rejected}회 / σa=${FILTER_SETTINGS.accelerationSigma}m/s² / χ² 임계값 ${FILTER_SETTINGS.gateChi2.toFixed(2)}`;
      const active=state.route?.found&&!stale&&fix.accuracy<=25&&!state.lastRejected;
      let instruction=state.route?.found?'신호가 안정되면 안내를 이어갑니다.':'목적지를 선택하면 현재 위치에서 입구까지 안내합니다.';
      // Entry remains a user confirmation, including when indoor GNSS is poor.
      $('confirmEntrance').hidden=state.mode!=='live'||!state.goal;
      $('confirmEntrance').textContent='입구에 도착했어요 →';
      const estimate=navigationEstimate({position:fix.position,route:state.route,entry,indoorDistance:state.indoorDistance,accuracy:fix.accuracy,stale,rejected:state.lastRejected});
      $('liveRemaining').textContent=estimate.knownRemaining===null?'실외 보행로 연결 확인 중':`등록 경로 ${Math.round(estimate.knownRemaining)}m · 약 ${estimate.estimatedMinutes}분`;
      $('liveDistanceNote').textContent=estimate.knownRemaining===null?'교내에 등록된 OSM 보행로에서 안내합니다.':`실외 ${Math.round(estimate.outdoorRemaining)}m + 입구 이후 ${state.indoorDistance.toFixed(1)}m${estimate.entranceGap>1?` · 입구 연결 ${Math.round(estimate.entranceGap)}m는 별도 확인`:''}. 시간은 1.25m/s 보행 추정, 신호·계단·승강기 대기 제외.`;
      if(!estimate.qualityOk)$('liveRemaining').textContent='신호가 안정되면 안내를 이어갑니다';
      if(active){
        const p=routeProgress(fix.position,state.route.points),near=distance(fix.position,entry)<Math.max(12,fix.accuracy);
        if(near){instruction='학생회관 입구 주변입니다. 입구를 확인해 주세요.';$('confirmEntrance').hidden=false;}
        else if(p&&p.remaining<10){instruction=`등록 보행로 안내 끝 · 입구까지 약 ${Math.round(state.route.endGap)}m 연결로 미등록. 현장에서 입구를 확인해 주세요.`;$('confirmEntrance').hidden=false;}
        else if(p){
          const next=state.route.points[p.index],d=distance(fix.position,next),vec=[next[0]-fix.position[0],next[1]-fix.position[1]];
          const unit=frame.headingVector(0),east=frame.headingVector(90),bearing=(Math.atan2(vec[0]*east[0]+vec[1]*east[1],vec[0]*unit[0]+vec[1]*unit[1])*180/Math.PI+360)%360;
          const diff=deg===null?0:((bearing-deg+540)%360)-180;
          const direction=deg===null?'경로를 따라':Math.abs(diff)<30?'앞으로':Math.abs(diff)>150?'뒤쪽 경로로':diff>0?'오른쪽 방향으로':'왼쪽 방향으로';
          instruction=p.crossTrack>Math.max(15,fix.accuracy+14)?'경로에서 벗어났어요. 보행로 위치를 확인해 주세요.':`${direction} ${Math.round(d)}m · 입구 접근 구간까지 ${Math.round(p.remaining)}m`;
          if(p.crossTrack>Math.max(15,fix.accuracy+14)&&state.offRoute>=3)instruction='현재 위치에서 경로를 다시 찾습니다.';
        }
      }
      if(state.mode==='log')instruction=state.recording?.session==='outside'?'학생회관 입구 주변 · 실측 GNSS':`${state.recording?.label||'저장 로그'} · GNSS 수집 기록`;
      $('locationInstruction').textContent=instruction;
      const data={position:[...fix.position,0],raw:[...fix.raw,0],accuracy:fix.accuracy,heading:deg===null?null:frame.headingVector(deg),headingDegrees:deg,headingUp:state.mode==='live'&&state.headingUp,stale,follow:state.follow,route:state.route?.found?state.route:null,showTrack:state.mode==='log',track:state.accepted.slice(-600),rawTrack:state.raw.slice(-600)};
      scene?.updateLocation?.(data);
      svg.replaceChildren();
      function shape(tag,attrs){const el=document.createElementNS(svg.namespaceURI,tag);for(const [k,v] of Object.entries(attrs))el.setAttribute(k,v);svg.append(el);return el;}
      if(data.route)shape('path',{d:data.route.points.map((p,i)=>`${i?'L':'M'}${p[0]},${-p[1]}`).join(' '),fill:'none',stroke:'#167a52','stroke-width':1});
      if(data.showTrack)for(const [points,color,dash] of [[data.rawTrack,'#d9974a','1 .6'],[data.track,'#2789e9','none']])if(points.length>1)shape('path',{d:points.map((p,i)=>`${i?'L':'M'}${p[0]},${-p[1]}`).join(' '),fill:'none',stroke:color,'stroke-width':.35,'stroke-dasharray':dash});
      shape('circle',{cx:fix.raw[0],cy:-fix.raw[1],r:fix.accuracy,fill:'#468be61a',stroke:'#468be6','stroke-width':.15});
      shape('circle',{cx:fix.raw[0],cy:-fix.raw[1],r:.65,fill:'#ec9f46'});
      if(data.heading){const [x,y]=data.heading,p=fix.position;shape('path',{d:`M${p[0]},${-p[1]} L${p[0]+x*5-y*2},${-(p[1]+y*5+x*2)} L${p[0]+x*5+y*2},${-(p[1]+y*5-x*2)} Z`,fill:'#2885df66'});}
      shape('circle',{cx:fix.position[0],cy:-fix.position[1],r:.85,fill:stale?'#87918e':'#2789e9',stroke:'#fff','stroke-width':.28});
      if(state.follow&&document.getElementById('map3d').hidden){$('mapSvg').setAttribute('viewBox',`${fix.position[0]-65} ${-fix.position[1]-65} 130 130`);}
    }else{$('locationQuality').textContent=state.error||message;}
  }
  function updateRoute(){
    if(!state.fix)return;
    if(state.mode==='log'){state.route=null;render();return;}
    state.route=router.route(state.fix.position,entry);
    if(!state.route.found)notify(state.route.message);
    else $('gnssLabStatus').textContent=`OSM 보행로 ${state.route.distance.toFixed(1)}m / 도로와 현재 위치의 이격 ${state.route.startGap.toFixed(1)}m / 입구 연결로 미등록 ${state.route.endGap.toFixed(1)}m. 미등록 구간에는 경로를 그리지 않습니다.`;
    render();
  }
  function ingest(position){
    const c=position.coords,t=position.timestamp/1000;
    if(!finite(c.latitude)||!finite(c.longitude)||Math.abs(c.latitude)>90||Math.abs(c.longitude)>180)return;
    if(state.mode==='live'&&(Math.abs(Date.now()-position.timestamp)>15000)){state.error='오래된 위치를 받았습니다.';render();return;}
    const xy=frame.fromGeo(c.latitude,c.longitude),accuracyConvention=state.mode==='live'?'browser_r95':c.accuracyConvention||'browser_r95',result=filter.update({x:xy[0],y:xy[1],t,accuracy:c.accuracy,accuracyConvention});
    state.records.push({timestamp:position.timestamp,coords:{latitude:c.latitude,longitude:c.longitude,accuracy:c.accuracy,accuracyConvention,heading:c.heading??null,speed:c.speed??null},accepted:result.accepted,reason:result.reason});
    state.raw.push(xy);state.lastRejected=!result.accepted;
    if(result.accepted){lastGoodAt=clock();state.error='';state.fix={position:result.position,raw:xy,accuracy:c.accuracy,accuracyConvention,t};state.accepted.push(result.position);}
    else if(!state.fix)state.error=c.accuracy>50?'정확도 반경이 50m를 넘어 정상 수신을 기다립니다.':'사용 가능한 새 위치를 기다립니다.';
    if(compass&&clock()-compass.at<3000){setHeading(compass.value,compass.source);}
    else if(result.accepted&&finite(c.heading)&&finite(c.speed)&&c.speed>=.7)setHeading(c.heading,'이동 방향 · 진북');
    else {state.heading=null;state.headingSource=null;}
    if(result.accepted&&state.goal&&state.mode!=='log'){
      if(!state.route)updateRoute();
      else if(state.route.found){const p=routeProgress(result.position,state.route.points);state.offRoute=p&&p.crossTrack>Math.max(15,c.accuracy+14)?state.offRoute+1:0;if(state.offRoute>=3){state.offRoute=0;updateRoute();}}
    }
    render();
  }
  function setHeading(value,source){state.heading=circularHeading(state.heading,value);state.headingSource=source;state.headingAt=clock();}
  function orientation(event){
    if(state.mode!=='live')return;
    let value=null,source='휴대폰 나침반 · 자북';
    if(finite(event.webkitCompassHeading)&&(!finite(event.webkitCompassAccuracy)||(event.webkitCompassAccuracy>=0&&event.webkitCompassAccuracy<=40)))value=event.webkitCompassHeading;
    else if(event.absolute&&finite(event.alpha)&&Math.abs(event.beta||0)<45&&Math.abs(event.gamma||0)<45){value=360-event.alpha;source='절대 방향 · 기기 기준';}
    if(value!==null){value+=(screen.orientation?.angle||0);compass={value,source,at:clock()};setHeading(value,source);render();}
  }
  async function enableHeading(){
    const generation=++headingGeneration;
    try{if(typeof DeviceOrientationEvent==='undefined')throw Error('이 브라우저는 휴대폰 방향 센서를 제공하지 않습니다.');
      if(typeof DeviceOrientationEvent.requestPermission==='function'&&await DeviceOrientationEvent.requestPermission()!=='granted')throw Error('동작 및 방향 접근을 허용해 주세요.');
      if(generation!==headingGeneration||state.mode!=='live')return;
      window.addEventListener('deviceorientation',orientation);window.addEventListener('deviceorientationabsolute',orientation);orientationEnabled=true;notify('방향 센서를 켰어요. 나침반은 주변 금속의 영향을 받을 수 있어요.');
    }catch(e){notify(e.message);}
  }
  function reset(mode){stop();state.mode=mode;state.logMode=mode;state.recording=null;state.records=[];state.raw=[];state.accepted=[];state.fix=null;state.route=null;state.error='';state.lastRejected=false;state.offRoute=0;lastGoodAt=0;filter.reset();panel.hidden=false;clock=()=>Date.now();}
  function stop(){
    headingGeneration++;if(state.watch!==null)navigator.geolocation?.clearWatch(state.watch);state.watch=null;
    if(orientationEnabled){window.removeEventListener('deviceorientation',orientation);window.removeEventListener('deviceorientationabsolute',orientation);orientationEnabled=false;}
    clearTimeout(replayTimer);state.playing=false;state.heading=null;compass=null;state.mode='off';panel.hidden=true;$('replayControls').hidden=true;
    getScene()?.updateLocation?.(null);svg.replaceChildren();$('locateButton').setAttribute('aria-pressed','false');
  }
  async function startLive({navigate=false}={}){
    if(!window.isSecureContext){notify('현재 위치는 HTTPS 또는 localhost에서 사용할 수 있어요.');return;}
    if(!navigator.geolocation){notify('이 브라우저는 위치 정보를 제공하지 않습니다.');return;}
    reset('live');state.follow=true;state.navigating=navigate;state.headingUp=true;$('followLocation').setAttribute('aria-pressed','true');$('followLocation').textContent='위치 따라가기';$('locateButton').setAttribute('aria-pressed','true');render();
    state.watch=navigator.geolocation.watchPosition(ingest,error=>{state.error=error.code===1?'위치 권한이 꺼져 있어요. 브라우저 설정에서 허용해 주세요.':error.code===3?'위치 수신 시간이 초과됐어요. 실외에서 다시 시도해 주세요.':'위치를 받을 수 없어요.';$('locationQuality').textContent=state.error;notify(state.error);},{enableHighAccuracy:true,maximumAge:1000,timeout:15000});
    await setView('3d');if(state.mode==='live'){getScene()?.showCampus();collapseSheet();render();}
  }
  async function startNavigation(id,details={}){state.goal=id;state.goalName=details.name||'';state.indoorDistance=details.indoorDistance||0;await startLive({navigate:true});}
  let replay=[];
  function step(index){
    state.replayIndex=index;clock=()=>replay[index].timestamp;ingest(replay[index]);$('gnssReplayProgress').value=Math.round(index/Math.max(1,replay.length-1)*100);
  }
  function play(){if(!state.playing)return;const next=state.replayIndex+1;if(next>=replay.length){state.playing=false;$('pauseGnssDemo').textContent='처음부터';render();return;}step(next);replayTimer=setTimeout(play,500);}
  async function startReplay(records,mode='demo',recording=null){
    if(!Array.isArray(records)||!records.length||records.length>20000)throw Error('로그는 1~20,000개 위치여야 합니다.');
    for(const r of records)if(!finite(r.timestamp)||!finite(r.coords?.latitude)||!finite(r.coords?.longitude)||!finite(r.coords?.accuracy))throw Error('위치 로그 형식을 확인해 주세요.');
    reset(mode);replay=records;state.recording=recording;state.goal||='prime_hall';$('replayControls').hidden=false;$('pauseGnssDemo').textContent='일시 정지';step(0);state.playing=true;replayTimer=setTimeout(play,500);
    await setView('3d');getScene()?.showCampus();collapseSheet();render();
  }
  async function startDemo(){const r=await fetch('/data/gnss_demo.json');if(!r.ok)throw Error('재현 입력을 읽지 못했습니다.');const demo=await r.json();await startReplay(demo.records,'demo');}
  async function startRecorded(id='outside'){
    const value=await loadMeasuredRecording(id);measuredSession.value=id;
    await startReplay(value.records,'log',{kind:'measured',session:id,label:value.label,source:value.source,sha256:value.sha256});
    $('evidenceDialog').close();
    if(id!=='outside')notify('실내 저장 GNSS입니다. 층과 복도를 자동 추정한 위치가 아닙니다.');
  }
  function seek(index){if(!replay.length||!finite(index))return;index=Math.max(0,Math.min(replay.length-1,Math.round(index)));clearTimeout(replayTimer);const mode=state.mode,goal=state.goal,recording=state.recording;reset(mode);state.goal=goal;state.recording=recording;$('replayControls').hidden=false;for(let i=0;i<=index;i++)step(i);state.playing=false;$('pauseGnssDemo').textContent='재생';render();}
  function exportLog(){
    if(!state.records.length){notify('저장할 수신 로그가 없어요.');return;}
    const mode=state.logMode||state.mode;
    const value={schema:'folded-city-gnss-v1',mode,simulated:mode==='demo',recording:state.recording,settings:filter.settings,coordinateFrame:'registered SLAM XY metres',georeference:map.georeference,records:state.records,kalman:filter.history,rts:rtsSmooth(filter.history)};
    const blob=new Blob([JSON.stringify(value,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`folded-city-${mode}-gnss.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    $('gnssLabStatus').textContent=`${state.records.length}개 원 수신과 칼만·RTS 결과를 저장했습니다. 실시간 안내는 미래 정보를 쓰지 않으며 RTS는 후보정 전용입니다.`;
  }
  $('locateButton').onclick=startLive;$('headingButton').onclick=()=>state.mode==='live'?enableHeading():notify('먼저 내 위치를 켜 주세요.');$('stopLocation').onclick=stop;
  $('followLocation').onclick=()=>{state.follow=!state.follow;$('followLocation').setAttribute('aria-pressed',String(state.follow));$('followLocation').textContent=state.follow?'위치 따라가기':'따라가기 꺼짐';render();};
  $('confirmEntrance').onclick=()=>{stop();onIndoor();};
  $('startGnssDemo').onclick=()=>{$('evidenceDialog').close();startDemo().catch(e=>notify(e.message));};
  $('startRecordedGnss').onclick=()=>startRecorded(measuredSession.value).catch(e=>notify(e.message));
  function toggleReplay(){clearTimeout(replayTimer);if(state.replayIndex>=replay.length-1)seek(0);state.playing=!state.playing;$('pauseGnssDemo').textContent=state.playing?'일시 정지':'재생';render();if(state.playing)play();}
  $('pauseGnssDemo').onclick=toggleReplay;$('pauseMeasuredReplay').onclick=toggleReplay;
  $('gnssReplayProgress').oninput=e=>seek(Math.round(Number(e.target.value)/100*(replay.length-1)));
  $('measuredReplayProgress').oninput=e=>seek(Math.round(Number(e.target.value)/100*(replay.length-1)));
  $('exportGnssLog').onclick=exportLog;
  $('importGnssLog').onchange=async e=>{try{const file=e.target.files[0];if(!file)return;if(file.size>20000000)throw Error('20MB 이하 로그를 선택해 주세요.');const value=JSON.parse(await file.text());if(value.schema!=='folded-city-gnss-v1')throw Error('Folded City 위치 로그를 선택해 주세요.');await startReplay(value.records,value.simulated?'demo':'log',value.recording||null);$('evidenceDialog').close();}catch(error){notify(error.message);}finally{e.target.value='';}};
  setInterval(()=>{if(state.mode==='live')render();},1000);
  window.addEventListener('pagehide',stop);
  return {state,filter,frame,router,startLive,startNavigation,startDemo,startRecorded,startReplay,seek,stop,ingest,render,
    pauseFollow(){if(state.mode==='off'||!state.follow)return;state.follow=false;$('followLocation').setAttribute('aria-pressed','false');$('followLocation').textContent='내 위치로 돌아가기';render();},
    setGoal(id,details={}){state.goal=id;if(details.name)state.goalName=details.name;if(finite(details.indoorDistance))state.indoorDistance=details.indoorDistance;if(state.mode!=='off')updateRoute();}};
}
