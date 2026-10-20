const $ = (id) => document.getElementById(id);
import {loadingScreen} from './loading-screen.js?v=3';
import {createWalkController} from './walk-controller.js?v=journey-2';
import {createBottomSheet} from './sheet-controller.js?v=safari-1';
import {createLocationController} from './location-controller.js';
import {applyCampusLabels,campusBuildingMatches} from './campus-data.js';
const bottomSheet=createBottomSheet();
const svgNS = 'http://www.w3.org/2000/svg';
const state = { level: 'all', view: '3d', sceneMode:'campus', selectedBuilding:null, route: null, photoIndex: 0, progress: 0, playing: false, box: [-40,-25,65,60], three: null, threeLoading:null };
const escapeHTML = (s) => String(s).replace(/[&<>"']/g, (c) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const [map, places, graph, photos, navigation, campusSource] = await Promise.all([
  'map','catalog','graph','photos','navigation','campus_source'
].map((name) => fetch(`/data/${name}.json`, {signal:AbortSignal.timeout(30000)}).then((r) => { if (!r.ok) throw new Error(`${name} 자료를 읽지 못했습니다.`); return r.json(); })));
const campus=applyCampusLabels(campusSource);
const placeById = Object.fromEntries(places.map((p) => [p.id,p]));
const nodeById = Object.fromEntries(graph.nodes.map((n) => [n.id,n]));
const photoById = Object.fromEntries(photos.map((p) => [p.id,p]));
for(const p of places){
  const id=p.evidence?.session_node;if(!id||!p.photo||photoById[id])continue;
  const [session,node]=id.split(':');const photo={id,session,node:Number(node),url:p.photo,position:p.position};
  photos.push(photo);photoById[id]=photo;
}
function routePolicy(){return $('routePolicy').value;}
const walker=createWalkController({navigation,ensureScene:()=>setView('3d'),getScene:()=>state.three,getGoal:()=>$('destination').value,getPolicy:routePolicy,onError:toast,onStart:()=>location.stop()});
const location=createLocationController({map,campus,navigation,places,entry:placeById.outside_start.position,getScene:()=>state.three,setView,notify:toast,collapseSheet:()=>bottomSheet.set(0),onLevel:selectLevel,async onIndoor(place){
  if(place)$('start').value=place;
  const route=await findRoute();bottomSheet.set(0);return route;
}});
$('campusCount').textContent=campus.buildings.length;

function element(tag, attrs, parent) {
  const el = document.createElementNS(svgNS, tag);
  for (const [k,v] of Object.entries(attrs)) el.setAttribute(k,v);
  parent?.appendChild(el); return el;
}
function pathFrom(points, close=false) { return points.map((p,i) => `${i?'L':'M'}${p[0].toFixed(3)},${(-p[1]).toFixed(3)}`).join(' ') + (close?' Z':''); }
function toast(text) { $('toast').textContent=text; $('toast').style.display='block'; clearTimeout(state.toastTimer); state.toastTimer=setTimeout(()=>$('toast').style.display='none',4200); }
function applyBox() { $('mapSvg').setAttribute('viewBox',state.box.join(' ')); }
function fitPoints(points) {
  if (!points.length) return;
  const xs=points.map(p=>p[0]), ys=points.map(p=>-p[1]);
  const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
  const width=Math.max(maxX-minX+15,25), height=Math.max(maxY-minY+15,25);
  state.box=[(minX+maxX-width)/2,(minY+maxY-height)/2,width,height]; applyBox();
}
function renderMap() {
  $('osmLayer').replaceChildren(); $('floorLayer').replaceChildren(); $('placesLayer').replaceChildren();
  for (const feature of map.vectors.osm) {
    const pts=feature.xy_m; if (!pts?.length) continue;
    const building=feature.kind==='building';
    element('path',{d:pathFrom(pts,building),fill:building?'#e1e6dc':'none',stroke:building?'#c8d1c3':'#fff','stroke-width':building?.12:1.0,'stroke-linejoin':'round'},$('osmLayer'));
    if (building && feature.name && pts.some(p=>p[0]>-60&&p[0]<50&&p[1]>-50&&p[1]<45)) {
      const x=pts.reduce((a,p)=>a+p[0],0)/pts.length,y=pts.reduce((a,p)=>a+p[1],0)/pts.length;
      const text=element('text',{x,y:-y,'font-size':.8,'text-anchor':'middle',fill:'#a0ac9b'},$('osmLayer'));text.textContent=feature.name;
    }
  }
  for (const [id,level] of Object.entries(navigation.levels)) {
    if (!['all',id].includes(state.level) && !(state.level==='outside'&&id==='1')) continue;
    const opacity=state.level==='all'?(id==='1'?.75:.4):1;
    element('path',{d:level.floor_rings.map(r=>pathFrom(r,true)).join(' '),fill:'#e0e4de',stroke:'#c0c9bd','fill-opacity':opacity,'stroke-width':.08,'fill-rule':'evenodd'},$('floorLayer'));
    element('path',{d:level.walkable_rings.map(r=>pathFrom(r,true)).join(' '),fill:id==='1'?'#eaf2e6':'#d0e4dd','fill-opacity':opacity,stroke:'none','fill-rule':'evenodd'},$('floorLayer'));
    element('path',{d:level.obstacle_rings.map(r=>pathFrom(r,true)).join(' '),fill:'#64766a','fill-opacity':state.level==='all'?.4:.85,stroke:'none','fill-rule':'evenodd'},$('floorLayer'));
  }
  if(state.level!=='outside') for(const portal of navigation.portals.filter(p=>p.kind==='stairs')) {
    element('path',{d:pathFrom(portal.polyline),fill:'none',stroke:'#bda582','stroke-width':.7,'stroke-dasharray':'.12 .16'},$('floorLayer'));
  }
  for (const p of places) {
    if (state.level!=='all' && state.level!==p.level) continue;
    const g=element('g',{class:'place-marker',tabindex:0,role:'button','aria-label':p.name,transform:`translate(${p.position[0]},${-p.position[1]})`},$('placesLayer'));
    element('circle',{r:.6,fill:'#fff',stroke:p.id==='prime_hall'?'#173b32':'#86a48a','stroke-width':.12},g);
    element('circle',{r:.23,fill:p.id==='prime_hall'?'#173b32':'#127a5c'},g);
    const text=element('text',{x:.9,y:.27,'font-size':.8,fill:'#355344','font-weight':p.id==='prime_hall'?700:500},g);text.textContent=p.name;
    g.addEventListener('click',()=>choosePlace(p.id));g.addEventListener('keydown',(e)=>{if(e.key==='Enter')choosePlace(p.id);});
  }
  renderRoute();
}
function renderRoute() {
  $('routeLayer').replaceChildren();
  const route=state.route?.route;
  if (!route?.length) return;
  for (let i=1;i<route.length;i++) {
    const a=route[i-1],b=route[i];
    if (state.level!=='all' && ![a.level,b.level].includes(state.level) && ![a.level,b.level].includes('stairs')) continue;
    const transition=b.kind==='stairs'||b.kind==='door';
    element('path',{d:pathFrom([a.position,b.position]),fill:'none',stroke:transition?'#b87926':b.level==='2'?'#287a8c':'#127a5c','stroke-width':.28,'stroke-linecap':'round','stroke-dasharray':transition?'.2 .18':''},$('routeLayer'));
  }
  for (const [index,label] of [[0,'출발'],[route.length-1,'도착']]) {
    const p=route[index].position;
    element('circle',{cx:p[0],cy:-p[1],r:.46,fill:index?'#173b32':'white',stroke:'#127a5c','stroke-width':.15},$('routeLayer'));
    const t=element('text',{x:p[0],y:-p[1]-1.0,'text-anchor':'middle','font-size':.8,fill:'#173b32','font-weight':700},$('routeLayer'));t.textContent=label;
  }
}
function selectLevel(level) {
  state.level=level; document.querySelectorAll('[data-level]').forEach(b=>b.classList.toggle('active',b.dataset.level===level));
  updateSceneHUD();
  renderMap(); state.three?.setLevel(level);
}
function searchPlaces(query) {
  const q=query.trim().toLowerCase();
  const items=places.filter(p=>!q||[p.name,p.address,...p.aliases].some(s=>s.toLowerCase().includes(q)));
  $('results').replaceChildren();
  const shown=q?items.slice(0,6):['prime_hall','career_office','classical_music','food_market','cu'].map(id=>placeById[id]).filter(Boolean);
  $('resultsHeading').textContent=q?'검색 결과':'학생회관의 장소';
  for (const p of shown) {
    const b=document.createElement('button');b.className='search-result';
    b.dataset.place=p.id;
    b.innerHTML=`<img src="/photos_corrected/${escapeHTML(p.photo.split('/').at(-1))}" alt="" loading="lazy" width="72" height="64"><span class="result-copy"><strong>${escapeHTML(p.name)}</strong><small>${p.id==='food_market'?'1층 식당 입구':p.level==='outside'?'실외':p.level+'층'} <span>· 제1학생회관</span></small></span><span class="result-arrow" aria-hidden="true">↗</span>`;
    b.addEventListener('click',()=>choosePlace(p.id));$('results').appendChild(b);
  }
  const buildings=campus.buildings.filter(b=>campusBuildingMatches(b,q));
  for(const b of buildings.slice(0,4)) {
    const button=document.createElement('button');button.className='search-result';
    button.dataset.building=b.id;
    button.innerHTML=`<span class="building-icon" aria-hidden="true">⌂</span><span class="result-copy"><strong>${escapeHTML(b.n)}</strong><small>캠퍼스 지도에서 보기</small></span><span class="result-arrow" aria-hidden="true">↗</span>`;
    button.addEventListener('click',async()=>{await setView('3d');state.three?.selectBuilding(b.id);});$('results').appendChild(button);
  }
  $('resultsCount').textContent=`${shown.length+Math.min(buildings.length,4)}곳`;
  if (!items.length&&!buildings.length) { const p=document.createElement('p');p.className='empty-note';p.textContent='등록된 장소를 찾지 못했습니다.';$('results').appendChild(p); }
}
function showJourney(name){
  const choosing=name==='places';
  $('placesPanel').hidden=!choosing;$('journeyPanel').hidden=choosing;
  document.querySelector('.workspace').dataset.journey=name;
  $('sheetScroll').scrollTop=0;bottomSheet.reveal();
}
function updateDestinationPreview(){
  const p=placeById[$('destination').value];
  $('destinationPreview').innerHTML=`<img src="/photos_corrected/${escapeHTML(p.photo.split('/').at(-1))}" alt="" width="64" height="64"><div><small>도착할 곳</small><strong>${escapeHTML(p.name)}</strong><span>${p.id==='food_market'?'1층 식당 입구까지 안내':p.level==='outside'?'학생회관 실외':`제1학생회관 · ${p.level}층`}</span></div>`;
}
async function choosePlace(id) { $('destination').value=id;$('search').blur();updateDestinationPreview();bottomSheet.set(1);showJourney('route');await findRoute(); }
let routeRequest=0;
async function findRoute() {
  const token=++routeRequest,liveStart=location.indoorSnapshot();showJourney('route');updateDestinationPreview();
  location.setGoal($('destination').value);
  if(walker.isActive())walker.exit();
  $('routeButton').disabled=true;$('journeyPanel').setAttribute('aria-busy','true');state.playing=false;$('playButton').textContent='▶';
  $('routeResult').innerHTML='<p class="route-note">경로를 찾고 있어요…</p>';
  try {
    const query=new URLSearchParams({start:$('start').value,end:$('destination').value,mode:'walk',policy:routePolicy()});
    if(liveStart){query.set('x',liveStart.position[0]);query.set('y',liveStart.position[1]);query.set('level',liveStart.level);}
    const response=await fetch(`/api/route?${query}`); const r=await response.json();if(token!==routeRequest)return;if(!response.ok)throw new Error(r.error);
    state.route=r; $('routePlayer').hidden=!r.found;
    if (!r.found) {location.setIndoorRoute(null);$('routeResult').innerHTML=`<p class="route-note">${escapeHTML(r.message)}</p>`;renderRoute();$('replayLayer').replaceChildren();state.three?.updateRoute([]);return;}
    const destinationNote=$('destination').value==='food_market'?'식당으로 내려가는 1층 입구까지 안내합니다. 식당 내부는 포함하지 않습니다.':r.elevator_used?'엘리베이터 운행·문턱·접근성을 확인해 주세요. 대기시간 미포함이며 무장애 경로를 보장하지 않습니다.':'실제 개방 여부와 현장 안내를 확인해 주세요.';
    const routeName={shortest:'가장 짧은 도보 경로',main_stairs:'메인 계단 이용',sub_stairs:'오른쪽 서브계단 이용',elevator:'엘리베이터 이용'}[r.policy];
    $('routeResult').innerHTML=`<div class="distance-row"><strong>${r.distance_m.toLocaleString()}<small> m</small></strong><span>${routeName}</span></div><button id="walkRoute" class="route-walk-button">사진으로 경로 미리보기 <span aria-hidden="true">→</span></button><p class="route-note">${destinationNote}</p><details class="route-details"><summary>다른 경로 비교 <span aria-hidden="true">⌄</span></summary><div class="route-alternatives" id="routeAlternatives"></div></details><details class="route-details"><summary>이동 경로 상세 <span aria-hidden="true">⌄</span></summary><div id="steps"></div></details>`;
    for(const alternative of r.alternatives||[]){
      const button=document.createElement('button');button.className=alternative.policy===r.policy?'selected':'';button.disabled=!alternative.found;
      const actual=alternative.portals?.includes('elevator')?' · 이용 확인 필요':'';
      const labels={shortest:'가장 짧은 길',main_stairs:'메인 계단',sub_stairs:'오른쪽 서브계단',elevator:'엘리베이터'};
      button.innerHTML=`<span>${escapeHTML(labels[alternative.policy]||alternative.label)}</span> <strong>${alternative.found?alternative.distance_m+' m':'연결 없음'}</strong>${actual?' <small>이용 확인 필요</small>':''}`;
      button.addEventListener('click',()=>{$('routePolicy').value=alternative.policy;updatePolicyNote();findRoute();});
      $('routeAlternatives').appendChild(button);
    }
    const liveButton=document.createElement('button');liveButton.id='liveRoute';liveButton.className='route-live-button';liveButton.textContent='◎ 현재 위치에서 안내';
    liveButton.addEventListener('click',async()=>{
      if(walker.isActive())walker.exit();
      location.stop();$('start').value='outside_start';const route=await findRoute();
      if(route?.found)location.startNavigation($('destination').value,{name:placeById[$('destination').value].name,indoorDistance:route.distance_m});
    });
    $('walkRoute').before(liveButton);
    const indoorButton=document.createElement('button');indoorButton.id='indoorLiveRoute';indoorButton.className='route-indoor-button';indoorButton.textContent='건물 안에서 안내 시작';indoorButton.onclick=()=>location.chooseIndoor();$('walkRoute').before(indoorButton);
    location.setGoal($('destination').value,{name:placeById[$('destination').value].name,indoorDistance:r.distance_m});
    const scope=document.createElement('p');scope.id='routeDistanceScope';scope.className='route-note';scope.textContent=location.state.mode==='off'?`${r.start}에서 출발하는 경로`:'입구 도착을 확인한 뒤 이어지는 실내 경로';$('routeResult').prepend(scope);
    $('walkRoute').addEventListener('click',()=>{location.stop();walker.start($('start').value);});
    r.segments.forEach((s,i)=>{
      const label=s.kind==='walk'?s.label.replace('보행 공간','복도'):s.portal_id==='sub_vestibule'?'서브계단 앞 통로':s.label;
      const div=document.createElement('div');div.className='step';div.innerHTML=`<span class="step-index">${i+1}</span><div><strong>${escapeHTML(label)}</strong><small>${s.distance.toFixed(1)}m${s.kind==='elevator'?' · 이용 확인 필요':''}</small><button>주변 사진 보기 ↗</button></div>`;
      div.querySelector('button').addEventListener('click',()=>showNearestPhoto(s.photo_node));$('steps').appendChild(div);
    });
    selectLevel('all');fitPoints(r.route.map(n=>n.position));
    if(state.view==='3d'){await setView('3d');if(location.state.mode==='off')state.three?.showIndoor();}
    state.three?.updateRoute(r.route);if(state.view==='3d'&&location.state.mode==='off')state.three?.focusRoute();updateSceneHUD();setProgress(0);location.setIndoorRoute(r);location.render();return r;
  } catch(error) {if(token===routeRequest){state.route=null;location.setIndoorRoute(null);renderRoute();state.three?.updateRoute([]);$('routeResult').innerHTML='<p class="route-note">경로를 불러오지 못했어요. 출발지·이동 방식 변경에서 다시 계산해 주세요.</p>';toast(error.message||'경로를 계산하지 못했습니다.');}} finally {if(token===routeRequest){$('routeButton').disabled=false;$('journeyPanel').setAttribute('aria-busy','false');}}
}
function showNearestPhoto(nodeId) {
  let chosen=photoById[nodeId];
  const n=state.route?.route.find(p=>p.id===nodeId)||nodeById[nodeId];
  if (!chosen && n) chosen=photos.filter(p=>(p.position[2]>2.1)===(n.level==='2')).sort((a,b)=>Math.hypot(a.position[0]-n.position[0],a.position[1]-n.position[1])-Math.hypot(b.position[0]-n.position[0],b.position[1]-n.position[1]))[0];
  if(chosen)showPhoto(photos.indexOf(chosen));
}
function showPhoto(index) {
  state.photoIndex=(index+photos.length)%photos.length;const p=photos[state.photoIndex];
  $('photoImage').src='/'+p.url;$('photoTitle').textContent=`${{outside:'학생회관 앞',lobby:'1층 로비',floor2:'계단·2층',bridge_out1f:'학생회관 출입구',bridge_12_sub:'오른쪽 서브계단'}[p.session]||'학생회관'}`;
  $('photoCaption').textContent='이 장소 주변에서 실제로 촬영한 사진입니다.';
  if(!$('photoDialog').open)$('photoDialog').showModal();
}
function setProgress(value) {
  state.progress=Number(value);$('routeProgress').value=value;$('replayLayer').replaceChildren();
  const route=state.route?.route;if(!route?.length)return;
  const lengths=[0];for(let i=1;i<route.length;i++)lengths.push(lengths[i-1]+Math.hypot(...route[i].position.map((v,j)=>v-route[i-1].position[j])));
  const target=Number(value)/100*lengths.at(-1);let index=1;while(index<route.length-1&&lengths[index]<target)index++;
  if(route.length===1)index=0;
  const previous=Math.max(0,index-1),fraction=lengths[index]>lengths[previous]?(target-lengths[previous])/(lengths[index]-lengths[previous]):0;
  const p=route[previous].position.map((v,j)=>v+(route[index].position[j]-v)*fraction);
  $('playProgress').textContent=`${Math.round(value)}% · ${route[index].level==='2'?'2층':route[index].level==='stairs'?'층간 이동':'실외·1층'}`;
  element('circle',{cx:p[0],cy:-p[1],r:.65,fill:'#fff',stroke:'#127a5c','stroke-width':.22},$('replayLayer'));
  element('circle',{cx:p[0],cy:-p[1],r:.28,fill:'#127a5c'},$('replayLayer'));
  state.currentNode=route[index].id;state.three?.moveMarker(p);
}
let last=0;
function playback(time) {if(state.playing&&state.route?.found){setProgress(Math.min(100,state.progress+(time-last)/500));if(state.progress>=100){state.playing=false;$('playButton').textContent='▶';}}last=time;requestAnimationFrame(playback);}
requestAnimationFrame(playback);

function updateSceneHUD() {
  const campusView=state.view==='3d'&&state.sceneMode==='campus';
  const walkView=state.view==='3d'&&state.sceneMode==='walk';
  const nearCampus=!state.worldRegion||state.worldRegion==='campus';
  $('mapTitle').textContent=campusView?(nearCampus?'건국대학교 서울캠퍼스':state.worldRegion==='world'?'세계 지도':'주변 지도'):'제1학생회관';
  $('floorDescription').textContent=campusView?(nearCampus?`전역 배경지도 · 교내 ${campus.buildings.length}개 3D 건물`:'지도를 자유롭게 이동해 보세요 · 3D 건물은 교내 제공'):state.level==='all'?'실외 · 1층 · 계단 · 2층':state.level==='outside'?'출입구 주변 실외':`${state.level}층 보행 공간`;
  $('floorControls').hidden=campusView||walkView;$('routePlayer').hidden=!state.route?.found||campusView||walkView||location.state.mode!=='off';
  $('campusHome').classList.toggle('active',campusView&&nearCampus&&!state.selectedBuilding);
  $('indoorHome').classList.toggle('active',!campusView&&state.view==='3d');
  const b=state.selectedBuilding;
  $('buildingCard').hidden=walkView||!b||(!campusView&&!!state.route?.found);
  if(b) {
    const available=b.id==='way/367253036';
    $('buildingName').textContent=available?'제1학생회관':b.n||'캠퍼스 건물';
    $('buildingDetail').textContent=available?'사진으로 둘러보고, 1·2층 목적지까지 찾아보세요.':'이 건물의 실내 길찾기는 아직 준비 중입니다.';
    $('buildingAction').hidden=!available;$('buildingAction').textContent='실내 길찾기 ↗';
  }
}
async function initThree() {
  const {createCampusScene}=await import('./campus-scene.js?v=walking-guide-1');
  state.three=await createCampusScene($('threeCanvas'),{campus,map,navigation,places,
    onEnterBuilding(){location.pauseFollow();state.three?.showIndoor();choosePlace($('destination').value);},onWalkLook(){walker.onLook();},
    onMode(mode,b){state.sceneMode=mode;state.selectedBuilding=b;updateSceneHUD();},
    onMapRegion(region){state.worldRegion=region;updateSceneHUD();},
    onMapInteract(){location.pauseFollow();},
    onBuilding(b){state.selectedBuilding=b;updateSceneHUD();},
    onLoading(message){loadingScreen.model(message);},onError:toast});
  state.three.setLevel(state.level);state.three.updateRoute(state.route?.route||[]);$('modelLoading').hidden=true;
}
async function setView(view) {
  state.view=view;document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
  $('map2d').hidden=view!=='2d';$('map3d').hidden=view!=='3d';
  if(view==='3d'&&!state.three){try{state.threeLoading||=initThree();await state.threeLoading;}catch(error){state.threeLoading=null;loadingScreen.model('');await setView('2d');toast('3D 보기를 시작하지 못해 평면 지도로 전환했습니다.');console.error(error);}}
  updateSceneHUD();
}

for (const p of places) {
  for(const id of ['start','destination']){const option=document.createElement('option');option.value=p.id;option.textContent=`${p.name} · ${p.level==='outside'?'실외':p.level+'층'}`;$(id).appendChild(option);}
}
$('start').value='outside_start';$('destination').value='prime_hall';
updateDestinationPreview();
$('changePlace').addEventListener('click',()=>{showJourney('places');$('search').value='';searchPlaces('');$('search').focus();});
$('destination').addEventListener('change',findRoute);
$('start').addEventListener('change',()=>{location.stop();findRoute();});
$('search').addEventListener('input',e=>{showJourney('places');searchPlaces(e.target.value);});
$('search').addEventListener('focus',()=>bottomSheet.expand());
$('routeButton').addEventListener('click',findRoute);
$('walkStart').addEventListener('click',()=>choosePlace($('destination').value));
function updatePolicyNote(){
  $('policyNote').textContent=routePolicy()==='elevator'?'운행·접근성을 확인해 주세요. 대기시간은 포함하지 않으며 무장애 경로를 보장하지 않습니다.':'실제 개방 여부와 현장 안내를 확인해 주세요.';
}
$('routePolicy').addEventListener('change',()=>{updatePolicyNote();if(walker.isActive())walker.start($('start').value);else if(state.route)findRoute();});
document.querySelectorAll('[data-level]').forEach(b=>b.addEventListener('click',()=>selectLevel(b.dataset.level)));
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
document.querySelectorAll('.dialog-close').forEach(b=>b.addEventListener('click',()=>b.closest('dialog').close()));
$('previousPhoto').addEventListener('click',()=>showPhoto(state.photoIndex-1));$('nextPhoto').addEventListener('click',()=>showPhoto(state.photoIndex+1));
$('routePhoto').addEventListener('click',()=>showNearestPhoto(state.currentNode||state.route?.route[0]?.id));
$('routeProgress').addEventListener('input',e=>setProgress(e.target.value));
$('playButton').addEventListener('click',()=>{if(state.progress>=100)setProgress(0);state.playing=!state.playing;$('playButton').textContent=state.playing?'Ⅱ':'▶';});
function zoom(factor){const b=state.box;const w=Math.max(5,Math.min(400,b[2]*factor)),h=w*b[3]/b[2];state.box=[b[0]+(b[2]-w)/2,b[1]+(b[3]-h)/2,w,h];applyBox();}
$('zoomIn').addEventListener('click',()=>state.view==='3d'?state.three?.zoom(.8):zoom(.8));
$('zoomOut').addEventListener('click',()=>state.view==='3d'?state.three?.zoom(1.25):zoom(1.25));
$('resetView').addEventListener('click',()=>{state.box=[-40,-25,65,60];applyBox();state.three?.reset();});
$('campusHome').addEventListener('click',async()=>{await setView('3d');state.three?.showCampus();});
$('indoorHome').addEventListener('click',async()=>{selectLevel('all');await setView('3d');state.three?.showIndoor();});
$('topView').addEventListener('click',()=>state.three?.topView());
$('buildingAction').addEventListener('click',()=>choosePlace($('destination').value));
$('mapSvg').addEventListener('wheel',e=>{e.preventDefault();zoom(e.deltaY>0?1.08:.92);},{passive:false});
let drag=null;
$('mapSvg').addEventListener('pointerdown',e=>{if(e.target.closest('.place-marker'))return;drag={x:e.clientX,y:e.clientY,box:[...state.box]};$('mapSvg').setPointerCapture(e.pointerId);});
$('mapSvg').addEventListener('pointermove',e=>{if(!drag)return;const bounds=$('mapSvg').getBoundingClientRect();const scale=Math.max(drag.box[2]/bounds.width,drag.box[3]/bounds.height);state.box=[drag.box[0]-(e.clientX-drag.x)*scale,drag.box[1]-(e.clientY-drag.y)*scale,drag.box[2],drag.box[3]];applyBox();});
$('mapSvg').addEventListener('pointerup',()=>drag=null);
document.addEventListener('keydown',e=>{if((e.metaKey||e.ctrlKey)&&e.key==='k'){e.preventDefault();$('search').focus();}});

$('evidenceButton').addEventListener('click',()=>{$('evidenceDialog').showModal();$('evidenceButton').setAttribute('aria-expanded','true');});
$('evidenceDialog').addEventListener('close',()=>$('evidenceButton').setAttribute('aria-expanded','false'));
$('menuSearch').addEventListener('click',()=>{
  $('evidenceDialog').close();if(walker.isActive())walker.exit();
  bottomSheet.expand();
  showJourney('places');$('search').focus();
});
renderMap();applyBox();searchPlaces('');showJourney('places');await setView('3d');

$('menuIndoorStart').onclick=async()=>{$('evidenceDialog').close();if(!state.route?.found)await findRoute();location.chooseIndoor();};
