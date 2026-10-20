import {createGeoFrame} from './geo-frame.js';
const $=id=>document.getElementById(id),read=async p=>{const r=await fetch(p);if(!r.ok)throw Error(`자료를 읽지 못했습니다 (${r.status})`);return r.json();};
try{
const [audit,nav,map,campus]=await Promise.all(['/data/gnss_slam_comparison.json','/data/navigation.json','/data/map.json','/data/campus_source.json'].map(read));
const frame=createGeoFrame(map),g=map.georeference,R=g.R_map_xy_to_EN,t=g.t_map_xy_to_EN_m,offset=audit.manual_preview_offset_en_m;
const colours={reference:'#26372f',raw:'#c36b3d',kalman:'#2876b5',rts:'#6f4f94'};
let session=new URLSearchParams(location.search).get('session')||'lobby',index=0,timer=null;
if(!audit.sessions[session])session='lobby';
for(const [id,s] of Object.entries(audit.sessions)){const o=document.createElement('option');o.value=id;o.textContent=s.label;$('session').append(o);}$('session').value=session;
function project(p){return $('frame').value==='enu'?[R[0][0]*p[0]+R[0][1]*p[1]+t[0]+offset[0],R[1][0]*p[0]+R[1][1]*p[1]+t[1]+offset[1]]:p.slice(0,2);}
function draw(){
 const s=audit.sessions[session],rows=s.records,r=rows[index];$('title').textContent=s.label;$('time').max=rows.length-1;$('time').value=index;
 $('timeLabel').textContent=`${index+1}/${rows.length} · ${(r.gnss_stamp-s.reference_trajectory[0].t).toFixed(1)}초`;
 const svg=$('plot');svg.replaceChildren();
 const points=[...s.reference_trajectory.map(r=>r.position),...rows.map(r=>r.raw)].map(project);
 for(const layer of Object.values(nav.levels))for(const ring of layer.floor_rings)points.push(...ring.map(project));
 const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),bounds=[Math.min(...xs)-5,Math.min(...ys)-5,Math.max(...xs)+5,Math.max(...ys)+5];
 const scale=Math.min(810/(bounds[2]-bounds[0]),560/(bounds[3]-bounds[1])),cx=(bounds[0]+bounds[2])/2,cy=(bounds[1]+bounds[3])/2;
 const pos=p=>{const q=project(p);return [450+(q[0]-cx)*scale,320-(q[1]-cy)*scale];};
 function shape(tag,attrs){const el=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const [k,v] of Object.entries(attrs))el.setAttribute(k,v);svg.append(el);return el;}
 function path(ps,attrs){return shape('path',{d:ps.map((p,i)=>`${i?'L':'M'}${pos(p).join(',')}`).join(' '),fill:'none',...attrs});}
 if($('frame').value==='enu')for(const road of campus.roads){const ps=road.p.map(p=>{const e=p[0]-t[0]-offset[0],n=-p[1]-t[1]-offset[1];return [R[0][0]*e+R[1][0]*n,R[0][1]*e+R[1][1]*n];});path(ps,{stroke:'#e1e7df','stroke-width':3});}
 for(const [id,layer] of Object.entries(nav.levels)){
  if(session==='floor2'&&id==='1'||session==='lobby'&&id==='2')continue;
  for(const ring of layer.floor_rings)path([...ring,ring[0]],{fill:'#eef1ec',stroke:'#b3bcb1','stroke-width':.6});
  for(const ring of layer.obstacle_rings)path([...ring,ring[0]],{fill:'#d0d5cf',stroke:'none',opacity:.5});
 }
 path(s.reference_trajectory.map(r=>r.position),{stroke:colours.reference,'stroke-width':1.5,opacity:.5});
 for(const key of ['raw','kalman','rts']){
  const trace=rows.slice(0,index+1).filter(r=>r[key]).map(r=>r[key]);if(trace.length)path(trace,{stroke:colours[key],'stroke-width':1.7,opacity:.8});
 }
 if(r.reference)path([r.reference,r.raw],{stroke:'#b68b72','stroke-width':1.5,'stroke-dasharray':'5 5'});
 for(const key of ['reference','raw','kalman','rts'])if(r[key]){const [x,y]=pos(r[key]);shape('circle',{cx:x,cy:y,r:key==='reference'?6:4,fill:colours[key],stroke:'white','stroke-width':1.5});}
 const x=60,y=598,len=10*scale;shape('path',{d:`M${x},${y-5}V${y}H${x+len}V${y-5}`,stroke:'#657768',fill:'none'});shape('text',{x,y:y+20,fill:'#657768','font-size':13}).textContent='10m';
 shape('text',{x:30,y:28,fill:'#657768','font-size':14}).textContent=$('frame').value==='enu'?'동쪽 E →  /  북쪽 N ↑':'지도 X →  /  지도 Y ↑';
 const geo=r.reference?frame.toGeo(...r.reference):null,diff=p=>p&&r.reference?`${Math.hypot(p[0]-r.reference[0],p[1]-r.reference[1]).toFixed(2)}m`:'대응 없음';
 const values=[['SLAM 대비 원본 차이',diff(r.raw)],['칼만 / RTS 차이',`${diff(r.kalman)} / ${diff(r.rts)}`],['원본 위도 / 경도',`${r.latitude.toFixed(7)} / ${r.longitude.toFixed(7)}`],['SLAM 대응 위도 / 경도',geo?`${geo.latitude.toFixed(7)} / ${geo.longitude.toFixed(7)}`:'보간 범위 밖'],['원본 / SLAM 지도 XY',`${r.raw.map(v=>v.toFixed(2)).join(', ')} / ${r.reference?r.reference.slice(0,2).map(v=>v.toFixed(2)).join(', '):'없음'}`]];
 $('values').replaceChildren();for(const [key,value] of values){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=key;dd.textContent=value;$('values').append(dt,dd);}
 $('quality').textContent=`기기 보고 반경 ${r.reported_accuracy.toFixed(1)}m · 저장 당시 측정 나이 ${r.age_s.toFixed(2)}초 · ${r.accepted?'필터 수락':'필터 거부: '+r.reason}`;
 $('metrics').innerHTML=['raw_common_residual_m','kalman_residual_m','rts_residual_m'].map(key=>`<td>${s.metrics[key].rmse.toFixed(2)}m</td>`).join('');
 $('transform').textContent=`origin = ${g.origin_latitude_deg.toFixed(10)}, ${g.origin_longitude_deg.toFixed(10)}\nENU = R × [map X, map Y] + t + 화면 이동\nR = [${R[0].map(v=>v.toFixed(7)).join(', ')}; ${R[1].map(v=>v.toFixed(7)).join(', ')}]\nt = [${t.map(v=>v.toFixed(4)).join(', ')}] m\n화면 이동 = [${offset.join(', ')}] m     축척 = 1\n다른 구현 간 좌표 변환 차이 ≤ ${audit.coordinate_checks.python_vs_production_from_geo_max_m.toExponential(2)} m`;
}
function pause(){clearInterval(timer);timer=null;$('play').textContent='재생';}
function select(id){if(!audit.sessions[id])throw Error('Unknown session');pause();session=id;index=0;$('session').value=id;draw();}
$('session').onchange=e=>select(e.target.value);$('frame').onchange=draw;$('time').oninput=e=>{pause();index=+e.target.value;draw();};
$('play').onclick=()=>{if(timer){pause();return;}if(index>=audit.sessions[session].records.length-1)index=0;$('play').textContent='일시 정지';timer=setInterval(()=>{index++;if(index>=audit.sessions[session].records.length){index--;pause();}draw();},140);};
window.gnssAudit={data:audit,select,seek(i){pause();index=Math.max(0,Math.min(audit.sessions[session].records.length-1,Math.round(i)));draw();},get session(){return session;},get index(){return index;}};draw();
}catch(error){document.querySelector('main').textContent=error.message;console.error(error);}
