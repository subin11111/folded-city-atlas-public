import * as THREE from 'three';
import {createWorldRegistration,displayElevationOffset} from './world-registration.js?v=indoor-datum-1';
import {orbitCamera,followChanged} from './map-camera.js';

// Background coverage is global; custom building/terrain geometry remains campus-only.
export async function createWorldMap(container,{mapData,campus,frame,scene,lift,onReady,onRegion,onPick,onError,onUserMove}){
  const {Map:LibreMap,setWorkerUrl,setWorkerCount}=await import('/vendor/maplibre/maplibre-gl.mjs');
  setWorkerUrl('/vendor/maplibre/maplibre-gl-worker.mjs');setWorkerCount(2);
  const registration=createWorldRegistration(mapData,{lift});
  const element=document.createElement('div');element.className='world-map';element.hidden=true;
  element.setAttribute('aria-label','전역 배경지도와 건국대학교 교내 3차원 건물');container.prepend(element);
  const notice=document.createElement('div');notice.className='world-map-notice';notice.hidden=true;
  notice.textContent='배경지도를 불러오지 못했어요. 인터넷 연결을 확인해 주세요.';container.append(notice);
  // OpenFreeMap serves OSM-derived vector tiles without a client API key.
  const libre=new LibreMap({container:element,style:'/data/world_style.json',center:[campus.meta.lon,campus.meta.lat],
    zoom:16.1,pitch:50,bearing:-32,minZoom:1,maxZoom:21,maxPitch:70,renderWorldCopies:true,
    attributionControl:false,canvasContextAttributes:{antialias:true},refreshExpiredTiles:true});
  const camera=new THREE.Camera(),model=new THREE.Matrix4().fromArray(registration.matrix);
  let sceneMode='campus',displayZOffset=lift;
  function setSceneMode(next){
    sceneMode=next;displayZOffset=displayElevationOffset(mapData,{mode:next,campusLift:lift});
    model.elements[14]=displayZOffset*registration.scale;
    element.dataset.displayZOffset=displayZOffset.toFixed(4);
    if(ready)libre.triggerRepaint();
  }
  let ready=false,visible=true,draws=0,errors=0,lastRegion='',lastFollow=null,autoFrameCampus=true;
  let interaction=matchMedia('(pointer:fine)').matches?'rotate':'pan',orbit=null;
  const tools=document.createElement('div');tools.className='world-map-tools';tools.setAttribute('aria-label','지도 조작');
  tools.innerHTML='<button type="button" data-map-tool="pan" aria-label="드래그로 지도 이동">이동</button><button type="button" data-map-tool="rotate" aria-label="드래그로 지도 회전">회전</button><button type="button" data-map-tool="north" aria-label="북쪽 맞추기"><span>↑</span> N</button>';
  container.append(tools);
  function setInteraction(value){
    interaction=value;
    for(const b of tools.querySelectorAll('[data-map-tool]'))if(b.dataset.mapTool!=='north')b.setAttribute('aria-pressed',String(b.dataset.mapTool===value));
    element.dataset.interaction=value;tools.title=value==='rotate'?'마우스로 드래그해 회전 · 휠로 확대 · 터치는 두 손가락으로 회전':'드래그해 이동 · 오른쪽 드래그 또는 두 손가락으로 회전';
  }
  function userMove(){lastFollow=null;autoFrameCampus=false;onUserMove?.();}
  for(const b of tools.querySelectorAll('button'))b.addEventListener('click',()=>{
    userMove();if(b.dataset.mapTool==='north')libre.easeTo({bearing:0,duration:400});else setInteraction(b.dataset.mapTool);
  });
  setInteraction(interaction);
  const canvas=libre.getCanvas();
  canvas.addEventListener('pointerdown',event=>{
    if(!ready||!visible||interaction!=='rotate'||event.pointerType!=='mouse'||event.button!==0||event.ctrlKey||event.shiftKey)return;
    libre.stop();orbit={id:event.pointerId,x:event.clientX,y:event.clientY,bearing:libre.getBearing(),pitch:libre.getPitch(),moved:false};
    canvas.setPointerCapture(event.pointerId);event.preventDefault();event.stopPropagation();
  },true);
  canvas.addEventListener('pointermove',event=>{
    if(!orbit||orbit.id!==event.pointerId)return;
    const dx=event.clientX-orbit.x,dy=event.clientY-orbit.y;
    if(Math.hypot(dx,dy)<4&&!orbit.moved)return;
    if(!orbit.moved){orbit.moved=true;userMove();}
    libre.jumpTo(orbitCamera(orbit,dx,dy));event.preventDefault();event.stopPropagation();
  },true);
  function endOrbit(event){
    if(!orbit||orbit.id!==event.pointerId)return;
    const previous=orbit;orbit=null;if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);
    if(!previous.moved&&event.type==='pointerup'&&libre.getZoom()>=14){const r=canvas.getBoundingClientRect();onPick?.({x:event.clientX-r.left,y:event.clientY-r.top},camera);}
    event.preventDefault();event.stopPropagation();
  }
  canvas.addEventListener('pointerup',endOrbit,true);canvas.addEventListener('pointercancel',endOrbit,true);
  const bounds=campus.campus.flat().map(p=>registration.toLngLat(frame.toLocal(p)));
  const west=Math.min(...bounds.map(p=>p[0])),east=Math.max(...bounds.map(p=>p[0]));
  const south=Math.min(...bounds.map(p=>p[1])),north=Math.max(...bounds.map(p=>p[1]));
  function region(){
    const c=libre.getCenter(),near=c.lng>=west-.006&&c.lng<=east+.006&&c.lat>=south-.006&&c.lat<=north+.006;
    const next=near&&libre.getZoom()>=13?'campus':libre.getZoom()<7?'world':'surroundings';
    if(next!==lastRegion){lastRegion=next;onRegion?.(next);}
  }
  function setVisible(next){visible=next;element.hidden=!next||!ready;tools.hidden=!next||!ready;notice.hidden=true;if(next&&ready){libre.resize();libre.triggerRepaint();}}
  function showCampus(duration=850){
    lastFollow=null;autoFrameCampus=true;
    libre.fitBounds([[west,south],[east,north]],{padding:{top:120,bottom:55,left:25,right:25},pitch:50,bearing:-32,duration});
  }
  libre.on('error',event=>{
    errors++;if(visible)notice.hidden=false;
    if(!ready)onError?.('전역 배경지도 연결을 기다리고 있어요. 교내 지도는 계속 사용할 수 있습니다.');
    console.warn('배경지도 요청 실패',event.error?.message||'network error');
  });
  libre.on('sourcedata',event=>{if(event.isSourceLoaded)notice.hidden=true;});
  libre.on('move',()=>{region();element.dataset.bearing=libre.getBearing().toFixed(2);element.dataset.pitch=libre.getPitch().toFixed(2);tools.querySelector('[data-map-tool="north"] span').style.transform=`rotate(${-libre.getBearing()}deg)`;});
  for(const event of ['dragstart','zoomstart','rotatestart','pitchstart'])libre.on(event,e=>{if(e.originalEvent)userMove();});
  libre.on('click',event=>{if(ready&&libre.getZoom()>=14)onPick?.(event.point,camera);});
  libre.on('style.load',()=>{
    // Prevent a remote style from enabling extruded buildings outside our own coverage.
    for(const layer of libre.getStyle().layers||[])if(layer.type==='fill-extrusion')libre.removeLayer(layer.id);
    libre.setProjection({type:'mercator'});
    libre.addLayer({id:'folded-city-campus',type:'custom',renderingMode:'3d',
      onAdd(_map,gl){
        this.renderer=new THREE.WebGLRenderer({canvas:libre.getCanvas(),context:gl,antialias:true});
        this.renderer.autoClear=false;this.renderer.outputColorSpace=THREE.SRGBColorSpace;
        this.renderer.localClippingEnabled=true;
        this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1;
        this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFSoftShadowMap;
      },
      render(_gl,args){
        if(!visible||libre.getZoom()<13.5)return;
        camera.projectionMatrix.fromArray(args.defaultProjectionData.mainMatrix).multiply(model);
        camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
        const background=scene.background;scene.background=null;
        this.renderer.resetState();
        // MapLibre owns canvas sizing. Match its current physical-pixel viewport after every resize.
        const canvas=libre.getCanvas();this.renderer.setViewport(0,0,canvas.width,canvas.height);
        this.renderer.render(scene,camera);scene.background=background;draws++;
      },
      onRemove(){this.renderer?.dispose();}
    });
    ready=true;setVisible(visible);showCampus(0);region();onReady?.();
  });
  return {
    camera,libre,registration,get ready(){return ready;},get zoomLevel(){return libre.getZoom();},
    setVisible,showCampus,setSceneMode,
    resize(){if(visible&&ready){libre.resize();if(autoFrameCampus)showCampus(0);}},repaint(){if(visible&&ready)libre.triggerRepaint();},
    jumpTo(options){autoFrameCampus=false;libre.jumpTo(options);},
    zoom(factor){autoFrameCampus=false;libre.zoomTo(Math.max(1,Math.min(21,libre.getZoom()-Math.log2(factor))),{duration:200});},
    topView(){autoFrameCampus=false;libre.easeTo({pitch:0,bearing:0,duration:550});},
    focus(bounds,direction,duration=850){
      autoFrameCampus=false;
      const center=bounds.getCenter(new THREE.Vector3()),radius=Math.max(8,bounds.getBoundingSphere(new THREE.Sphere()).radius);
      const position=registration.toLngLat(center.toArray()),r=element.getBoundingClientRect();
      const metresPerPixel=radius*3/Math.max(180,Math.min(r.width,r.height));
      const zoom=Math.max(1,Math.min(20.5,Math.log2(78271.51696*Math.cos(position[1]*Math.PI/180)/metresPerPixel)));
      const R=mapData.georeference.R_map_xy_to_EN,E=R[0][0]*direction.x+R[0][1]*direction.y,N=R[1][0]*direction.x+R[1][1]*direction.y;
      libre.easeTo({center:position,zoom,pitch:50,bearing:Math.atan2(-E,-N)*180/Math.PI,duration});
    },
    follow(data){
      if(!data?.follow||!visible||!ready){lastFollow=null;libre.triggerRepaint();return;}
      const p=data.position;if(!followChanged(lastFollow,data)){libre.triggerRepaint();return;}
      autoFrameCampus=false;
      const card=document.getElementById('locationCard'),r=element.getBoundingClientRect();
      const top=r.width<=650&&card&&!card.hidden?Math.min(r.height*.48,Math.max(20,card.getBoundingClientRect().bottom-r.top+10)):90;
      libre.easeTo({center:registration.toLngLat(p),zoom:Math.max(data.indoor?20:18,libre.getZoom()),pitch:50,
        ...(data.headingUp&&Number.isFinite(data.headingDegrees)?{bearing:data.headingDegrees}:{}),
        padding:{top,bottom:45,left:20,right:20},duration:lastFollow?350:0});lastFollow={position:[...p],headingDegrees:data.headingDegrees,headingUp:data.headingUp};
    },
    stats(){return {ready,coverage:'global',provider:'OpenFreeMap',zoom:libre.getZoom(),center:libre.getCenter().toArray(),
      minZoom:libre.getMinZoom(),maxBounds:libre.getMaxBounds()||null,campusDraws:draws,errors,
      sceneMode,displayZOffset,verticalDatum:'display only; first floor aligned to flat basemap in indoor mode',custom3DCoverage:'konkuk-campus',outsideBuildingExtrusions:0};}
  };
}
