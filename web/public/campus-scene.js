import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {createCampusFrame,insideRing,ringCentre} from './campus-data.js';
import {createDisplayTerrain} from './campus-terrain.js';
import {appearanceFor} from './campus-appearance.js';
import {createWorldMap} from './world-map.js?v=indoor-datum-1';

export async function createCampusScene(container,{campus,map,navigation,places,onMode,onBuilding,onLoading,onError,onEnterBuilding,onWalkLook,onMapRegion,onMapInteract}) {
  const surface=createDisplayTerrain(campus);
  const frame={...createCampusFrame(campus,map),terrainHeight:surface.height};
  const scene=new THREE.Scene();scene.background=new THREE.Color('#e9eee7');
  const renderer=new THREE.WebGLRenderer({antialias:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.0;
  renderer.localClippingEnabled=true;renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);
  renderer.domElement.classList.add('local-scene-canvas');
  const camera=new THREE.PerspectiveCamera(43,1,1,5000);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.09;
  controls.minDistance=4;controls.maxDistance=4200;controls.maxPolarAngle=Math.PI*.475;
  const hemi=new THREE.HemisphereLight(0xf7fbff,0x899c7d,1.2);hemi.up.set(0,0,1);scene.add(hemi);
  const sun=new THREE.DirectionalLight(0xfff5de,2.0);sun.position.set(-300,-450,650);sun.up.set(0,0,1);
  sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);sun.shadow.camera.left=-650;sun.shadow.camera.right=650;
  sun.shadow.camera.top=650;sun.shadow.camera.bottom=-650;sun.shadow.camera.near=1;sun.shadow.camera.far=1800;
  sun.shadow.normalBias=.15;sun.shadow.bias=-.0002;scene.add(sun);
  const campusGroup=new THREE.Group(),indoorGroup=new THREE.Group(),routeGroup=new THREE.Group(),labelLayer=document.createElement('div');
  labelLayer.className='scene-labels';container.appendChild(labelLayer);scene.add(campusGroup,indoorGroup,routeGroup);
  indoorGroup.visible=false;
  const buildingMeshes=[],buildingInfo=new Map(),labels=[],floorModels=new Map();
  let mode='campus',level='all',activeBuilding=null,route=[],animation=null,loadPromise=null,loaded=0,failed=0;
  let currentBounds=null,rendering=true,drag=null;
  let world=null;
  let walkState=null,walkMotion=null,lookDrag=null;
  const upperClip=new THREE.Plane(new THREE.Vector3(0,0,-1),4.3),lowerClip=new THREE.Plane(new THREE.Vector3(0,0,1),-2.4);
  const navPlaceholder=new THREE.Group();indoorGroup.add(navPlaceholder);
  const focusGrid=new THREE.GridHelper(110,22,0xcbd5c7,0xdde5d8);focusGrid.rotation.x=Math.PI/2;focusGrid.position.set(-8,-4,-1.66);indoorGroup.add(focusGrid);

  function mesh(geometry,material,parent=campusGroup) {
    const m=new THREE.Mesh(geometry,material);parent.add(m);return m;
  }
  function shapeFrom(rings) {
    const shape=new THREE.Shape(rings[0].map(p=>new THREE.Vector2(p[0],p[1])));
    for(const ring of rings.slice(1))shape.holes.push(new THREE.Path(ring.map(p=>new THREE.Vector2(p[0],p[1]))));
    return shape;
  }
  function material(color,extra={}) {return new THREE.MeshStandardMaterial({color,roughness:.92,metalness:0,...extra});}
  function addLabel(text,position,kind,select,id) {
    const el=document.createElement(select?'button':'div');el.className=`scene-label ${kind}`;
    el.textContent=text;el.hidden=true;el.dataset.object=id||'';labelLayer.appendChild(el);
    if(select)el.addEventListener('click',e=>{e.stopPropagation();select();});
    labels.push({el,position:new THREE.Vector3(...position),kind,id});return el;
  }
  function makeWindowTexture(style) {
    const canvas=document.createElement('canvas');canvas.width=128;canvas.height=128;
    const c=canvas.getContext('2d');c.fillStyle=style.wall;c.fillRect(0,0,128,128);
    c.fillStyle=style.joint;c.fillRect(0,123,128,5);
    if(style.pattern==='brick-accent'){
      c.fillStyle=style.accent;c.fillRect(0,0,9,128);
      c.strokeStyle=style.joint;c.lineWidth=.6;
      for(let y=0;y<128;y+=7){c.beginPath();c.moveTo(0,y);c.lineTo(9,y);c.stroke();}
    }
    if(style.pattern==='curtain-wall'){
      c.fillStyle=style.glass;c.fillRect(0,10,128,108);
      c.fillStyle=style.wall;for(let x=0;x<128;x+=16)c.fillRect(x,10,2,108);
      c.fillRect(0,0,128,12);c.fillRect(0,116,128,12);
    } else {
    for(let i=0;i<3;i++) {
      const x=12+i*40,w=style.pattern==='stone-fins'?17:26;c.fillStyle=style.glass;c.fillRect(x,24,w,67);
      c.fillStyle=style.wall;c.fillRect(x+w/2,24,1.5,67);
      c.fillStyle=style.joint;c.fillRect(x-2,91,w+4,3);
    }
    }
    const t=new THREE.CanvasTexture(canvas);t.wrapS=t.wrapT=THREE.RepeatWrapping;t.repeat.set(1/9,1/style.floorHeight);t.colorSpace=THREE.SRGBColorSpace;
    t.anisotropy=Math.min(8,renderer.capabilities.getMaxAnisotropy());return t;
  }

  // 공개 고도는 배경 지형에만 사용하며 실내 층간 높이는 원래 스캔 기준을 유지한다.
  const tg=campus.terrain,positions=[],colours=[],indices=[];
  const campusRings=campus.campus;
  for(let j=0;j<tg.nz;j++)for(let i=0;i<tg.nx;i++) {
    const x=tg.x0+i*tg.step,z=tg.z0+j*tg.step,h=surface.heights[j*tg.nx+i],p=frame.toLocal([x,z],h);
    positions.push(...p);
    const inCampus=campusRings.some(r=>insideRing([x,z],r));
    const colour=new THREE.Color(inCampus?'#bfcea9':'#d8dccf');colour.multiplyScalar(.975+(h%3)*.009);
    colours.push(colour.r,colour.g,colour.b);
    if(i<tg.nx-1&&j<tg.nz-1&&campusRings.some(r=>insideRing([x+tg.step/2,z+tg.step/2],r))) {const a=j*tg.nx+i,b=a+1,c=a+tg.nx,d=c+1;indices.push(a,c,b,b,c,d);}
  }
  const terrainGeometry=new THREE.BufferGeometry();terrainGeometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
  terrainGeometry.setAttribute('color',new THREE.Float32BufferAttribute(colours,3));terrainGeometry.setIndex(indices);terrainGeometry.computeVertexNormals();
  const terrain=mesh(terrainGeometry,material('#ffffff',{vertexColors:true,side:THREE.DoubleSide}));terrain.receiveShadow=true;

  const landColours={water:'#5e9ba9',reservoir:'#5e9ba9',park:'#a6bf87',forest:'#8faa74',wood:'#8faa74',grass:'#b3c98e',pitch:'#8ba46f',garden:'#a0ba83'};
  for(const land of campus.lands) {
    if(!land.r?.length)continue;
    const rings=land.r.map(r=>r.map(p=>frame.toLocal(p,frame.terrainHeight(...p))));
    const geometry=new THREE.ShapeGeometry(shapeFrom(rings));const a=geometry.attributes.position;
    const water=['water','reservoir','pond'].includes(land.k);
    const waterZ=(surface.waters.find(w=>w.land===land)?.height??frame.terrainHeight(...ringCentre(land.r[0])))-frame.datum+.14;
    const R=map.georeference.R_map_xy_to_EN,t=map.georeference.t_map_xy_to_EN_m,offset=map.manual_preview_offset_en_m||[2,-2];
    // Sample by position instead of matching rounded Float32 vertex keys.
    for(let i=0;i<a.count;i++){
      const x=a.getX(i),y=a.getY(i),east=R[0][0]*x+R[0][1]*y+t[0]+offset[0],south=-(R[1][0]*x+R[1][1]*y+t[1]+offset[1]);
      a.setZ(i,water?waterZ:frame.terrainHeight(east,south)-frame.datum+.11);
    }
    a.needsUpdate=true;geometry.computeVertexNormals();
    const m=mesh(geometry,material(landColours[land.k]||'#b6c9a6',{side:THREE.DoubleSide,roughness:water?.35:.95,metalness:water?.08:0,
      polygonOffset:true,polygonOffsetFactor:water?-2:-1,polygonOffsetUnits:water?-4:-1}));m.receiveShadow=!water;
  }

  const roadPositions={street:[],walk:[]};
  for(const road of campus.roads) {
    const walk=['footway','path','pedestrian','steps','cycleway'].includes(road.k);
    const width=walk?(road.k==='pedestrian'?3.2:1.8):({tertiary:8,secondary:11,residential:6,service:4.2,living_street:4}[road.k]||5);
    for(let i=1;i<road.p.length;i++) {
      const a=road.p[i-1],b=road.p[i],dx=b[0]-a[0],dz=b[1]-a[1],length=Math.hypot(dx,dz);if(length<.01)continue;
      const n=[-dz/length*width/2,dx/length*width/2];
      // 긴 도로도 지형을 따라가도록 5m 이하의 조각으로 나눈다.
      for(let k=0;k<Math.ceil(length/5);k++) {
        const t0=k/Math.ceil(length/5),t1=(k+1)/Math.ceil(length/5);
        const p=[a[0]+dx*t0,a[1]+dz*t0],q=[a[0]+dx*t1,a[1]+dz*t1];
        if(!campusRings.some(r=>insideRing([(p[0]+q[0])/2,(p[1]+q[1])/2],r)))continue;
        const points=[[p[0]+n[0],p[1]+n[1]],[p[0]-n[0],p[1]-n[1]],[q[0]+n[0],q[1]+n[1]],[q[0]-n[0],q[1]-n[1]]].map(v=>frame.toLocal(v,frame.terrainHeight(...v)+.18));
        for(const index of [0,1,2,2,1,3])roadPositions[walk?'walk':'street'].push(...points[index]);
      }
    }
  }
  for(const [kind,vertices] of Object.entries(roadPositions)) {
    const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));g.computeVertexNormals();
    mesh(g,material(kind==='walk'?'#e8dfc8':'#aab1aa',{side:THREE.DoubleSide})).receiveShadow=true;
  }

  const windowTextures=new Map();
  for(const b of campus.buildings) {
    const style=appearanceFor(b);
    if(!windowTextures.has(style.id))windowTextures.set(style.id,makeWindowTexture(style));
    const isStudent=b.id===frame.student?.id;
    const displayBase=isStudent?b.g:Math.min(b.g,...b.o.map(p=>frame.terrainHeight(...p)));
    const rings=[b.o,...(b.ho||[])].map(r=>r.map(p=>frame.toLocal(p,displayBase)));
    const geometry=new THREE.ExtrudeGeometry(shapeFrom(rings),{depth:b.h,bevelEnabled:false,steps:1,curveSegments:1});
    geometry.translate(0,0,displayBase-frame.datum);
    const roof=material(style.roof);
    const walls=material('#ffffff',{map:windowTextures.get(style.id),roughness:style.pattern==='curtain-wall'?.48:.92});
    const m=mesh(geometry,[roof,walls]);m.castShadow=true;m.receiveShadow=true;m.userData={building:b,appearance:style.id};
    buildingMeshes.push(m);const centre=frame.toLocal(ringCentre(b.o),displayBase+b.h);
    buildingInfo.set(b.id,{source:b,mesh:m,centre:new THREE.Vector3(...centre),bounds:new THREE.Box3().setFromObject(m),baseColours:[roof.color.clone(),walls.color.clone()]});
    if(b.n)addLabel(isStudent?'제1학생회관 · 실내지도':b.n,[centre[0],centre[1],centre[2]+2],isStudent?'available':'building',()=>selectBuilding(b.id),b.id);
  }

  // 참고 데이터의 수목은 장식용이다. 길찾기의 장애물로 사용하지 않는다.
  const trunks=new THREE.InstancedMesh(new THREE.CylinderGeometry(.25,.4,3.5,5),material('#8f8167'),campus.trees.length);
  const crowns=new THREE.InstancedMesh(new THREE.IcosahedronGeometry(2.7,1),material('#5c824e'),campus.trees.length);
  const upperCrowns=new THREE.InstancedMesh(new THREE.IcosahedronGeometry(2.1,0),material('#789b5d'),campus.trees.length);
  const dummy=new THREE.Object3D();
  campus.trees.forEach((p,i)=>{
    const point=frame.toLocal(p,frame.terrainHeight(p[0],p[1])),s=p[2]||1;
    dummy.position.set(point[0],point[1],point[2]+1.75*s);dummy.rotation.set(Math.PI/2,0,0);dummy.scale.setScalar(s);dummy.updateMatrix();trunks.setMatrixAt(i,dummy.matrix);
    dummy.position.z=point[2]+5.3*s;dummy.rotation.set(.2,i*2.399,.4);dummy.scale.set(s,s,1.35*s);dummy.updateMatrix();crowns.setMatrixAt(i,dummy.matrix);
    dummy.position.set(point[0]+.5*s,point[1]-.3*s,point[2]+7.0*s);dummy.scale.set(s,s,s);dummy.updateMatrix();upperCrowns.setMatrixAt(i,dummy.matrix);
  });
  campusGroup.add(trunks,crowns,upperCrowns);crowns.castShadow=true;
  const campusBounds=new THREE.Box3();for(const ring of campus.campus)for(const p of ring)campusBounds.expandByPoint(new THREE.Vector3(...frame.toLocal(p,frame.terrainHeight(...p))));
  campusBounds.max.z=Math.max(campusBounds.max.z,45);campusBounds.min.z=Math.min(campusBounds.min.z,-4);
  const indoorBounds=new THREE.Box3(new THREE.Vector3(-35,-29,-2),new THREE.Vector3(18,20,6));

  // 실내 모델 로딩 전에도 실제 공간 격자로 만든 층별 평면을 보여준다.
  for(const [lv,g] of Object.entries(navigation.levels)) {
    const rings=g.floor_rings.filter(r=>r.length>=3);
    for(const ring of rings) {
      const m=mesh(new THREE.ShapeGeometry(new THREE.Shape(ring.map(p=>new THREE.Vector2(...p)))),material(lv==='1'?'#dce8d4':'#bed8cb',{side:THREE.DoubleSide,transparent:true,opacity:.8}),navPlaceholder);
      m.position.z=g.z;
    }
  }
  for(const p of places)addLabel(p.name,[p.position[0],p.position[1],p.position[2]+.8],'indoor',null,p.id);
  const marker=mesh(new THREE.SphereGeometry(.28,16,10),new THREE.MeshBasicMaterial({color:'#f4c85a',depthTest:false,depthWrite:false,transparent:true}),scene);marker.renderOrder=1001;marker.visible=false;
  // Live position is distinct from the yellow virtual-tour marker.
  const liveGroup=new THREE.Group();scene.add(liveGroup);liveGroup.visible=false;
  let liveData=null,lastLocationFocus=null,lastRoute=null,lastTrackKey='';
  const livePath=new THREE.Group();scene.add(livePath);
  const dot=mesh(new THREE.SphereGeometry(1.15,20,12),new THREE.MeshBasicMaterial({color:'#2789e9',depthTest:false}),liveGroup);dot.renderOrder=1010;
  const rawDot=mesh(new THREE.SphereGeometry(.55,12,8),new THREE.MeshBasicMaterial({color:'#e6a04b',depthTest:false}),liveGroup);rawDot.renderOrder=1009;
  const accuracyRing=mesh(new THREE.RingGeometry(.985,1,80),new THREE.MeshBasicMaterial({color:'#3f91df',side:THREE.DoubleSide,transparent:true,opacity:.6,depthTest:false,depthWrite:false}),liveGroup);
  const accuracyFill=mesh(new THREE.CircleGeometry(1,80),new THREE.MeshBasicMaterial({color:'#439aee',side:THREE.DoubleSide,transparent:true,opacity:.09,depthTest:false,depthWrite:false}),liveGroup);
  const coneGeometry=new THREE.BufferGeometry();coneGeometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,-3,8,0,3,8,0],3));
  const headingCone=mesh(coneGeometry,new THREE.MeshBasicMaterial({color:'#2789e9',side:THREE.DoubleSide,transparent:true,opacity:.4,depthTest:false,depthWrite:false}),liveGroup);
  function groundAt(p){
    const g=map.georeference,R=g.R_map_xy_to_EN,t=g.t_map_xy_to_EN_m,o=map.manual_preview_offset_en_m||[2,-2];
    const east=R[0][0]*p[0]+R[0][1]*p[1]+t[0]+o[0],south=-(R[1][0]*p[0]+R[1][1]*p[1]+t[1]+o[1]);
    return frame.terrainHeight(east,south)-frame.datum+.5;
  }
  function clearLivePath(){while(livePath.children.length){const child=livePath.children[0];child.geometry?.dispose();child.material?.dispose();livePath.remove(child);}}
  function lineLive(points,color,dashed=false,kind='route'){
    if(points.length<2)return;const geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(p[0],p[1],groundAt(p))));
    const material=dashed?new THREE.LineDashedMaterial({color,dashSize:1.2,gapSize:.8,depthTest:false}):new THREE.LineBasicMaterial({color,depthTest:false});
    const line=new THREE.Line(geometry,material);line.renderOrder=1005;line.userData.kind=kind;if(dashed)line.computeLineDistances();livePath.add(line);
  }
  function syncLocationViewport(){
    const r=container.getBoundingClientRect(),card=document.getElementById('locationCard');
    if(mode==='campus'&&liveData&&r.width<=650&&r.height>0&&card&&!card.hidden){
      const overlayBottom=Math.max(0,Math.min(r.height-110,card.getBoundingClientRect().bottom-r.top));
      const targetY=(overlayBottom+r.height-65)/2;
      // Shift the camera's visible centre below the mobile card, never the GNSS coordinates.
      camera.setViewOffset(r.width,r.height,0,r.height/2-targetY,r.width,r.height);
    }else camera.clearViewOffset();
  }
  function updateLocation(data){
    liveData=data;liveGroup.visible=!!data&&mode!=='walk';livePath.visible=!!data&&mode==='campus';
    syncLocationViewport();
    if(!data){lastLocationFocus=null;lastRoute=null;lastTrackKey='';clearLivePath();world?.follow(null);return;}
    const p=data.position,z=groundAt(p)+.4;
    dot.position.set(p[0],p[1],z);dot.material.color.set(data.stale?'#8b9690':'#2789e9');rawDot.position.set(data.raw[0],data.raw[1],groundAt(data.raw)+.3);
    for(const ring of [accuracyRing,accuracyFill]){ring.position.copy(rawDot.position);ring.scale.set(data.accuracy,data.accuracy,1);}
    headingCone.visible=!!data.heading&&!data.stale;if(data.heading){headingCone.position.set(p[0],p[1],z);headingCone.rotation.z=Math.atan2(-data.heading[0],data.heading[1]);}
    const trackKey=data.showTrack?`${data.rawTrack?.length}/${data.track?.length}/${data.rawTrack?.[0]}/${data.rawTrack?.at(-1)}`:'';
    if(data.route!==lastRoute||trackKey!==lastTrackKey){
      lastRoute=data.route;lastTrackKey=trackKey;clearLivePath();
      if(data.route)lineLive(data.route.points,'#168357');
      if(data.showTrack){lineLive(data.rawTrack||[],'#d9974a',true,'raw-recording');lineLive(data.track||[],'#2789e9',false,'filtered-recording');}
    }
    if(mode!=='walk'&&world?.ready){world.follow(data);return;}
    if(data.follow&&mode==='campus'&&(!lastLocationFocus||lastLocationFocus.distanceTo(new THREE.Vector3(...p))>2)){
      animation=null;const target=new THREE.Vector3(p[0],p[1],z),delta=camera.position.clone().sub(controls.target);
      if(!lastLocationFocus||delta.length()>320){delta.set(...frame.vectorFromEN(70,-95,120));}
      controls.target.copy(target);camera.position.copy(target).add(delta);lastLocationFocus=new THREE.Vector3(...p);currentBounds=new THREE.Box3(target.clone().addScalar(-60),target.clone().addScalar(60));
    }
  }

  function fly(bounds,direction,duration=950) {
    currentBounds=bounds.clone();
    if(mode!=='walk'&&world?.ready){animation=null;world.focus(bounds,direction,duration);return;}
    const target=bounds.getCenter(new THREE.Vector3()),radius=bounds.getBoundingSphere(new THREE.Sphere()).radius;
    const verticalFov=THREE.MathUtils.degToRad(camera.fov);
    // A portrait campus is a pannable map, not a tiny model fitted to the narrowest dimension.
    // Growing the visible map height therefore reveals a larger, centred campus view.
    const fov=bounds===campusBounds?verticalFov:Math.min(verticalFov,2*Math.atan(Math.tan(verticalFov/2)*camera.aspect));
    const distance=radius/Math.sin(fov/2)*(bounds===campusBounds ? .79 : 1.04);
    const position=target.clone().add(direction.clone().normalize().multiplyScalar(distance));
    if(!duration){animation=null;camera.position.copy(position);controls.target.copy(target);return;}
    animation={at:performance.now(),duration,fromPosition:camera.position.clone(),fromTarget:controls.target.clone(),position,target};
  }
  function changeMode(next) {
    mode=next;campusGroup.visible=mode==='campus';indoorGroup.visible=mode==='indoor'||mode==='walk';routeGroup.visible=mode==='indoor';marker.visible=mode==='indoor'&&route.length>0;
    liveGroup.visible=!!liveData&&mode==='campus';livePath.visible=!!liveData&&mode==='campus';
    controls.enabled=mode!=='walk'&&!world?.ready;
    world?.setSceneMode(mode);world?.setVisible(mode!=='walk');renderer.domElement.hidden=mode!=='walk'&&!!world?.ready;
    focusGrid.visible=!world?.ready;
    camera.fov=mode==='walk'?72:43;camera.near=mode==='walk'?.05:mode==='campus'?1:.1;camera.far=mode==='walk'?1000:5000;camera.updateProjectionMatrix();
    scene.background.set(mode==='walk'?'#242c34':'#e9eee7');
    setLevel(level);
    labelLayer.classList.toggle('indoor-mode',mode==='indoor');onMode?.(mode,activeBuilding?.source||null);
    syncLocationViewport();
  }
  function setLevel(next) {
    level=next;upperClip.constant=next==='1'?1.25:4.3;
    for(const [name,obj] of floorModels) {
      obj.visible=mode==='walk'||next==='all'||(next==='outside'?['outside','bridge_out1f'].includes(name):next==='2'?['floor2','bridge_12_sub'].includes(name):true);
      obj.traverse(o=>{if(o.isMesh)for(const m of Array.isArray(o.material)?o.material:[o.material])m.clippingPlanes=mode==='walk'?[]:next==='2'?[upperClip,lowerClip]:[upperClip];});
    }
    applyRouteLevel();
  }
  async function loadInterior() {
    if(loadPromise)return loadPromise;
    loadPromise=(async()=>{
      const loader=new GLTFLoader();
      for(const name of ['outside','bridge_out1f','lobby','floor2','bridge_12_sub']) {
        onLoading?.(`실내 스캔 로딩 · ${loaded+failed+1}/5`);
        try {
          const gltf=await loader.loadAsync(`/models/${name}.glb`);
          gltf.scene.traverse(o=>{if(o.isMesh){o.frustumCulled=true;for(const m of Array.isArray(o.material)?o.material:[o.material]){m.clippingPlanes=[upperClip];m.needsUpdate=true;}}});
          indoorGroup.add(gltf.scene);floorModels.set(name,gltf.scene);loaded++;setLevel(level);
          if(loaded>=2)navPlaceholder.visible=false;
        }catch(error){failed++;onError?.(`${name} 실내 모델을 읽지 못했습니다.`);console.error(name,error);}
      }
      onLoading?.('');if(loaded)navPlaceholder.visible=false;
      window.foldedCity3D={models:loaded,failed,campusBuildings:buildingMeshes.length,trees:campus.trees.length};
    })();
    return loadPromise;
  }
  function showCampus(instant=false) {
    lastLocationFocus=null;
    activeBuilding=null;changeMode('campus');onBuilding?.(null);
    if(world?.ready){currentBounds=campusBounds.clone();world.showCampus(instant?0:1100);}
    else fly(campusBounds,new THREE.Vector3(...frame.vectorFromEN(500,-650,670)),instant?0:1100);
  }
  function showIndoor(focus=true) {
    activeBuilding=buildingInfo.get(frame.student.id);changeMode('indoor');onBuilding?.(activeBuilding.source);
    if(focus)fly(indoorBounds,new THREE.Vector3(38,-48,49));
    loadInterior();
  }
  function selectBuilding(id) {
    const info=buildingInfo.get(id);if(!info)return;
    if(id===frame.student.id){if(onEnterBuilding)onEnterBuilding(info.source);else showIndoor();return;}
    activeBuilding=info;changeMode('campus');onBuilding?.(info.source);
    fly(info.bounds.clone().expandByScalar(14),new THREE.Vector3(...frame.vectorFromEN(45,-65,55)));
  }
  function applyRouteLevel() {
    for(const obj of routeGroup.children) obj.visible=level==='all'||level===obj.userData.level||obj.userData.kind==='stairs'||(level==='outside'&&obj.userData.kind==='door');
  }
  function updateRoute(next) {
    route=next;for(const o of [...routeGroup.children]){o.geometry.dispose();o.material.dispose();routeGroup.remove(o);}
    for(let i=1;i<route.length;i++) {
      const a=new THREE.Vector3(...route[i-1].position),b=new THREE.Vector3(...route[i].position);a.z+=.2;b.z+=.2;
      const len=a.distanceTo(b);if(len<.005)continue;
      const kind=route[i].kind,colour=['stairs','door'].includes(kind)?'#d39b40':route[i].level==='2'?'#277f91':'#16b888';
      const m=mesh(new THREE.CylinderGeometry(.18,.18,len,6),new THREE.MeshBasicMaterial({color:colour,depthTest:false,depthWrite:false,transparent:true}),routeGroup);
      m.position.copy(a).add(b).multiplyScalar(.5);m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),b.sub(a).normalize());
      m.renderOrder=1000;m.userData={level:route[i].level,kind};
    }
    if(route.length)marker.position.set(...route[0].position.map((v,i)=>v+(i===2?.3:0)));
    marker.visible=mode==='indoor'&&route.length>0;applyRouteLevel();
  }
  function focus(p) {
    showIndoor(false);const bounds=new THREE.Box3(new THREE.Vector3(p[0]-9,p[1]-9,p[2]-1),new THREE.Vector3(p[0]+9,p[1]+9,p[2]+4));
    fly(bounds,new THREE.Vector3(16,-23,22));
  }
  function topView() {if(mode!=='walk'&&world?.ready)world.topView();else fly(currentBounds||campusBounds,new THREE.Vector3(...frame.vectorFromEN(0,-.001,1)));}
  function walkLook() {
    if(!walkState)return;
    const {yaw,pitch}=walkState;
    controls.target.copy(camera.position).add(new THREE.Vector3(Math.cos(yaw)*Math.cos(pitch),Math.sin(yaw)*Math.cos(pitch),Math.sin(pitch)).multiplyScalar(4));
    camera.lookAt(controls.target);
  }
  function enterWalk(snapshot,{loadModels=false}={}) {
    activeBuilding=buildingInfo.get(frame.student.id);walkState={yaw:snapshot.yaw,pitch:-.03,position:snapshot.position,eye:snapshot.eye_height_m};
    const eye=new THREE.Vector3(snapshot.position[0],snapshot.position[1],snapshot.position[2]+snapshot.eye_height_m);
    const target=eye.clone().add(new THREE.Vector3(Math.cos(snapshot.yaw),Math.sin(snapshot.yaw),-.03).multiplyScalar(4));
    animation={at:performance.now(),duration:1050,fromPosition:camera.position.clone(),fromTarget:controls.target.clone(),position:eye,target};
    changeMode('walk');onBuilding?.(activeBuilding.source);if(loadModels)loadInterior();
  }
  function moveWalk(option,{onProgress}={}) {
    if(!walkState)return Promise.resolve();animation=null;
    const points=option.path.map(p=>new THREE.Vector3(...p.position));const lengths=[0];
    if(points.length<2)return Promise.resolve();
    for(let i=1;i<points.length;i++)lengths.push(lengths.at(-1)+points[i-1].distanceTo(points[i]));
    walkState.yaw=option.heading;
    return new Promise(resolve=>{walkMotion={points,lengths,total:lengths.at(-1),at:performance.now(),duration:Math.min(5500,Math.max(450,lengths.at(-1)*450)),resolve,onProgress};});
  }
  function resize() {
    const r=container.getBoundingClientRect();if(!r.width||!r.height)return;
    renderer.setSize(r.width,r.height);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();
    if(mode!=='walk'&&world?.ready){world.resize();return;}
    if(mode==='campus'&&!activeBuilding)fly(campusBounds,new THREE.Vector3(...frame.vectorFromEN(500,-650,670)),0);
    else if(mode!=='walk'&&currentBounds)fly(currentBounds,camera.position.clone().sub(controls.target),0);
    syncLocationViewport();
  }
  const observer=new ResizeObserver(resize);observer.observe(container);resize();showCampus(true);
  const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();
  renderer.domElement.addEventListener('pointerdown',e=>{
    drag=[e.clientX,e.clientY];
    if(mode==='walk'&&walkState&&animation)camera.position.set(walkState.position[0],walkState.position[1],walkState.position[2]+walkState.eye);
    animation=null;
    if(mode==='walk'&&walkState){lookDrag={x:e.clientX,y:e.clientY,yaw:walkState.yaw,pitch:walkState.pitch};renderer.domElement.setPointerCapture(e.pointerId);}
  });
  renderer.domElement.addEventListener('pointermove',e=>{
    if(!lookDrag||mode!=='walk')return;
    walkState.yaw=lookDrag.yaw+(e.clientX-lookDrag.x)*.004;
    walkState.pitch=THREE.MathUtils.clamp(lookDrag.pitch+(e.clientY-lookDrag.y)*.003,-.9,.9);
  });
  renderer.domElement.addEventListener('pointerup',e=>{
    if(mode==='walk'){if(lookDrag)onWalkLook?.(walkState.yaw);lookDrag=null;return;}
    if(mode!=='campus'||!drag||Math.hypot(e.clientX-drag[0],e.clientY-drag[1])>5)return;
    const r=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1);
    raycaster.setFromCamera(pointer,camera);const hits=raycaster.intersectObjects(buildingMeshes);if(hits.length)selectBuilding(hits[0].object.userData.building.id);
  });
  renderer.domElement.addEventListener('pointercancel',()=>{lookDrag=null;drag=null;});
  controls.addEventListener('start',()=>animation=null);
  let labelTick=0;
  function updateLabels() {
    const r=container.getBoundingClientRect(),used=[];
    const sorted=[...labels].sort((a,b)=>(a.kind==='available'?-1:0)-(b.kind==='available'?-1:0)||a.position.distanceToSquared(camera.position)-b.position.distanceToSquared(camera.position));
    for(const item of sorted) {
      const indoor=item.kind==='indoor';let visible=indoor?mode==='indoor'&&(level==='all'||places.find(p=>p.id===item.id)?.level===level):mode==='campus';
      const labelPosition=item.position.clone();
      if(indoor&&route.length) {
        const p=places.find(p=>p.id===item.id),end=route.at(-1).position;
        visible=visible&&Math.hypot(p.position[0]-end[0],p.position[1]-end[1])<1;
        labelPosition.z+=1;
      }
      if(!indoor&&world?.ready&&world.zoomLevel<(item.kind==='available'?13.5:14.5))visible=false;
      const projected=labelPosition.project(mode!=='walk'&&world?.ready?world.camera:camera);
      visible=visible&&projected.z>-1&&projected.z<1&&Math.abs(projected.x)<.97&&Math.abs(projected.y)<.95;
      const x=(projected.x+1)/2*r.width,y=(-projected.y+1)/2*r.height;
      if(visible&&!indoor&&item.kind!=='available') {
        visible=!used.some(p=>Math.abs(x-p[0])<86&&Math.abs(y-p[1])<26);
        if(activeBuilding&&item.id!==activeBuilding.source.id&&camera.position.distanceTo(item.position)>450)visible=false;
      }
      item.el.hidden=!visible;
      if(visible){item.el.style.transform=`translate(${x}px,${y}px) translate(-50%,-100%)`;used.push([x,y]);}
    }
  }
  function render(now) {
    if(!rendering)return;requestAnimationFrame(render);
    if(animation) {
      const p=Math.min(1,(now-animation.at)/animation.duration),e=1-(1-p)**3;
      camera.position.lerpVectors(animation.fromPosition,animation.position,e);controls.target.lerpVectors(animation.fromTarget,animation.target,e);
      if(p===1)animation=null;
    }
    if(container.closest('[hidden]'))return;
    if(mode==='walk'&&walkState) {
      if(walkMotion) {
        const fraction=Math.min(1,(now-walkMotion.at)/walkMotion.duration),distance=fraction*walkMotion.total;
        let i=1;while(i<walkMotion.lengths.length-1&&walkMotion.lengths[i]<distance)i++;
        const den=walkMotion.lengths[i]-walkMotion.lengths[i-1],t=den>0?(distance-walkMotion.lengths[i-1])/den:0;
        camera.position.lerpVectors(walkMotion.points[i-1],walkMotion.points[i],t);camera.position.z+=walkState.eye;
        walkMotion.onProgress?.(fraction);
        if(fraction===1){const done=walkMotion.resolve;walkState.position=walkMotion.points.at(-1).toArray();walkMotion=null;done();}
      }
      if(animation)camera.lookAt(controls.target);else walkLook();
    } else if(!world?.ready)controls.update();
    if(mode==='walk'||!world?.ready)renderer.render(scene,camera);
    if(++labelTick%3===0)updateLabels();
  }
  try{
    // Campus terrain needs a display lift. Indoor mode instead anchors its
    // first-floor datum to the flat basemap; all floor differences stay intact.
    const lift=frame.datum-Math.min(...surface.heights)+.15;
    world=await createWorldMap(container,{mapData:map,campus,frame,scene,lift,onRegion:onMapRegion,onError,onUserMove:onMapInteract,
      onReady(){
        controls.enabled=false;renderer.domElement.hidden=mode!=='walk';world.setSceneMode(mode);world.setVisible(mode!=='walk');focusGrid.visible=false;
      },
      onPick(point,projectCamera){
        if(mode!=='campus')return;
        const r=container.getBoundingClientRect(),x=point.x/r.width*2-1,y=1-point.y/r.height*2;
        const inverse=projectCamera.projectionMatrixInverse,near=new THREE.Vector3(x,y,-1).applyMatrix4(inverse),far=new THREE.Vector3(x,y,1).applyMatrix4(inverse);
        raycaster.set(near,far.sub(near).normalize());scene.updateMatrixWorld(true);
        const hit=raycaster.intersectObjects(buildingMeshes)[0];if(hit)selectBuilding(hit.object.userData.building.id);
      }
    });
  }catch(error){console.warn('전역지도 초기화 실패',error);onError?.('교내 지도를 표시합니다. 배경지도 연결을 확인해 주세요.');}
  requestAnimationFrame(render);
  window.foldedCity3D={models:0,failed:0,campusBuildings:buildingMeshes.length,trees:campus.trees.length};
  return {setLevel,updateRoute,focus,showCampus,showIndoor,selectBuilding,topView,enterWalk,moveWalk,loadWalkModels:loadInterior,updateLocation,
    setWalkYaw(yaw){if(walkState)walkState.yaw=yaw;},
    turnWalk(delta){if(walkState){if(animation)camera.position.set(walkState.position[0],walkState.position[1],walkState.position[2]+walkState.eye);animation=null;walkState.yaw+=delta;onWalkLook?.(walkState.yaw);}},
    getWalkYaw(){return walkState?.yaw||0;},
    focusRoute(){if(route.length){const bounds=new THREE.Box3().setFromPoints(route.map(p=>new THREE.Vector3(...p.position))).expandByScalar(6);fly(bounds,new THREE.Vector3(38,-48,49));}},
    moveMarker(p){marker.position.set(p[0],p[1],p[2]+.3);marker.visible=mode==='indoor';},
    zoom(factor){animation=null;if(mode!=='walk'&&world?.ready)world.zoom(factor);else camera.position.sub(controls.target).multiplyScalar(factor).add(controls.target);},
    reset(){mode==='indoor'?showIndoor():showCampus();},
    // Read-only rendering evidence and deterministic viewpoints for screenshot checks.
    campusView(target,offsetEN){changeMode('campus');animation=null;controls.target.set(...target);camera.position.copy(controls.target).add(new THREE.Vector3(...frame.vectorFromEN(...offsetEN)));controls.update();},
    worldView(options){changeMode('campus');world?.jumpTo(options);},
    stats(){return {mode,loaded,failed,buildings:buildingMeshes.length,registrationScale:1,terrain:surface.diagnostics,
      worldMap:world?.stats()||null,
      locationTracks:livePath.children.map(line=>({kind:line.userData.kind,points:line.geometry.attributes.position.count})),
      appearances:buildingMeshes.map(m=>({id:m.userData.building.id,style:m.userData.appearance}))};}};
}
