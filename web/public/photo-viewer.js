const $=id=>document.getElementById(id);

export function createPhotoViewer({onTurn,on3D,onNearest}) {
  const stage=$('photoStage'),images=[$('photoViewA'),$('photoViewB')],cache=new Map();
  let active=0,identity=null,serial=0,drag=null,enabled=false,corrected=true,presentedPhoto=null;
  for(const image of images){image.draggable=false;image.setAttribute('aria-hidden','true');}
  function preload(url){
    if(!cache.has(url)){
      const image=new Image();image.src=url;
      const promise=image.decode().catch(error=>{cache.delete(url);throw error;});cache.set(url,promise);
      if(cache.size>64)cache.delete(cache.keys().next().value);
    }
    return cache.get(url);
  }
  function unavailable(message){
    identity=null;stage.dataset.loading='false';$('photoUnavailable').hidden=false;
    for(const image of images){image.classList.remove('visible');image.setAttribute('aria-hidden','true');}
    $('photoBackdrop').style.backgroundImage='none';$('photoSourceInfo').textContent=message||'이 방향의 사진 없음';
    stage.removeAttribute('data-photo-id');
  }
  async function display(photo){
    if(!enabled)return;
    presentedPhoto=photo;
    if(!photo){serial++;unavailable();return;}
    const url=corrected&&photo.display_url?photo.display_url:photo.url;
    const variant=corrected&&photo.display_url?'v3 암부 보정':'원본';
    const caption=`실제 RGB · ${photo.session} #${photo.node} · ${variant} · 위치 ${photo.distance_m.toFixed(1)}m / 방향 ${photo.heading_delta_deg}° 차이 · 360° 아님`;
    $('photoSourceInfo').textContent='주변에서 촬영한 사진';
    $('photoBrightness').textContent=corrected?'밝게 보기 켬':'밝게 보기 끔';
    $('photoBrightness').disabled=!photo.display_url;
    if(identity===photo.id+'|'+url)return;
    identity=photo.id+'|'+url;const token=++serial;stage.dataset.loading='true';
    try{
      await preload(url);if(token!==serial||!enabled)return;
      const next=1-active;images[next].src=url;
      images[next].alt='현재 둘러보는 지점 주변의 실제 촬영 사진';
      await images[next].decode();if(token!==serial||!enabled)return;
      images[next].classList.add('visible');images[next].setAttribute('aria-hidden','false');
      images[active].classList.remove('visible');images[active].setAttribute('aria-hidden','true');active=next;
      $('photoBackdrop').style.backgroundImage=`url("${url}")`;
      $('photoUnavailable').hidden=true;stage.dataset.loading='false';stage.dataset.photoId=photo.id;
      stage.dataset.sourceVariant=variant;
    }catch(error){if(token===serial)unavailable('원본 사진을 읽지 못했습니다. 다른 방향 또는 3D 보기를 선택해 주세요.');}
  }
  $('photoFit').addEventListener('click',()=>{
    const fill=stage.classList.toggle('fill');$('photoFit').textContent=fill?'원본 비율 보기':'화면 채우기';
    $('photoFit').setAttribute('aria-pressed',String(fill));
  });
  $('photoBrightness').addEventListener('click',()=>{
    corrected=!corrected;$('photoBrightness').setAttribute('aria-pressed',String(corrected));display(presentedPhoto);
  });
  $('photoFallback3D').addEventListener('click',on3D);$('photoNearestDirection').addEventListener('click',onNearest);
  stage.addEventListener('pointerdown',e=>{
    if(e.target.closest('button,a'))return;drag={x:e.clientX};stage.setPointerCapture(e.pointerId);
  });
  stage.addEventListener('pointermove',e=>{if(!drag)return;const delta=e.clientX-drag.x;drag.x=e.clientX;if(delta)onTurn(delta*.004);});
  stage.addEventListener('pointerup',()=>drag=null);stage.addEventListener('pointercancel',()=>drag=null);
  return {display,preload,
    show(value){enabled=value;stage.hidden=!value;if(!value){serial++;identity=null;}},
    clear(){serial++;unavailable();},
    preloadSequence(samples){for(const frame of samples)if(frame.photo)preload(corrected&&frame.photo.display_url?frame.photo.display_url:frame.photo.url).catch(()=>{});}
  };
}
