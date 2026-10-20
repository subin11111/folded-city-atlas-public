export const MOBILE_LAYOUT='(max-width:650px), (max-height:500px) and (pointer:coarse)';

// Move the real controls, not copies, so permissions and keyboard focus keep working.
export function dockMobileControl(element){
  const destination=document.getElementById('menuMapControls');
  if(!destination||!element)return ()=>{};
  const home=element.parentNode,next=element.nextSibling,media=matchMedia(MOBILE_LAYOUT);
  function place(){
    if(media.matches)destination.append(element);
    else if(next?.parentNode===home)home.insertBefore(element,next);
    else home.append(element);
  }
  media.addEventListener('change',place);place();
  return ()=>media.removeEventListener('change',place);
}
