// Keep preview time separate from the map's render loop and hidden-tab time.
export function createRoutePlayback({state,onProgress,onPlaying}) {
  let last=null;
  function resetClock(){last=null;}
  function setPlaying(playing){
    resetClock();
    state.playing=Boolean(playing&&state.route?.found);
    onPlaying(state.playing);
  }
  function tick(time,hidden=false){
    if(hidden||!state.playing||!state.route?.found){resetClock();return;}
    if(last!==null){
      onProgress(Math.min(100,state.progress+Math.max(0,time-last)/500));
      if(state.progress>=100){setPlaying(false);return;}
    }
    last=time;
  }
  return {setPlaying,tick,resetClock};
}
