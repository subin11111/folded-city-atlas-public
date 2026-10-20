import {routeProgress,distance} from './outdoor-routing.js';
// Distances remain on the registered graph: never replace an unobserved entrance gap by a straight path.
export function navigationEstimate({position,route,entry,indoorDistance=0,accuracy=Infinity,stale=false,rejected=false}){
  const progress=route?.found?routeProgress(position,route.points):null;
  const qualityOk=!stale&&!rejected&&accuracy<=25;
  const nearEntry=distance(position,entry)<Math.max(12,Math.min(accuracy,25));
  const indoor=Math.max(0,Number(indoorDistance)||0),outdoor=progress?.remaining??null;
  return {qualityOk,progress,nearEntry,outdoorRemaining:outdoor,indoorDistance:indoor,
    knownRemaining:outdoor===null?null:outdoor+indoor,
    estimatedMinutes:outdoor===null?null:Math.max(1,Math.ceil((outdoor+indoor)/1.25/60)),
    entranceGap:route?.found?route.endGap:null,
    completeConnection:route?.found&&route.entryConnected===true,
    stage:!qualityOk?'weak_signal':!progress?'no_route':nearEntry?'confirm_entry':progress.remaining<10?'road_end':'outdoor'};
}
