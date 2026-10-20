// WGS84 ECEF -> ENU -> registered SLAM XY, consistent with campus-data.js.
const rad=Math.PI/180;
function ecef(lat,lon){lat*=rad;lon*=rad;const a=6378137,e2=6.6943799901413165e-3,N=a/Math.sqrt(1-e2*Math.sin(lat)**2);return [N*Math.cos(lat)*Math.cos(lon),N*Math.cos(lat)*Math.sin(lon),N*(1-e2)*Math.sin(lat)];}
export function createGeoFrame(map){
  const g=map.georeference,lat=g.origin_latitude_deg*rad,lon=g.origin_longitude_deg*rad,o=ecef(g.origin_latitude_deg,g.origin_longitude_deg),R=g.R_map_xy_to_EN,t=g.t_map_xy_to_EN_m,offset=map.manual_preview_offset_en_m||[2,-2];
  const vector=(east,north)=>[R[0][0]*east+R[1][0]*north,R[0][1]*east+R[1][1]*north];
  return {vector,
    fromGeo(latitude,longitude){const d=ecef(latitude,longitude).map((v,i)=>v-o[i]);return vector(-Math.sin(lon)*d[0]+Math.cos(lon)*d[1]-t[0]-offset[0],-Math.sin(lat)*Math.cos(lon)*d[0]-Math.sin(lat)*Math.sin(lon)*d[1]+Math.cos(lat)*d[2]-t[1]-offset[1]);},
    // Metre-scale inverse for log export; ECEF projection is used for all incoming fixes.
    toGeo(x,y){const east=R[0][0]*x+R[0][1]*y+t[0]+offset[0],north=R[1][0]*x+R[1][1]*y+t[1]+offset[1];return {latitude:g.origin_latitude_deg+north/111000,longitude:g.origin_longitude_deg+east/(111320*Math.cos(lat))};},
    headingVector(deg){return vector(Math.sin(deg*rad),Math.cos(deg*rad));},
    registrationAccuracy:g.reported_accuracy_median_m
  };
}
