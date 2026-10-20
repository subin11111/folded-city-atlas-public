// 캠퍼스 데이터의 동/남 미터 좌표를 기존 SLAM 지도 좌표로 변환한다.
// 축척은 변경하지 않는다. 지리 정합은 기존 GNSS 임시 배치를 그대로 따른다.
const campusLabels={
  'relation/16467729':{
    n:'수의학관 · 수의과대학',
    aliases:['수의학관','수의과대학','수의대','수의관','15번 건물'],
    name_source:'https://petindustry.konkuk.ac.kr/location.do?menuSeq=13375&siteId=COMPANION'
  }
};

// Keep the downloaded OSM source and its provenance hash unchanged.
export function applyCampusLabels(campus){
  return {...campus,buildings:campus.buildings.map(b=>campusLabels[b.id]?{...b,...campusLabels[b.id]}:b)};
}

export function campusBuildingMatches(building,query){
  const normalise=text=>String(text).toLowerCase().replace(/\s+/g,'');
  const q=normalise(query);
  return !!q&&[building.n,...(building.aliases||[])].filter(Boolean).some(name=>normalise(name).includes(q));
}

export function createCampusFrame(campus, map) {
  const rad=Math.PI/180, phi=campus.meta.lat*rad;
  const mlat=111132.92-559.82*Math.cos(2*phi)+1.175*Math.cos(4*phi);
  const mlon=111412.84*Math.cos(phi)-93.5*Math.cos(3*phi);
  const geo=map.georeference, lat0=geo.origin_latitude_deg*rad,lon0=geo.origin_longitude_deg*rad;
  const R=geo.R_map_xy_to_EN,t=geo.t_map_xy_to_EN_m;
  const offset=map.manual_preview_offset_en_m||[2,-2];
  function ecef(lat,lon) {
    const a=6378137,e2=6.6943799901413165e-3,n=a/Math.sqrt(1-e2*Math.sin(lat)**2);
    return [n*Math.cos(lat)*Math.cos(lon),n*Math.cos(lat)*Math.sin(lon),n*(1-e2)*Math.sin(lat)];
  }
  const origin=ecef(lat0,lon0);
  const student=campus.buildings.find(b=>b.id==='way/367253036');
  const datum=(student?.g||21.5)+1.55;
  function toLocal(point,height=datum) {
    if(campus.meta.projection==='local_ENU_east_south') {
      const east=point[0]-t[0]-offset[0],north=-point[1]-t[1]-offset[1];
      return [R[0][0]*east+R[1][0]*north,R[0][1]*east+R[1][1]*north,height-datum];
    }
    const lat=(campus.meta.lat-point[1]/mlat)*rad,lon=(campus.meta.lon+point[0]/mlon)*rad;
    const d=ecef(lat,lon).map((v,i)=>v-origin[i]);
    const east=-Math.sin(lon0)*d[0]+Math.cos(lon0)*d[1]-t[0]-offset[0];
    const north=-Math.sin(lat0)*Math.cos(lon0)*d[0]-Math.sin(lat0)*Math.sin(lon0)*d[1]+Math.cos(lat0)*d[2]-t[1]-offset[1];
    return [R[0][0]*east+R[1][0]*north,R[0][1]*east+R[1][1]*north,height-datum];
  }
  function terrainHeight(x,z) {
    const g=campus.terrain;
    const u=Math.max(0,Math.min(g.nx-1.000001,(x-g.x0)/g.step));
    const v=Math.max(0,Math.min(g.nz-1.000001,(z-g.z0)/g.step));
    const i=Math.floor(u),j=Math.floor(v),du=u-i,dv=v-j;
    const at=(a,b)=>g.dm[b*g.nx+a]/10;
    return at(i,j)*(1-du)*(1-dv)+at(i+1,j)*du*(1-dv)+at(i,j+1)*(1-du)*dv+at(i+1,j+1)*du*dv;
  }
  return {toLocal,terrainHeight,datum,student,scale:1,
    vectorFromEN:(east,north,up)=>[R[0][0]*east+R[1][0]*north,R[0][1]*east+R[1][1]*north,up]};
}

export function insideRing(point,ring) {
  let inside=false;
  for(let i=0,j=ring.length-1;i<ring.length;j=i++) {
    const a=ring[i],b=ring[j];
    if((a[1]>point[1])!==(b[1]>point[1])&&point[0]<(b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0])inside=!inside;
  }
  return inside;
}

export function ringCentre(ring) {
  let twiceArea=0,x=0,y=0;
  for(let i=0;i<ring.length;i++) {
    const a=ring[i],b=ring[(i+1)%ring.length],cross=a[0]*b[1]-b[0]*a[1];
    twiceArea+=cross;x+=(a[0]+b[0])*cross;y+=(a[1]+b[1])*cross;
  }
  return Math.abs(twiceArea)>.001?[x/(3*twiceArea),y/(3*twiceArea)]:ring[0];
}
