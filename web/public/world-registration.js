// Local SLAM metres -> WGS84 -> Web Mercator. The existing registration is unchanged.
const radians=Math.PI/180,A=6378137,E2=6.6943799901413165e-3;
// The flat basemap has no surveyed altitude. Align its display plane to the
// first floor, using ONE translation for every indoor floor and route vertex.
// Never align each selected floor separately or change the navigation data.
export function displayElevationOffset(map,{mode='campus',campusLift=0,clearance=.08}={}){
  if(mode!=='indoor')return campusLift;
  const ground=map.levels.find(level=>level.id==='1')?.z;
  if(!Number.isFinite(ground))throw Error('실내 1층의 기준 높이가 필요합니다.');
  return clearance-ground;
}
export function mercatorPoint([lon,lat]){
  const phi=Math.max(-85.05112878,Math.min(85.05112878,lat))*radians;
  return [(lon+180)/360,(1-Math.log(Math.tan(Math.PI/4+phi/2))/Math.PI)/2];
}
export function createWorldRegistration(map,{lift=0}={}){
  const g=map.georeference,R=g.R_map_xy_to_EN,t=g.t_map_xy_to_EN_m,o=map.manual_preview_offset_en_m||[2,-2];
  const lat=g.origin_latitude_deg*radians,lon=g.origin_longitude_deg*radians;
  const sl=Math.sin(lat),cl=Math.cos(lat),sn=Math.sin(lon),cn=Math.cos(lon),N=A/Math.sqrt(1-E2*sl*sl);
  const origin=[N*cl*cn,N*cl*sn,N*(1-E2)*sl];
  function toLngLat([x,y]){
    const east=R[0][0]*x+R[0][1]*y+t[0]+o[0],north=R[1][0]*x+R[1][1]*y+t[1]+o[1];
    const ecef=[origin[0]-sn*east-sl*cn*north,origin[1]+cn*east-sl*sn*north,origin[2]+cl*north];
    const p=Math.hypot(ecef[0],ecef[1]);let phi=Math.atan2(ecef[2],p*(1-E2));
    for(let i=0;i<8;i++){const n=A/Math.sqrt(1-E2*Math.sin(phi)**2);phi=Math.atan2(ecef[2]+E2*n*Math.sin(phi),p);}
    return [Math.atan2(ecef[1],ecef[0])/radians,phi/radians];
  }
  const anchor=toLngLat([0,0]),[mx,my]=mercatorPoint(anchor),scale=1/(2*Math.PI*A*Math.cos(anchor[1]*radians));
  // Use the WGS84-to-Mercator Jacobian, not a spherical approximation to ENU metres.
  const xp=mercatorPoint(toLngLat([10,0])),xm=mercatorPoint(toLngLat([-10,0]));
  const yp=mercatorPoint(toLngLat([0,10])),ym=mercatorPoint(toLngLat([0,-10]));
  // Column-major matrix. Mercator Y points south; local Z stays up.
  const matrix=[(xp[0]-xm[0])/20,(xp[1]-xm[1])/20,0,0,(yp[0]-ym[0])/20,(yp[1]-ym[1])/20,0,0,0,0,scale,0,mx,my,lift*scale,1];
  return {toLngLat,anchor,scale,matrix};
}
