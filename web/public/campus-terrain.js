import {insideRing} from './campus-data.js';

// Display-only regularisation. Never used as a surveyed height or a navigation surface.
const isWater=land=>['water','reservoir','pond'].includes(land.k);
const inPolygon=(p,rings)=>insideRing(p,rings[0])&&!rings.slice(1).some(r=>insideRing(p,r));
function segmentDistance(p,a,b){
  const dx=b[0]-a[0],dy=b[1]-a[1],t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy||1)));
  return Math.hypot(p[0]-a[0]-dx*t,p[1]-a[1]-dy*t);
}
function boundaryDistance(p,rings){
  let d=Infinity;
  for(const ring of rings)for(let i=0;i<ring.length;i++)d=Math.min(d,segmentDistance(p,ring[i],ring[(i+1)%ring.length]));
  return d;
}
export function sampleHeight(grid,values,x,y){
  const u=Math.max(0,Math.min(grid.nx-1.000001,(x-grid.x0)/grid.step));
  const v=Math.max(0,Math.min(grid.nz-1.000001,(y-grid.z0)/grid.step));
  const i=Math.floor(u),j=Math.floor(v),a=u-i,b=v-j,at=(dx,dy)=>values[(j+dy)*grid.nx+i+dx];
  return at(0,0)*(1-a)*(1-b)+at(1,0)*a*(1-b)+at(0,1)*(1-a)*b+at(1,1)*a*b;
}
export function createDisplayTerrain(campus,{maxSlope=.35}={}){
  const g=campus.terrain,raw=Float64Array.from(g.dm,v=>v/10),heights=raw.slice();
  if(!(maxSlope>0)||heights.length!==g.nx*g.nz||heights.some(v=>!Number.isFinite(v)))throw Error('Invalid background terrain');
  const waters=campus.lands.filter(isWater).map(land=>{
    const interior=[];
    for(let j=0;j<g.nz;j++)for(let i=0;i<g.nx;i++)if(inPolygon([g.x0+i*g.step,g.z0+j*g.step],land.r))interior.push(raw[j*g.nx+i]);
    interior.sort((a,b)=>a-b);
    const height=interior.length?interior[Math.floor(interior.length/2)]:sampleHeight(g,raw,...land.r[0][0]);
    return {land,height};
  });
  for(let j=0;j<g.nz;j++)for(let i=0;i<g.nx;i++){
    const p=[g.x0+i*g.step,g.z0+j*g.step],index=j*g.nx+i;
    for(const water of waters){
      // Keep a continuous mesh below the exact lake polygon: no cell-centre cut-outs.
      if(inPolygon(p,water.land.r))heights[index]=Math.min(heights[index],water.height-.12);
      else if(boundaryDistance(p,water.land.r)<=g.step*Math.SQRT2)heights[index]=Math.min(heights[index],water.height+.04);
    }
  }
  // Upper-envelope relaxation removes DEM spikes, retaining lower terrain rather
  // than inventing elevations. Eight-neighbour bounds also cover both triangle diagonals.
  const forward=[[-1,0],[0,-1],[-1,-1],[1,-1]],backward=[[1,0],[0,1],[1,1],[-1,1]];
  let changed=true,passes=0;
  while(changed&&passes<g.nx+g.nz){
    changed=false;passes++;
    for(const reverse of [false,true])for(let row=0;row<g.nz;row++)for(let col=0;col<g.nx;col++){
      const j=reverse?g.nz-1-row:row,i=reverse?g.nx-1-col:col,index=j*g.nx+i;
      for(const [dx,dy] of reverse?backward:forward){
        const x=i+dx,y=j+dy;if(x<0||x>=g.nx||y<0||y>=g.nz)continue;
        const ceiling=heights[y*g.nx+x]+maxSlope*g.step*Math.hypot(dx,dy);
        if(heights[index]>ceiling+1e-9){heights[index]=ceiling;changed=true;}
      }
    }
  }
  let rawMaxStep=0,displayMaxStep=0,changedVertices=0;
  for(let j=0;j<g.nz;j++)for(let i=0;i<g.nx;i++){
    const a=j*g.nx+i;if(Math.abs(raw[a]-heights[a])>.001)changedVertices++;
    for(const b of [i+1<g.nx?a+1:null,j+1<g.nz?a+g.nx:null])if(b!==null){rawMaxStep=Math.max(rawMaxStep,Math.abs(raw[a]-raw[b]));displayMaxStep=Math.max(displayMaxStep,Math.abs(heights[a]-heights[b]));}
  }
  return {heights,waters,height:(x,y)=>sampleHeight(g,heights,x,y),
    diagnostics:{displayOnly:true,maxSlope,rawMaxStep,displayMaxStep,changedVertices,passes}};
}
