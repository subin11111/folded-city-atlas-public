import fs from 'node:fs';
import {GnssFilter,rtsSmooth} from '../public/gnss-filter.js';
import {createGeoFrame} from '../public/geo-frame.js';
const data=JSON.parse(fs.readFileSync(new URL('../public/data/gnss_outside_recorded.json',import.meta.url),'utf8'));
const frame=createGeoFrame(JSON.parse(fs.readFileSync(new URL('../public/data/map.json',import.meta.url),'utf8'))),filter=new GnssFilter();
const statuses=[];
for(const r of data.records){const p=frame.fromGeo(r.coords.latitude,r.coords.longitude),s=filter.update({x:p[0],y:p[1],t:r.timestamp/1000,accuracy:r.coords.accuracy,accuracyConvention:r.coords.accuracyConvention});statuses.push({t:r.timestamp/1000,position:p,accuracy:r.coords.accuracy,accepted:s.accepted,reason:s.reason});}
const smooth=rtsSmooth(filter.history);
const metrics={source:data.source,source_sha256:data.sha256,unique_fixes:data.records.length,accepted:statuses.filter(r=>r.accepted).length,rejected:statuses.filter(r=>!r.accepted).length,segments:filter.history.filter(r=>r.segment).length,independent_ground_truth:false,absolute_error_rmse:null};
fs.writeFileSync(new URL('../../docs/assets/gnss-navigation-20261020/recorded-filter-results.json',import.meta.url),JSON.stringify({metrics,statuses,history:filter.history,smooth},null,2)+'\n');
console.log(JSON.stringify(metrics,null,2));
