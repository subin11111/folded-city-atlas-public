import test from 'node:test';
import assert from 'node:assert/strict';
import {inspectBrowserPosition,watchBrowserPosition} from './public/geolocation-source.js';

test('coarse but valid browser positions remain displayable, not confused with denied permission',()=>{
 const p={timestamp:10000,coords:{latitude:37.54,longitude:127.07,accuracy:1800}};
 assert.equal(inspectBrowserPosition(p,10500).valid,true);
 assert.equal(inspectBrowserPosition({...p,coords:{...p.coords,accuracy:null}},10500).valid,false);
 assert.equal(inspectBrowserPosition(p,30000).reason,'stale');
});
test('first coarse request and high-accuracy watch run together and cancel stale callbacks',()=>{
 const received=[],errors=[];let quick,watch,cleared;
 const geo={watchPosition(ok,bad,options){watch={ok,bad,options};return 0;},getCurrentPosition(ok,bad,options){quick={ok,bad,options};},clearWatch(id){cleared=id;}};
 const stop=watchBrowserPosition(geo,p=>received.push(p),e=>errors.push(e));
 assert.equal(quick.options.enableHighAccuracy,false);assert.equal(watch.options.enableHighAccuracy,true);
 quick.ok('coarse');watch.ok('fine');quick.bad({code:3});assert.deepEqual(received,['coarse','fine']);assert.equal(errors.length,0);
 stop();watch.ok('late');assert.equal(cleared,0);assert.equal(received.length,2);
});
test('denial, missing service and synchronous acquisition errors are visible',()=>{
 const e=[];watchBrowserPosition(null,()=>{},x=>e.push(x));assert.equal(e[0].unsupported,true);
 const geo={watchPosition(){throw Error('unavailable');},getCurrentPosition(ok,bad){bad({code:1});}};
 watchBrowserPosition(geo,()=>{},x=>e.push(x));assert.ok(e.some(x=>x.code===1));assert.ok(e.some(x=>x.code===2));
});
