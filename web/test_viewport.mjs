import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {viewportMetrics,installViewport} from './public/viewport.js';
import {sheetDetents} from './public/sheet-controller.js';

test('visible viewport wins over Safari layout height for chrome and keyboard changes',()=>{
  for(const height of [650,760,320]){
    assert.deepEqual(viewportMetrics({innerWidth:393,innerHeight:852,visualViewport:{width:393,height,offsetTop:18,offsetLeft:0,scale:1}}),{width:393,height,top:18,left:0});
    for(const inset of [0,34]){
      const points=sheetDetents(height-56-47,inset);
      assert.equal(points[0],56+inset);
      assert.ok(points[0]<points[1]&&points[1]<points[2]);
      assert.ok(points[2]<=height-56-47);
    }
  }
});

test('pinch zoom remains native and browsers without VisualViewport still work',()=>{
  assert.equal(viewportMetrics({visualViewport:{scale:2}}),null);
  assert.deepEqual(viewportMetrics({innerWidth:390,innerHeight:650}),{height:650,width:390,top:0,left:0});
});

test('viewport events are coalesced, write CSS metrics and release listeners',()=>{
  const host=new EventTarget(),viewport=new EventTarget(),values=new Map(),frames=new Map();let id=0;
  Object.assign(viewport,{width:393,height:650,offsetTop:0,offsetLeft:0,scale:1});
  Object.assign(host,{innerWidth:393,innerHeight:852,visualViewport:viewport,requestAnimationFrame(fn){frames.set(++id,fn);return id;},cancelAnimationFrame(frame){frames.delete(frame);}});
  const destroy=installViewport(host,{style:{setProperty:(k,v)=>values.set(k,v)}});
  assert.equal(values.get('--app-height'),'650px');
  viewport.height=320;viewport.offsetTop=24;
  viewport.dispatchEvent(new Event('resize'));viewport.dispatchEvent(new Event('scroll'));
  assert.equal(frames.size,1);const callback=[...frames.values()][0];frames.clear();callback();
  assert.equal(values.get('--app-height'),'320px');assert.equal(values.get('--app-top'),'24px');
  destroy();host.dispatchEvent(new Event('resize'));assert.equal(frames.size,0);
});

test('mobile chrome and photo navigation share one bounded frame with safe areas',()=>{
  const css=fs.readFileSync(new URL('./public/viewport.css',import.meta.url),'utf8');
  const html=fs.readFileSync(new URL('./public/index.html',import.meta.url),'utf8');
  assert.match(html,/\/viewport\.css\?v=[^"\s]+/);
  for(const name of ['safe-area-inset-top','safe-area-inset-bottom','safe-area-inset-left','safe-area-inset-right'])assert.ok(css.includes(name));
  assert.ok(css.includes('.workspace,.workspace.walking'));
  assert.ok(css.includes('height:var(--app-height)'));
  assert.ok(css.includes('.location-anchor-dialog select { font-size:16px; }'));
});

test('mobile keeps auxiliary controls in the menu without duplicating their actions',()=>{
  const layout=fs.readFileSync(new URL('./public/mobile-layout.js',import.meta.url),'utf8');
  const app=fs.readFileSync(new URL('./public/app.js',import.meta.url),'utf8');
  const css=fs.readFileSync(new URL('./public/viewport.css',import.meta.url),'utf8');
  assert.ok(layout.includes('destination.append(element)'));
  assert.ok(layout.includes('home.insertBefore(element,next)'));
  assert.ok(!layout.includes('cloneNode'));
  assert.ok(app.includes("['topView','mapControls']"));
  assert.ok(css.includes('.location-controls span{display:none}'));
});
