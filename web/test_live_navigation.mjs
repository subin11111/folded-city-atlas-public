import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {orbitCamera,followChanged} from './public/map-camera.js';
import {navigationEstimate} from './public/live-navigation.js';
test('left drag changes viewpoint without moving the map centre; pitch stays bounded',()=>{
  assert.notEqual(orbitCamera({bearing:-32,pitch:50},200,0).bearing,-32);
  assert.equal(orbitCamera({bearing:0,pitch:50},0,1000).pitch,70);
  assert.equal(orbitCamera({bearing:0,pitch:50},0,-1000).pitch,0);
});
test('heading-up camera rotates even when the user turns in place across north',()=>{
  const previous={position:[0,0],headingDegrees:359,headingUp:true};
  assert.equal(followChanged(previous,{position:[0,0],headingDegrees:5,headingUp:true}),true);
  assert.equal(followChanged(previous,{position:[0,0],headingDegrees:0,headingUp:true}),false);
  assert.equal(followChanged(previous,{position:[3,0],headingDegrees:359,headingUp:true}),true);
});
const route={found:true,points:[[0,0],[100,0]],endGap:37.24,entryConnected:false};
test('live guidance updates registered remaining distance, and does not hide missing entrance connection',()=>{
  const start=navigationEstimate({position:[20,0],route,entry:[137,0],indoorDistance:32.8,accuracy:5});
  const next=navigationEstimate({position:[60,0],route,entry:[137,0],indoorDistance:32.8,accuracy:5});
  assert.equal(start.stage,'outdoor');assert.ok(next.knownRemaining<start.knownRemaining);
  assert.equal(start.knownRemaining,112.8);assert.equal(start.completeConnection,false);assert.equal(start.entranceGap,37.24);
  assert.equal(navigationEstimate({position:[95,0],route,entry:[137,0],accuracy:5}).stage,'road_end');
});
test('poor signals pause automatic guidance and entry remains a manual confirmation',()=>{
  const value={position:[130,0],route,entry:[137,0],accuracy:5};
  assert.equal(navigationEstimate(value).stage,'confirm_entry');
  assert.equal(navigationEstimate({...value,accuracy:80}).stage,'weak_signal');
  assert.equal(navigationEstimate({...value,stale:true}).stage,'weak_signal');
  assert.equal(navigationEstimate({...value,rejected:true}).stage,'weak_signal');
});
test('indoor overview keeps the global basemap and current-location guidance has an explicit action',()=>{
  const scene=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  const app=fs.readFileSync(new URL('./public/app.js',import.meta.url),'utf8');
  assert.ok(scene.includes("world?.setVisible(mode!=='walk')"));
  const world=fs.readFileSync(new URL('./public/world-map.js',import.meta.url),'utf8');
  assert.ok(world.includes('this.renderer.localClippingEnabled=true'));
  assert.ok(app.includes("liveButton.id='liveRoute'"));assert.ok(app.includes('location.startNavigation('));
  assert.ok(app.includes('location.pauseFollow()'));
});
