import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {createRoutePlayback} from './public/route-playback.js';

function setup(){
  const state={route:{found:true},progress:0,playing:false},changes=[];
  const player=createRoutePlayback({state,onProgress:value=>state.progress=value,onPlaying:value=>changes.push(value)});
  return {state,changes,player};
}
test('first frame after starting or resuming never includes idle time',()=>{
  const {state,player}=setup();
  player.tick(0);player.setPlaying(true);player.tick(120000);
  assert.equal(state.progress,0);
  player.tick(120500);assert.equal(state.progress,1);
  player.setPlaying(false);player.tick(150000);assert.equal(state.progress,1);
  player.setPlaying(true);player.tick(200000);assert.equal(state.progress,1);
  player.tick(200500);assert.equal(state.progress,2);
});
test('hidden-tab time is excluded even when no frames run while hidden',()=>{
  const {state,player}=setup();player.setPlaying(true);
  player.tick(0);player.tick(500);assert.equal(state.progress,1);
  player.resetClock();player.tick(100000);assert.equal(state.progress,1);
  player.tick(100500);assert.equal(state.progress,2);
  player.tick(101000,true);player.tick(200000);assert.equal(state.progress,2);
});
test('arrival stops at 100 percent and a new run starts without a time jump',()=>{
  const {state,changes,player}=setup();player.setPlaying(true);
  player.tick(0);player.tick(51000);
  assert.equal(state.progress,100);assert.equal(state.playing,false);
  assert.deepEqual(changes,[true,false]);
  state.progress=0;player.setPlaying(true);player.tick(100000);
  assert.equal(state.progress,0);player.tick(100500);assert.equal(state.progress,1);
  state.route={found:false};player.setPlaying(true);assert.equal(state.playing,false);
});
test('moving the actual scene marker invalidates the demand-rendered basemap',()=>{
  const source=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  const method=source.match(/moveMarker\(p\)\{([^}]+)\}/)[0];
  const marker={position:{set(...p){this.value=p;}},visible:false};let paints=0;
  const scene=vm.runInNewContext(`({${method}})`,{marker,mode:'indoor',world:{repaint(){paints++;}}});
  scene.moveMarker([1,2,3]);scene.moveMarker([4,5,6]);
  assert.deepEqual(marker.position.value,[4,5,6.3]);assert.equal(marker.visible,true);assert.equal(paints,2);
});
