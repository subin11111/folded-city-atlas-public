import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {productFile} from './scripts/build-product.mjs';

test('product static assets are not duplicated in the Python function bundle',()=>{
  const config=JSON.parse(fs.readFileSync(new URL('./vercel.json',import.meta.url),'utf8'));
  assert.equal(config.outputDirectory,'dist');
  assert.match(config.functions['api/dispatch.py'].excludeFiles, /[,{]dist\/\*\*[,}]/);
  assert.ok(config.functions['api/dispatch.py'].excludeFiles.length<=256);
});
test('research and replay cannot enter the product static bundle',()=>{
  for(const p of ['research.html','research.css','research/mapping-story.mp4','gnss-audit.html','gnss-audit.js','photo-audit.html','photo_audit/frame.jpg','gnss-recordings.js','data/gnss_demo.json','data/gnss_lobby_recorded.json','data/gnss_slam_comparison.json'])assert.equal(productFile(p),false,p);
  for(const p of ['index.html','indoor-position.js','location-controller.js','models/floor2.glb','data/navigation.json','data/photo_library.json','photos_corrected/lobby_39.jpg','vendor/maplibre/maplibre-gl.mjs'])assert.equal(productFile(p),true,p);
});
test('live location never reads a replay or advances on a timer',()=>{
  const source=fs.readFileSync(new URL('./public/location-controller.js',import.meta.url),'utf8');
  assert.doesNotMatch(source,/startReplay|startRecorded|startDemo|gnss_demo|gnss-recordings/);
  assert.ok(source.includes("if(state.mode!=='live')return"));
  assert.ok(source.includes('requestWalkingSensors()'));
  assert.ok(source.includes('pendingTransition'));
});
