import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {sheetDetents,nearestDetent} from './public/sheet-controller.js';
test('Sheet has ordered collapsed / medium / expanded heights even on small screens',()=>{
  for(const height of [240,420,600,788,1000]){
    const points=sheetDetents(height);
    assert.equal(points[0],56);assert.ok(points[0]<points[1]);assert.ok(points[1]<points[2]);
    assert.ok(points[2]<=Math.max(240,height));
    for(let i=0;i<3;i++)assert.equal(nearestDetent(points[i],points),i);
  }
});
test('Upward / downward flicks choose the matching detent and clamp extreme speed',()=>{
  const points=sheetDetents(788);
  assert.equal(nearestDetent(points[1],points,2),2);
  assert.equal(nearestDetent(points[1],points,-2),0);
  assert.equal(nearestDetent(-1000,points),0);
  assert.equal(nearestDetent(3000,points),2);
});
test('Sheet can be resized by grip, touch content handoff and keyboard without wide-view button',()=>{
  const html=fs.readFileSync(new URL('./public/index.html',import.meta.url),'utf8');
  const code=fs.readFileSync(new URL('./public/sheet-controller.js',import.meta.url),'utf8');
  assert.ok(html.includes('id="sheetGrip"'));assert.ok(html.includes('role="slider"'));
  assert.ok(!html.includes('id="wideMap"'));
  for(const event of ['pointerdown','pointermove','pointerup','touchmove','touchend','keydown'])assert.ok(code.includes(`'${event}'`));
  assert.ok(code.includes('touch.top<=0&&scroll.scrollTop<=0'));
  assert.ok(code.includes('scroll.inert=mobile.matches&&index===0'));
});
test('Map viewport grows with collapsed sheet and 3D camera refits on resize',()=>{
  const css=fs.readFileSync(new URL('./public/journey-shell.css',import.meta.url),'utf8');
  const scene=fs.readFileSync(new URL('./public/campus-scene.js',import.meta.url),'utf8');
  assert.ok(css.includes('height:calc(100% - var(--place-sheet-height) + 18px)'));
  assert.ok(css.includes('.workspace[data-sheet]:not(.walking) #map3d'));
  assert.ok(scene.includes("else if(mode!=='walk'&&currentBounds)fly(currentBounds"));
  assert.ok(scene.includes('if(!duration){animation=null;'));
  assert.ok(scene.includes('bounds===campusBounds?verticalFov'));
});
