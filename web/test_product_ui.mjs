import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
const html=fs.readFileSync(new URL('./public/index.html',import.meta.url),'utf8');
const app=fs.readFileSync(new URL('./public/app.js',import.meta.url),'utf8');

test('UI identifiers remain unique and controller targets still exist',()=>{
  const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
  assert.equal(new Set(ids).size,ids.length);
  const dynamic=new Set(['routeAlternatives','walkRoute','liveRoute','indoorLiveRoute','steps']);
  for(const file of ['app.js','walk-controller.js','photo-viewer.js']){
    const source=fs.readFileSync(new URL(`./public/${file}`,import.meta.url),'utf8');
    for(const match of source.matchAll(/\$\('([^']+)'\)/g))assert.ok(ids.includes(match[1])||dynamic.has(match[1]),`${file}: ${match[1]}`);
  }
});

test('Product menu contains guidance only; research controls are not shipped in the UI',()=>{
  const menu=html.slice(html.indexOf('<dialog id="evidenceDialog"'),html.indexOf('<div id="toast"'));
  for(const id of ['projectInfo','currentViewDetails','currentRouteDetails','sceneBadge','avoidStairs','verifiedOnly','evidenceContent'])assert.ok(!menu.includes(`id="${id}"`),id);
  assert.ok(!html.includes('class="status-strip"'));
  assert.ok(!html.includes('href="/photo-audit.html"'));
  assert.ok(!app.includes('href="/photo-audit.html"'));
  assert.ok(menu.includes('id="menuIndoorStart"'));
  assert.ok(html.includes('https://www.openstreetmap.org/copyright'));
  assert.ok(html.includes('aria-controls="evidenceDialog"'));
});

test('Real photo cards lead directly from one search into a route',()=>{
  const places=JSON.parse(fs.readFileSync(new URL('./public/data/catalog.json',import.meta.url),'utf8'));
  for(const p of places)assert.ok(fs.existsSync(new URL(`./public/photos_corrected/${p.photo.split('/').at(-1)}`,import.meta.url)),p.id);
  for(const id of ['placesPanel','journeyPanel','changePlace','routeSettings','menuSearch'])assert.ok(html.includes(`id="${id}"`));
  assert.ok(!html.includes('role="tablist"'));
  assert.ok(!app.includes('selectPanel('));
  assert.ok(app.includes("showJourney('route');await findRoute()"));
  assert.ok(app.includes('$(' + "'search').blur()"));
});

test('Walking keeps secondary functions accessible without floating them over the photo',()=>{
  for(const id of ['walkSettings','walkSettingsToggle','walkMapPanel','walkMapToggle','walkSecondaryOptions','walkOtherDirections'])assert.ok(html.includes(`id="${id}"`));
  assert.ok(!html.includes('class="walk-location"'));
  assert.ok(!html.includes('class="walk-destination"'));
  assert.ok(html.includes('id="walkSettings" class="walk-settings" aria-label="안내 보기 설정" hidden'));
  assert.ok(html.includes('id="walkMapPanel" class="walk-map-panel" aria-label="현재 탐색 위치" hidden'));
  for(const id of ['photoFit','photoBrightness','walkViewToggle','walkPhotoButton'])assert.ok(html.indexOf(`id="${id}"`)>html.indexOf('id="walkSettings"'));
  const walker=fs.readFileSync(new URL('./public/walk-controller.js',import.meta.url),'utf8');
  assert.ok(walker.includes('(option.recommended?options:secondary)'));
  assert.ok(walker.includes('ResizeObserver(syncSheetHeight)'));
});

test('Mobile fills the available viewport with a consistent native font stack',()=>{
  const css=fs.readFileSync(new URL('./public/journey-shell.css',import.meta.url),'utf8');
  assert.ok(html.includes('viewport-fit=cover'));
  assert.ok(css.includes('"Apple SD Gothic Neo"'));
  assert.ok(css.includes('text-size-adjust: 100%'));
  assert.ok(css.includes('body.is-walking .topbar { display:none; }'));
  assert.ok(css.includes('.walking .map-shell { grid-column:2;'));
  assert.ok(css.includes('.photo-images,.photo-stage.fill .photo-images { inset:0; }'));
  assert.ok(css.includes('env(safe-area-inset-bottom,0px)'));
  assert.ok(css.includes('.search-input input { font-size:16px; }'));
  assert.ok(html.includes('class="photo-stage fill"'));
});
