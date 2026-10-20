import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
const html=fs.readFileSync(new URL('./public/research.html',import.meta.url),'utf8');
test('visual research page puts motivation before the full pipeline and detailed design',()=>{
  assert.ok(html.indexOf('id="why"')<html.indexOf('id="pipeline"'));
  assert.ok(html.indexOf('id="pipeline"')<html.indexOf('id="design"'));
  assert.ok(html.includes('저장된 2,875프레임'));
  assert.ok(html.includes('현장 보행이나 자동 실내 추적 영상은 아니다'));
});
test('every research media asset exists, and videos do not autoplay or preload large files',()=>{
  const paths=[...html.matchAll(/(?:src|poster|href)="(\/research\/[^"#]+)"/g)].map(m=>m[1]);
  assert.ok(paths.length>15);
  for(const path of paths)assert.ok(fs.statSync(new URL('./public'+path,import.meta.url)).size>0,path);
  for(const video of html.matchAll(/<video\b([^>]+)>/g)){assert.match(video[1],/controls/);assert.match(video[1],/preload="none"/);assert.doesNotMatch(video[1],/autoplay/);}
});
test('research stays in the repository and is not linked from the deployed product',()=>{
  const index=fs.readFileSync(new URL('./public/index.html',import.meta.url),'utf8');
  assert.ok(!index.includes('href="/research.html"'));
  assert.ok(index.includes('id="menuSearch"'));
});
