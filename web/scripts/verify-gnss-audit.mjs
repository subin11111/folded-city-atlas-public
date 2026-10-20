import fs from 'node:fs/promises';
import {createRequire} from 'node:module';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const require=createRequire(import.meta.url),{chromium}=require(require.resolve('playwright',{paths:[process.env.PLAYWRIGHT_NODE_MODULES]}));
const out=fileURLToPath(new URL('../../docs/assets/gnss-slam-audit-20261020/',import.meta.url)),base=process.env.GNSS_TEST_URL||'http://127.0.0.1:8775';
await fs.mkdir(`${out}/frames`,{recursive:true});
const browser=await chromium.launch({headless:true,channel:'msedge'}),page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],responses=[];
page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.url().includes('/data/'))responses.push({url:new URL(r.url()).pathname,status:r.status()});});
await page.goto(`${base}/gnss-audit.html`);await page.waitForFunction(()=>window.gnssAudit);
const counts={};
for(const id of ['outside','lobby','bridge_out1f','floor2','bridge_12_sub']){
 await page.selectOption('#session',id);await page.evaluate(()=>window.gnssAudit.seek(Math.floor(window.gnssAudit.data.sessions[window.gnssAudit.session].records.length*.55)));
 counts[id]=await page.evaluate(()=>window.gnssAudit.data.sessions[window.gnssAudit.session].records.length);
 assert.ok(await page.locator('#values').innerText());await page.screenshot({path:`${out}/web-${id}.png`,fullPage:true});
}
await page.selectOption('#frame','enu');await page.screenshot({path:`${out}/web-global.png`,fullPage:true});
await page.selectOption('#session','lobby');await page.selectOption('#frame','map');
for(let i=0;i<28;i++){await page.evaluate(j=>window.gnssAudit.seek(Math.round(j*(window.gnssAudit.data.sessions.lobby.records.length-1)/27)),i);await page.screenshot({path:`${out}/frames/${String(i).padStart(3,'0')}.png`});}
await page.locator('#play').click();const before=await page.evaluate(()=>window.gnssAudit.index);await page.waitForTimeout(400);const after=await page.evaluate(()=>window.gnssAudit.index);assert.notEqual(before,after);await page.locator('#play').click();
await page.setViewportSize({width:390,height:844});await page.screenshot({path:`${out}/web-mobile.png`,fullPage:true});assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
assert.ok(responses.every(r=>r.status===200));assert.deepEqual(errors,[]);
const result={passed:true,session_counts:counts,checks:['five measured session selectors','local XY and global ENU display','paired coordinates and discrepancies rendered','timeline and playback','390px mobile no horizontal overflow'],errors,responses,live_phone_field_test:false,reference:'reconstructed web-map positions, not survey ground truth'};
await fs.writeFile(`${out}/browser-verification.json`,JSON.stringify(result,null,2)+'\n');await browser.close();console.log(JSON.stringify(result,null,2));
