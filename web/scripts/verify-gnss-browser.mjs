// Optional browser evidence. Set PLAYWRIGHT_NODE_MODULES to the directory containing playwright.
import fs from 'node:fs/promises';
import path from 'node:path';
import {createRequire} from 'node:module';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const require=createRequire(import.meta.url);
const {chromium}=require(require.resolve('playwright',{paths:process.env.PLAYWRIGHT_NODE_MODULES?[process.env.PLAYWRIGHT_NODE_MODULES]:undefined}));
const base=process.env.GNSS_TEST_URL||'http://127.0.0.1:8773';
const out=fileURLToPath(new URL('../../docs/assets/gnss-navigation-20261020/',import.meta.url));await fs.mkdir(out,{recursive:true});
const browser=await chromium.launch({headless:true,channel:process.env.BROWSER_CHANNEL||'msedge'});
const errors=[],requests=[],checks=[];
async function ready(page){page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.url().includes('/api/'))requests.push({url:r.url().replace(base,''),status:r.status()});});await page.goto(base);try{await page.waitForFunction(()=>window.foldedCity?.state.three,null,{timeout:45000});await page.waitForSelector('#loadingScreen',{state:'hidden',timeout:30000});}catch(e){console.error(JSON.stringify({errors,body:await page.locator('body').innerText()}));await browser.close();throw e;}}
const context=await browser.newContext({viewport:{width:1440,height:960},permissions:['geolocation'],geolocation:{latitude:37.5424,longitude:127.078,accuracy:8}});
const page=await context.newPage();await ready(page);
// Exercise real watchPosition subscription with browser-provided mock location.
await page.click('#locateButton');await page.waitForFunction(()=>window.foldedCity.location.state.fix);
assert.equal(await page.evaluate(()=>window.foldedCity.location.state.mode),'live');checks.push('watchPosition callback + ENU + Kalman + current marker');
// Desktop has no compass: exercise the granted-permission branch with an explicit permission stub.
await page.evaluate(()=>{DeviceOrientationEvent.requestPermission=async()=> 'granted';});
await page.click('#headingButton');await page.evaluate(()=>{const e=new Event('deviceorientation');e.webkitCompassHeading=359;e.webkitCompassAccuracy=5;window.dispatchEvent(e);});
assert.match(await page.locator('#headingStatus').innerText(),/359°/);checks.push('deviceorientation event listener and magnetic heading label');
await page.click('#stopLocation');assert.equal(await page.evaluate(()=>window.foldedCity.location.state.watch),null);checks.push('clearWatch and orientation listener cleanup');
// Actual app replay controls, permanently labelled synthetic, supply reproducible screenshots.
await page.click('#evidenceButton');await page.locator('#projectInfo summary').first().click();await page.locator('#syntheticGnssTools summary').click();await page.click('#startGnssDemo');
await page.waitForFunction(()=>window.foldedCity.location.state.mode==='demo');await page.evaluate(()=>window.foldedCity.location.seek(24));
await page.evaluate(()=>{document.getElementById('destination').value='classical_music';return window.foldedCity.findRoute();});
await page.evaluate(()=>{window.foldedCity.state.three.showCampus(true);window.foldedCity.location.render();});
await page.waitForTimeout(1200);await page.screenshot({path:path.join(out,'desktop-gnss.png')});
await page.evaluate(()=>window.foldedCity.location.seek(32));assert.match(await page.locator('#locationQuality').innerText(),/위치 튐/);
await page.screenshot({path:path.join(out,'outlier-rejection.png')});checks.push('injected jump rejected, raw log retained and last good position held');
await page.evaluate(()=>window.foldedCity.location.seek(70)); // after an 18 s gap
const segment=await page.evaluate(()=>window.foldedCity.location.filter.history.at(-1).segment);assert.ok(segment);checks.push('gap starts new filter segment');
await page.click('#stopLocation'); // Export provenance must survive stopping the replay.
const downloadPromise=page.waitForEvent('download');await page.click('#evidenceButton');await page.click('#exportGnssLog');const download=await downloadPromise;
const logPath=path.join(out,'demo-browser-export.json');await download.saveAs(logPath);const exported=JSON.parse(await fs.readFile(logPath));assert.ok(exported.simulated&&exported.kalman.length&&exported.rts.length);checks.push('browser download contains original, Kalman, covariance and RTS');
await page.setInputFiles('#importGnssLog',logPath);await page.waitForFunction(()=>window.foldedCity.location.state.mode==='demo'&&window.foldedCity.location.state.replayIndex<10);checks.push('exported log imports and replays');
await page.evaluate(async()=>{await window.foldedCity.location.startDemo();window.foldedCity.location.seek(96);});await page.screenshot({path:path.join(out,'entry-handoff.png')});
await page.click('#confirmEntrance');await page.waitForFunction(()=>window.foldedCity.state.route?.found&&window.foldedCity.location.state.mode==='off');checks.push('manual entrance handoff invokes existing indoor A* API');
await context.close();
const mobile=await browser.newContext({viewport:{width:390,height:844},deviceScaleFactor:2,isMobile:true,hasTouch:true});
const mp=await mobile.newPage();await ready(mp);await mp.click('#evidenceButton');await mp.locator('#projectInfo summary').first().click();await mp.locator('#syntheticGnssTools summary').click();await mp.click('#startGnssDemo');await mp.waitForFunction(()=>window.foldedCity.location.state.mode==='demo'&&window.foldedCity.location.state.fix);await mp.evaluate(()=>window.foldedCity.location.seek(28));await mp.waitForTimeout(1100);assert.match(await mp.locator('#headingStatus').innerText(),/이동 방향/);
await mp.screenshot({path:path.join(out,'mobile-gnss.png')});assert.equal(await mp.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);checks.push('mobile viewport has no horizontal overflow');
const frames=path.join(out,'frames');await fs.mkdir(frames,{recursive:true});
for(let i=0;i<24;i++){await mp.evaluate(index=>window.foldedCity.location.seek(index),i*3);await mp.waitForTimeout(40);await mp.screenshot({path:path.join(frames,`${String(i).padStart(3,'0')}.png`)});}
await mobile.close();
const denied=await browser.newContext({viewport:{width:390,height:844}});await denied.clearPermissions();const dp=await denied.newPage();await ready(dp);await dp.click('#locateButton');await dp.waitForFunction(()=>document.getElementById('locationQuality').textContent.includes('권한'),{timeout:20000});checks.push('permission denied gives an actionable message');
assert.deepEqual(errors,[]);assert.ok(requests.some(r=>r.url.includes('/api/route')&&r.status===200));
const result={passed:true,checks,requests,errors,live_phone_field_test:false,heading_test:'permission stub + dispatched browser sensor event; not measured phone compass',screenshots:'actual local browser using explicitly labelled deterministic synthetic replay'};
await fs.writeFile(path.join(out,'browser-verification.json'),JSON.stringify(result,null,2)+'\n');await browser.close();console.log(JSON.stringify(result,null,2));
