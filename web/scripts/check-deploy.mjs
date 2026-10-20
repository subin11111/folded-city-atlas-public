// Read-only preflight: no deploy/login/push and no source-image processing.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const read=p=>JSON.parse(fs.readFileSync(path.join(root,p),'utf8'));
const errors=[];const warnings=[];
const ignorePatterns=fs.readFileSync(path.join(root,'.vercelignore'),'utf8').split(/\r?\n/).map(s=>s.trim());
if(ignorePatterns.includes('*')&&!ignorePatterns.includes('!web/package-lock.json'))errors.push('Git monorepo whitelist must retain web/package-lock.json');
for(const p of ['package.json','package-lock.json','public/product-shell.css','public/journey-shell.css','public/campus-terrain.js','public/campus-appearance.js','public/gnss-recordings.js'])if(!fs.existsSync(path.join(root,p)))errors.push(`Missing ${p}`);
const required=['public/index.html','public/app.js','public/walk-controller.js','public/photo-viewer.js','public/location-controller.js','public/gnss-filter.js','public/geo-frame.js','public/outdoor-routing.js','public/location.css','public/data/gnss_demo.json','public/data/gnss_outside_recorded.json','api/dispatch.py','server.py','navigation.py','walk.py','photo_library.py','vercel.json','requirements.txt','.python-version'];
required.push('public/world-map.js','public/world-registration.js','public/world-map.css','public/map-camera.js','public/live-navigation.js','public/data/world_style.json',
 'public/vendor/maplibre/maplibre-gl.mjs','public/vendor/maplibre/maplibre-gl-worker.mjs','public/vendor/maplibre/maplibre-gl.css','public/vendor/maplibre/LICENSE.txt');
for(const p of required)if(!fs.existsSync(path.join(root,p)))errors.push(`Missing ${p}`);
for(const p of ['gnss-audit.html','gnss-audit.js','gnss-audit.css','data/gnss_slam_comparison.json',...['outside','lobby','bridge_out1f','floor2','bridge_12_sub'].map(s=>`data/gnss_${s}_recorded.json`)])if(!fs.existsSync(path.join(root,'public',p)))errors.push(`Missing measured GNSS comparison ${p}`);
const config=read('vercel.json');
const research=fs.readFileSync(path.join(root,'public/research.html'),'utf8');
for(const match of research.matchAll(/(?:src|poster|href)="(\/research\/[^"#]+)"/g))if(!fs.existsSync(path.join(root,'public',match[1])))errors.push(`Missing research media ${match[1]}`);
if(!fs.existsSync(path.join(root,'public/research.css')))errors.push('Missing research stylesheet');
if(config.framework!==null||config.outputDirectory!=='public')errors.push('Expected buildless frontend + Python API');
if(!config.functions?.['api/dispatch.py']?.includeFiles)errors.push('Missing explicit runtime data include');
for(const [entry,settings] of Object.entries(config.functions||{}))for(const field of ['includeFiles','excludeFiles']){
 if(typeof settings[field]==='string'&&settings[field].length>256)errors.push(`Vercel ${entry}.${field} exceeds the 256-character schema limit`);
}
const endpoints=['route','health','walk/start','walk/options','walk/move'];
for(const endpoint of endpoints)if(!config.rewrites.some(r=>r.source===`/api/${endpoint}`&&r.destination===`/api/dispatch?endpoint=${endpoint}`))errors.push(`API rewrite missing: ${endpoint}`);
const nav=read('public/data/navigation.json'), regions=read('public/data/photo_regions.json');
const digest=crypto.createHash('sha256').update(fs.readFileSync(path.join(root,'public/data/navigation.json'))).digest('hex');
if(regions.navigation_sha256!==digest)errors.push('Photo navigation index is stale');
if(nav.recording_trajectory_used_for_routing!==false)errors.push('Recording trajectory must not be used for routing');
for(const place of read('public/data/catalog.json'))if(!place.navigation_node||!place.photo)errors.push(`Unresolved destination: ${place.id}`);
let count=0,total=0,max=0,largest='';
function walk(folder){for(const item of fs.readdirSync(folder,{withFileTypes:true})){
 const full=path.join(folder,item.name);
 if(item.isSymbolicLink()){errors.push(`Symlink not allowed in public: ${full}`);continue;}
 if(item.isDirectory()){walk(full);continue;}
 const relative=path.relative(root,full).split(path.sep).join('/'),size=fs.statSync(full).size;
 count++;total+=size;if(size>max){max=size;largest=relative;}
 if(size>=100*1024*1024)errors.push(`Git single-file limit risk: ${relative}`);
 if(/\.(db|sqlite3|zip|docx|pdf|npz)$/i.test(relative)||/(^|\/)\.env/.test(relative))errors.push(`Private/raw artifact in public: ${relative}`);
}}
walk(path.join(root,'public'));
for(const photo of read('public/data/photo_library.json'))for(const key of ['url','display_url','thumb_url']){
 const relative=photo[key];if(!relative||relative.includes('..')||!fs.existsSync(path.join(root,'public',relative)))errors.push(`Missing photo ${photo.id}/${key}`);
}
if(total>500*1024*1024)warnings.push('Large static asset set. Confirm Git upload, Vercel plan/build/transfer budgets before publishing.');
warnings.push('Photos may include people, posters and private notices. Review consent/redaction and building publication permission before public release. noindex is NOT access control.');
const result={status:errors.length?'FAIL':'LOCAL_PREFLIGHT_PASS',deployment_performed:false,files:count,static_bytes:total,largest_file:largest,largest_bytes:max,errors,warnings};
console.log(JSON.stringify(result,null,2));
if(errors.length)process.exitCode=1;
