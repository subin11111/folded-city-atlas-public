import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import crypto from 'node:crypto';
const folder=fileURLToPath(new URL('../../docs/assets/gnss-navigation-20261020/',import.meta.url));
const files=fs.readdirSync(folder).filter(f=>f!=='manifest.json'&&fs.statSync(path.join(folder,f)).isFile()).sort().map(name=>{
 const bytes=fs.readFileSync(path.join(folder,name));return {name,bytes:bytes.length,sha256:crypto.createHash('sha256').update(bytes).digest('hex')};
});
const report={date:'2026-10-20',files,scene_image_generation:false,actual_phone_field_test:false,recorded_fixes:338,synthetic_input_fixes:97,common_evaluation_epochs:96};
fs.writeFileSync(path.join(folder,'manifest.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({files:files.length,bytes:files.reduce((s,f)=>s+f.bytes,0)}));
