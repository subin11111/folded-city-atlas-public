// Create a new allowlisted handoff folder. Never merge into or clean an existing checkout.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {execFileSync} from 'node:child_process';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
execFileSync(process.execPath,[path.join(root,'scripts/check-deploy.mjs')],{stdio:'inherit'});
const out=path.join(root,'..','.dist',`folded-city-vercel-${new Date().toISOString().replace(/[:.]/g,'-')}`);
fs.mkdirSync(out,{recursive:true});
const files=['api','public','scripts','server.py','navigation.py','walk.py','photo_library.py','test_campus.mjs','test_product_ui.mjs','test_sheet.mjs','test_gnss.mjs','test_gnss_audit.mjs','test_navigation.py','test_walk.py','test_photo_library.py','test_deployment.py','package.json','package-lock.json','requirements.txt','.python-version','vercel.json','.gitignore','.vercelignore','VERCEL_DEPLOY.txt'];
for(const name of files)fs.cpSync(path.join(root,name),path.join(out,name),{recursive:true,errorOnExist:true,force:false});
execFileSync(process.execPath,[path.join(out,'scripts/check-deploy.mjs')],{stdio:'inherit'});
console.log(`Deployment source prepared (NOT deployed): ${out}`);
