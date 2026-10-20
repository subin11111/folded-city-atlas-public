// Copy the pinned dependency assets; no generated application bundle is needed.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=fileURLToPath(new URL('../',import.meta.url)),source=path.join(root,'node_modules/maplibre-gl');
const target=path.join(root,'public/vendor/maplibre');fs.mkdirSync(target,{recursive:true});
for(const name of ['maplibre-gl.mjs','maplibre-gl-worker.mjs','maplibre-gl.css'])fs.copyFileSync(path.join(source,'dist',name),path.join(target,name));
fs.copyFileSync(path.join(source,'LICENSE.txt'),path.join(target,'LICENSE.txt'));
console.log('Pinned MapLibre browser module, worker, styles and license prepared.');
