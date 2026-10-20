// Publish the navigation product, keeping research tools in source control only.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const source=path.join(root,'public'),output=path.join(root,'dist'),marker='.folded-city-product-build';
export function productFile(relative){
  const p=relative.split(path.sep).join('/');
  return !/^(research(?:[/.]|$)|photo_audit(?:[/.]|$)|photo-audit\.|gnss-audit\.|gnss-recordings\.js$)/.test(p)
    && !/^data\/(gnss_.*|photo_audit|provenance|asset_report|photo_colour_processing|campus_provenance|campus_osm_raw)\.json$/.test(p);
}
if(process.argv[1]&&path.resolve(process.argv[1])===fileURLToPath(import.meta.url)){
  if(path.dirname(output)!==root||path.basename(output)!=='dist')throw Error('Unexpected output directory');
  if(fs.existsSync(output)&&!fs.existsSync(path.join(output,marker)))throw Error('Existing dist is not a generated product build; preserving it.');
  if(fs.existsSync(output))fs.rmSync(output,{recursive:true});
  fs.mkdirSync(output);fs.writeFileSync(path.join(output,marker),'Folded City generated static product\n');
  let files=0,bytes=0;
  function copy(folder,relative=''){
    for(const entry of fs.readdirSync(folder,{withFileTypes:true})){
      const rel=path.posix.join(relative,entry.name);if(!productFile(rel))continue;
      if(entry.isSymbolicLink())throw Error('Symlinks are not allowed in product assets');
      const from=path.join(folder,entry.name),to=path.join(output,rel);
      if(entry.isDirectory()){fs.mkdirSync(to,{recursive:true});copy(from,rel);}
      else{fs.copyFileSync(from,to);files++;bytes+=fs.statSync(from).size;}
    }
  }
  copy(source);
  for(const filename of ['index.html','app.js','location-controller.js','indoor-position.js']){
    const text=fs.readFileSync(path.join(output,filename),'utf8');
    if(/startGnssDemo|startRecordedGnss|gnss-audit\.html|photo-audit\.html|href="\/research\.html"|window\.foldedCity=/.test(text))throw Error(`Research controls leaked into ${filename}`);
  }
  console.log(JSON.stringify({product_files:files,bytes,research_tools_published:false,output:'dist'}));
}
