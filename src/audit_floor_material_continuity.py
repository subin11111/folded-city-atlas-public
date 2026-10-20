#!/usr/bin/env python3
"""Verify actual floor material links, untouched geometry/UV and source bytes."""
import argparse,hashlib,json,struct
from pathlib import Path
from optimize_session import sha256_file,utc_now


def unpack(path,strict=False):
    raw=Path(path).read_bytes();size=struct.unpack_from('<I',raw,12)[0]
    if strict:
        assert raw[:4]==b'glTF' and struct.unpack_from('<I',raw,4)[0]==2
        assert struct.unpack_from('<I',raw,8)[0]==len(raw) and len(raw)%4==0
        assert size%4==0 and raw[16:20]==b'JSON'
        assert raw[24+size:28+size]==b'BIN\0'
        assert struct.unpack_from('<I',raw,20+size)[0]==len(raw)-28-size
        assert (len(raw)-28-size)%4==0
    return json.loads(raw[20:20+size]),raw[28+size:]


def image_hashes(doc,binary):
    result=[]
    for image in doc['images']:
        view=doc['bufferViews'][image['bufferView']];start=view.get('byteOffset',0);result.append(hashlib.sha256(binary[start:start+view['byteLength']]).hexdigest())
    return result


def material_image(doc,hashes,index):
    t=doc['materials'][index].get('pbrMetallicRoughness',{}).get('baseColorTexture',{}).get('index')
    return None if t is None else hashes[doc['textures'][t]['source']]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['floor_material_source_manifest']).parent;expected={r['source_texture_sha256']:r['new_texture_sha256'] for r in run['floor_material_reports']};result={}
    photographic=run.get('floor_photo_preservation',False)
    if photographic:assert all(old==new for old,new in expected.items()),'Photographic floor atlas changed'
    for name,s in run['sessions'].items():
        before,old=unpack(base/name/'clean_textured.glb');after,new=unpack(a.bundle/name/'clean_textured.glb',strict=True);assert new[:len(old)]==old,'Source binary changed';old_images=image_hashes(before,old);new_images=image_hashes(after,new);assert old_images==new_images[:len(old_images)];assert before['accessors']==after['accessors'];assert before['materials']==after['materials'][:len(before['materials'])];old_primitives=before['meshes'][0]['primitives'];new_primitives=after['meshes'][0]['primitives'];assert len(old_primitives)==len(new_primitives);changed=0
        for first,second in zip(old_primitives,new_primitives):
            assert {k:v for k,v in first.items() if k!='material'}=={k:v for k,v in second.items() if k!='material'},'Primitive topology or attributes changed';old_hash=material_image(before,old_images,first['material']);new_hash=material_image(after,new_images,second['material'])
            if old_hash in expected:assert new_hash==expected[old_hash];changed+=1
            else:assert first['material']==second['material'] and old_hash==new_hash,'Non-floor material changed'
        assert sha256_file(base/name/'clean_structural.ply')==sha256_file(a.bundle/name/'clean_structural.ply');assert changed==sum(s['floor_material_references_changed'].values());result[name]={'only_matching_floor_materials_changed':True,'changed_floor_primitives':changed,'original_binary_prefix_unchanged':True,'original_image_streams_preserved':len(old_images),'position_normal_UV_accessors_unchanged':True,'PLY_geometry_file_unchanged':True,'GLB_header_chunk_lengths_and_alignment_valid':True}
    output={'created_utc':utc_now(),'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'sessions':result,'exact_original_floor_atlas_restoration':photographic,'audit_source_sha256':sha256_file(Path(__file__)),'scope':run['scope'] if photographic or run.get('photo_seam_reconstruction') else 'Appearance-only display material change. Reliable original photographic detail is normalized; missing floor texture uses inferred smooth photographed stone colour, not reconstructed original tile layout.'};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n');print('Actual floor material links and untouched geometry/UV verified.')


if __name__=='__main__':main()
