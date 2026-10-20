import hashlib,json,struct
import numpy as np
import pytest
from level_floor_materials import consistent_floor_colour,replace_floor_references


def test_unreliable_floor_colours_are_smooth_not_stretched_patterns():
    y,x=np.indices((40,40));rgb=np.full((40,40,3),170,np.uint8);rgb[15:25,15:25]=[80,80,80];reliable=np.zeros((40,40),bool);reliable[5:35,5:35]=True;surface=np.ones_like(reliable)
    cfg={'lighting_radius_m':.5,'target_linear_luminance':.28,'log_detail_bounds':[-2.3,.65],'photo_edge_feather_m':.1}
    out,report=consistent_floor_colour(rgb,reliable,surface,.05,cfg)
    assert out.dtype==np.uint8 and out.shape==rgb.shape
    assert np.unique(out[~reliable],axis=0).shape[0]==1
    assert out[20,20].mean()<out[10,10].mean()
    assert report['log_detail_bounds']==[-2.3,.65]
    assert not report['invented_tile_pattern']


@pytest.mark.parametrize('restore_original', [False, True])
def test_floor_material_relink_preserves_original_bytes_and_material(tmp_path,restore_original):
    original=b'original-floor-image';replacement=tmp_path/'new.png';replacement.write_bytes(original if restore_original else b'new-derived-floor-image')
    doc={'asset':{'version':'2.0'},'buffers':[{'byteLength':len(original)}],'bufferViews':[{'buffer':0,'byteOffset':0,'byteLength':len(original)}],
         'images':[{'bufferView':0,'mimeType':'image/png'}],'samplers':[{}],'textures':[{'source':0,'sampler':0}],
         'materials':[{'pbrMetallicRoughness':{'baseColorTexture':{'index':0}}}],
         'meshes':[{'primitives':[{'attributes':{},'material':0}]}]}
    encoded=json.dumps(doc).encode();encoded+=b' '*(-len(encoded)%4);raw=struct.pack('<4sII',b'glTF',2,28+len(encoded)+len(original))+struct.pack('<I4s',len(encoded),b'JSON')+encoded+struct.pack('<I4s',len(original),b'BIN\0')+original
    source=tmp_path/'source.glb';target=tmp_path/'new.glb';source.write_bytes(raw);digest=hashlib.sha256(original).hexdigest();hits=replace_floor_references(source,target,{digest:replacement});assert hits[digest]==1
    output=target.read_bytes();size=struct.unpack_from('<I',output,12)[0];changed=json.loads(output[20:20+size]);payload=output[28+size:]
    assert len(output)%4==0 and len(payload)%4==0
    assert struct.unpack_from('<I',output,8)[0]==len(output)
    assert payload[:len(original)]==original
    assert changed['materials'][0]==doc['materials'][0]
    assert changed['meshes'][0]['primitives'][0]['material']==1
    image=changed['bufferViews'][changed['images'][-1]['bufferView']];start=image['byteOffset']
    assert payload[start:start+image['byteLength']]==replacement.read_bytes()
    assert source.read_bytes()==raw


def test_dark_photo_colour_noise_is_not_amplified():
    rng=np.random.default_rng(22);rgb=rng.integers(20,56,size=(40,40,3),dtype=np.uint8);mask=np.ones((40,40),bool)
    cfg={'lighting_radius_m':.5,'target_linear_luminance':.28,'log_detail_bounds':[-2.3,.65],'photo_edge_feather_m':.1}
    out,report=consistent_floor_colour(rgb,mask,mask,.05,cfg)
    ratio=out[5:-5,5:-5,0]/np.maximum(out[5:-5,5:-5,1],1)
    assert np.ptp(ratio)<.03
    assert report['dark_chroma_luminance_ramp']==[.05,.20]
