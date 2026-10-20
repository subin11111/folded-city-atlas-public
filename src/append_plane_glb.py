"""Append textured plane primitives to an index-filtered original GLB."""
import json
from pathlib import Path
import struct
import numpy as np


def append_planes(source,target,meshes,uvs,textures):
    if target.exists():raise FileExistsError(target)
    raw=Path(source).read_bytes();length=struct.unpack_from('<I',raw,12)[0]
    doc=json.loads(raw[20:20+length]);payload=bytearray(raw[28+length:])
    def view(data,target=None):
        payload.extend(b'\0'*(-len(payload)%4));start=len(payload);payload.extend(data)
        row={'buffer':0,'byteOffset':start,'byteLength':len(data)}
        if target:row['target']=target
        doc['bufferViews'].append(row);return len(doc['bufferViews'])-1
    def accessor(values,kind):
        values=np.asarray(values,dtype='<f4');row={'bufferView':view(values.tobytes(),34962),'componentType':5126,'count':len(values),'type':kind}
        if kind=='VEC3':row.update(min=values.min(0).tolist(),max=values.max(0).tolist())
        doc['accessors'].append(row);return len(doc['accessors'])-1
    for mesh,uv,texture in zip(meshes,uvs,textures):
        ids=np.asarray(mesh.triangles).ravel();attrs={'POSITION':accessor(np.asarray(mesh.vertices)[ids],'VEC3'),
            'NORMAL':accessor(np.asarray(mesh.vertex_normals)[ids],'VEC3'),'TEXCOORD_0':accessor(uv.reshape(-1,2),'VEC2')}
        doc.setdefault('images',[]).append({'bufferView':view(Path(texture).read_bytes()),'mimeType':'image/png'})
        doc.setdefault('samplers',[]).append({'magFilter':9729,'minFilter':9729,'wrapS':33071,'wrapT':33071})
        doc.setdefault('textures',[]).append({'source':len(doc['images'])-1,'sampler':len(doc['samplers'])-1})
        doc['materials'].append({'doubleSided':True,'extensions':{'KHR_materials_unlit':{}},'pbrMetallicRoughness':{
            'metallicFactor':0.,'roughnessFactor':1.,'baseColorTexture':{'index':len(doc['textures'])-1}}})
        doc['meshes'][0]['primitives'].append({'attributes':attrs,'material':len(doc['materials'])-1,'mode':4})
    doc['buffers'][0]['byteLength']=len(payload);encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with Path(target).open('wb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(payload)));f.write(struct.pack('<I4s',len(encoded),b'JSON'));f.write(encoded)
        f.write(struct.pack('<I4s',len(payload),b'BIN\0'));f.write(payload)
