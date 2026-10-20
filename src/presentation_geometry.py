"""Display-only vertical normalization; original calibrated maps remain immutable."""
import json,struct
from pathlib import Path
import numpy as np
from reconcile_multilevel_surfaces import sample_lattice


class LevelPresentation:
    def __init__(self,root,config):
        self.config=config;self.models={}
        for key,file in config['height_models'].items():
            pack=np.load(Path(root)/file);self.models[key]={k:pack[k] for k in pack.files}
        self.lower=float(config['level_anchors']['lower_floor_m']);self.upper=float(config['level_anchors']['upper_floor_m'])

    def warp(self,points,session):
        points=np.asarray(points,dtype=np.float32);result=points.copy()
        if session not in ['lobby','floor2','bridge_12_sub']:return result
        lower=sample_lattice(self.models['sub_lower' if session=='bridge_12_sub' else 'lower'],points[:,:2]);upper=sample_lattice(self.models['upper'],points[:,:2])
        z=points[:,2].astype(float);fraction=np.clip((z-lower)/np.maximum(upper-lower,1.),0.,1.)
        shift=(1-fraction)*(self.lower-lower)+fraction*(self.upper-upper)
        result[:,2]=(z+shift).astype(np.float32)
        assert np.array_equal(result[:,:2],points[:,:2])
        return result


def warp_glb(source,target,warp):
    """Change POSITION Z only; embedded photos, UV, X/Y and material bytes stay."""
    source,target=Path(source),Path(target)
    if target.exists():raise FileExistsError(target)
    raw=source.read_bytes();size=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+size]);binary=bytearray(raw[28+size:]);done=set()
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            index=primitive['attributes']['POSITION']
            if index in done:continue
            acc=doc['accessors'][index];view=doc['bufferViews'][acc['bufferView']]
            if acc['type']!='VEC3' or acc['componentType']!=5126 or view.get('byteStride',12)!=12:raise ValueError('Unsupported position stream')
            values=np.ndarray((acc['count'],3),dtype='<f4',buffer=binary,offset=view.get('byteOffset',0)+acc.get('byteOffset',0));changed=warp(values.copy())
            if not np.array_equal(changed[:,:2],values[:,:2]):raise ValueError('Presentation changed XY')
            values[:,2]=changed[:,2];acc['min']=values.min(0).astype(float).tolist();acc['max']=values.max(0).astype(float).tolist();done.add(index)
    encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with target.open('wb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(binary)));f.write(struct.pack('<I4s',len(encoded),b'JSON'));f.write(encoded);f.write(struct.pack('<I4s',len(binary),b'BIN\0'));f.write(binary)
