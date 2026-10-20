import json
import struct

import numpy as np

from build_unified_presentation import ceiling_fit, clip_halfspace
from presentation_geometry import warp_glb


def test_polygon_halfspace_clips_at_boundary_without_extrapolation():
    square=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    clipped=clip_halfspace(square,lambda p:p[0])
    assert len(clipped)==4
    assert (clipped[:,0]>=0).all()
    assert set(map(tuple,clipped))=={(0.,-1.),(1.,-1.),(1.,1.),(0.,1.)}
    assert clip_halfspace(square,lambda p:p[0]-2).shape==(0,2)


def test_coherent_ceiling_plane_accepted_but_vertical_wall_rejected():
    x,y=np.meshgrid(np.linspace(-2,2,20),np.linspace(-1,1,20))
    ceiling=np.c_[x.ravel(),y.ravel(),5.+.03*x.ravel()]
    plane,report=ceiling_fit(ceiling)
    assert plane is not None and report['accepted']
    wall=np.c_[np.zeros(x.size),x.ravel(),y.ravel()]
    plane,report=ceiling_fit(wall)
    assert plane is None and not report['accepted']


def test_disconnected_parallel_depth_layers_do_not_authorize_ceiling():
    x,y=np.meshgrid(np.linspace(-2,2,18),np.linspace(-1,1,18))
    q=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    mixed=np.r_[q+[0,0,3],q+[0,0,5],q+[0,0,7]]
    plane,report=ceiling_fit(mixed)
    assert plane is None and not report['accepted']


def test_glb_changes_z_only_and_preserves_other_binary_streams(tmp_path):
    positions=np.array([[1,2,3],[4,5,6],[7,8,9]],dtype='<f4')
    suffix=b'original-photo-and-uv-stream!'
    suffix+=b'\0'*(-len(suffix)%4)
    doc={'asset':{'version':'2.0'},'buffers':[{'byteLength':positions.nbytes+len(suffix)}],
         'bufferViews':[{'buffer':0,'byteOffset':0,'byteLength':positions.nbytes}],
         'accessors':[{'bufferView':0,'componentType':5126,'count':3,'type':'VEC3'}],
         'meshes':[{'primitives':[{'attributes':{'POSITION':0}}]}]}
    encoded=json.dumps(doc).encode();encoded+=b' '*(-len(encoded)%4)
    binary=positions.tobytes()+suffix
    raw=struct.pack('<4sII',b'glTF',2,28+len(encoded)+len(binary))+struct.pack('<I4s',len(encoded),b'JSON')+encoded+struct.pack('<I4s',len(binary),b'BIN\0')+binary
    source=tmp_path/'source.glb';target=tmp_path/'warped.glb';source.write_bytes(raw)
    def move(p):
        result=p.copy();result[:,2]+=.25;return result
    warp_glb(source,target,move)
    after=target.read_bytes();length=struct.unpack_from('<I',after,12)[0];payload=after[28+length:]
    values=np.frombuffer(payload[:positions.nbytes],dtype='<f4').reshape(-1,3)
    assert np.array_equal(values[:,:2],positions[:,:2])
    assert np.array_equal(values[:,2],positions[:,2]+.25)
    assert payload[positions.nbytes:]==suffix
    assert source.read_bytes()==raw
