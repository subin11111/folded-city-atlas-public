"""Measured photo-label boundary leveling and embedded GLB image replacement."""
import hashlib
import json
from pathlib import Path
import struct
import cv2
import numpy as np
from scipy import ndimage
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb


def level_boundaries(rgb,labels,valid):
    linear=srgb_to_linear(rgb/255.).astype(np.float32)
    lum=np.maximum(linear@np.array([.2126,.7152,.0722]),.008)
    log=np.log(lum);ids=np.unique(labels[valid]);lookup={int(v):i for i,v in enumerate(ids)}
    # Find low-texture interiors on both sides of a source-photo boundary.
    local=cv2.GaussianBlur(log,(0,0),1.5)
    rows=[];values=[];counts=[];records=[];samples=[]
    for axis in [0,1]:
        aa=(slice(None,-1),slice(None)) if axis==0 else (slice(None),slice(None,-1))
        bb=(slice(1,None),slice(None)) if axis==0 else (slice(None),slice(1,None))
        la,lb=labels[aa],labels[bb];ok=valid[aa]&valid[bb]&(la!=lb)
        # Real very dark objects and saturated highlights are not brightness references.
        ok&=(lum[aa]>.025)&(lum[aa]<.8)&(lum[bb]>.025)&(lum[bb]<.8)
        ok&=(labels[aa]==np.roll(labels,1,axis)[aa])&(labels[bb]==np.roll(labels,-1,axis)[bb])
        ok&=(abs(log[aa]-np.roll(log,1,axis)[aa])<.10)&(abs(log[bb]-np.roll(log,-1,axis)[bb])<.10)
        pairs=np.stack([np.minimum(la,lb),np.maximum(la,lb)],axis=-1)
        for a,b in np.unique(pairs[ok],axis=0):
            chosen=ok&(pairs[:,:,0]==a)&(pairs[:,:,1]==b)
            if chosen.sum()<24:continue
            va=local[aa][chosen];vb=local[bb][chosen]
            sign=np.where(la[chosen]==a,1.,-1.)
            difference=(va-vb)*sign
            # Require a coherent brightness discontinuity, not alternating lettering.
            median=float(np.median(difference));mad=float(np.median(abs(difference-median)))
            if mad>.18 or abs(median)>.8:continue
            row=np.zeros(len(ids));row[lookup[int(a)]]=1;row[lookup[int(b)]]=-1
            rows.append(row);values.append(-median);counts.append(min(int(chosen.sum()),300)/100)
            records.append({'labels':[int(a),int(b)],'samples':int(chosen.sum()),'before_log_luma_jump':median,'mad':mad})
            samples.append((axis,np.flatnonzero(chosen.ravel()),sign))
    gains=np.ones(len(ids))
    if rows:
        matrix=np.asarray(rows);target=np.asarray(values);weight=np.sqrt(counts)
        anchor=int(np.argmax([np.sum(valid&(labels==v)) for v in ids]));anchor_row=np.zeros((1,len(ids)));anchor_row[0,anchor]=5
        a=np.vstack([matrix*weight[:,None],np.eye(len(ids))*.35,anchor_row]);b=np.r_[target*weight,np.zeros(len(ids)+1)]
        offset=np.linalg.lstsq(a,b,rcond=None)[0];gains=np.exp(np.clip(offset,np.log(.6),np.log(1.6)))
        for r in records:
            i,j=r['labels'];r['after_log_luma_jump']=r['before_log_luma_jump']+float(np.log(gains[lookup[i]])-np.log(gains[lookup[j]]))
    field=np.ones(valid.shape,np.float32)
    for label,gain in zip(ids,gains):field[valid&(labels==label)]=gain
    # Soften the applied correction itself without averaging measured letters.
    result=linear.copy();result[valid]*=field[valid,None]
    actual_log=np.log(np.maximum(np.clip(result,0,1)@np.array([.2126,.7152,.0722]),.008))
    actual_log=cv2.GaussianBlur(actual_log,(0,0),1.5)
    for record,(axis,index,sign) in zip(records,samples):
        aa=(slice(None,-1),slice(None)) if axis==0 else (slice(None),slice(None,-1))
        bb=(slice(1,None),slice(None)) if axis==0 else (slice(None),slice(1,None))
        record['predicted_after_log_luma_jump']=record.get('after_log_luma_jump')
        record['after_log_luma_jump']=float(np.median((actual_log[aa].ravel()[index]-actual_log[bb].ravel()[index])*sign))
    before=float(np.median([abs(r['before_log_luma_jump']) for r in records])) if records else None
    after=float(np.median([abs(r['after_log_luma_jump']) for r in records])) if records else None
    accepted=bool(records and after<=before)
    if not accepted:result=linear
    return np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5),{
        'method':'Robust adjacent photo-label log-luminance graph; bounded gain; original RGB detail retained',
        'gain_bounds':[.6,1.6],'gains':{str(int(i)):float(g) for i,g in zip(ids,gains)},'boundaries':records,
        'applied':accepted,'candidate_median_absolute_boundary_jump_after':after,
        'median_absolute_boundary_jump_before':before,
        'median_absolute_boundary_jump_after':after if accepted else before,
        'scope':'Self-consistency on the same selected boundary samples; not ground-truth illumination.'}


def replace_glb_images(source,target,replacements):
    """Match original PNG bytes by hash; append replacements without changing geometry."""
    raw=Path(source).read_bytes();length=struct.unpack_from('<I',raw,12)[0]
    doc=json.loads(raw[20:20+length]);payload=bytearray(raw[28+length:]);count=0
    for image in doc.get('images',[]):
        if 'bufferView' not in image:continue
        view=doc['bufferViews'][image['bufferView']];start=view.get('byteOffset',0);blob=bytes(payload[start:start+view['byteLength']])
        key=hashlib.sha256(blob).hexdigest()
        if key not in replacements:continue
        data=Path(replacements[key]).read_bytes();payload.extend(b'\0'*(-len(payload)%4));start=len(payload);payload.extend(data)
        doc['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':len(data)});image['bufferView']=len(doc['bufferViews'])-1;count+=1
    doc['buffers'][0]['byteLength']=len(payload);encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with Path(target).open('wb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(payload)));f.write(struct.pack('<I4s',len(encoded),b'JSON'));f.write(encoded)
        f.write(struct.pack('<I4s',len(payload),b'BIN\0'));f.write(payload)
    return count
