"""Spatially coherent photo labels and bounded low-frequency exposure leveling."""
import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from PIL import Image

from surface_model_utils import project
from reconstruct_continuous_planes import sample_view,solve_log_gains
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb


def coherent_labels(weights,step=4,potts=.35,iterations=6):
    """Approximate Potts labeling via deterministic ICM, NOT graph-cut optimality."""
    weights=np.asarray(weights,np.float32)
    coarse=weights[:,::step,::step].copy()
    coarse/=np.maximum(coarse.max(axis=0,keepdims=True),1e-8)
    labels=coarse.argmax(0);valid=coarse.max(0)>0
    for _ in range(iterations):
        best=np.full(labels.shape,np.inf);chosen=labels.copy()
        # Synchronous ICM with alternating checkerboards avoids flip-flops.
        for parity in [0,1]:
            yy,xx=np.indices(labels.shape);update=((xx+yy)%2==parity)&valid
            for i in range(len(coarse)):
                cost=-coarse[i]+potts*ndimage.convolve((labels!=i).astype(np.float32),np.array([[0,1,0],[1,0,1],[0,1,0]],np.float32),mode='nearest')
                cost[coarse[i]<=0]=1e6
                improve=cost<best;chosen[improve]=i;best[improve]=cost[improve]
            labels[update]=chosen[update];best[:]=np.inf
    h,w=weights.shape[1:];labels=cv2.resize(labels.astype(np.float32),(w,h),interpolation=cv2.INTER_NEAREST).astype(int)
    # Coarse labels can land on an occluded fine pixel; use valid best view there.
    fine=weights.argmax(0);rows,cols=np.indices((h,w));invalid=weights[labels,rows,cols]<=0;labels[invalid]=fine[invalid]
    return labels


def flatten_photo_lighting(image,covered,pixel_m=.01):
    """Bounded luminance-only leveling. Keeps measured high-frequency RGB detail."""
    luminance=np.maximum(image@np.array([.2126,.7152,.0722]),.002)
    mask=covered.astype(np.float32);sigma=max(2.,.35/pixel_m)
    low=cv2.GaussianBlur(luminance*mask,(0,0),sigma)/np.maximum(cv2.GaussianBlur(mask,(0,0),sigma),1e-6)
    target=float(np.median(low[covered])) if covered.any() else .1
    # Exposure lift is global to a chart, not a replacement of real dark signs.
    global_gain=float(np.clip(.22/max(target,.005),1.,2.5))
    gain=np.clip(max(target,.005)/np.maximum(low,.005),.65,1.55)*global_gain
    result=image.copy();result[covered]*=gain[covered,None]
    return result,{'low_frequency_radius_m':.35,'local_gain_bounds':[.65,1.55],'global_exposure_gain':global_gain}


def texture_plane(plane,mask,frames,nodes,reference,dest,index):
    h,w=mask.shape;tw=min(2048,max(2,int(w*.06/.01)));th=min(2048,max(2,int(h*.06/.01)))
    yy,xx=np.indices((th,tw));uv=np.c_[(xx.ravel()+.5)*w*.06/tw,(yy.ravel()+.5)*h*.06/th]+plane['low']
    points=plane['centre']+uv@plane['basis'][:,:2].T;images=[];weights=[]
    for node in nodes:
        f=frames[node];valid,score=sample_view(f,points,plane['basis'][:,2])
        pixels,_=project(points,f['k_rgb'],f['world_optical']);maps=pixels.reshape(th,tw,2).astype(np.float32)
        rgb=cv2.remap(f['rgb'],maps[:,:,0],maps[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)/255.
        images.append(srgb_to_linear(rgb).astype(np.float32));weights.append(score.reshape(th,tw).astype(np.float32))
    step=max(1,int(max(tw,th)/256));gains=solve_log_gains([v[::step,::step] for v in images],[v[::step,::step] for v in weights])
    tree=cKDTree(np.asarray(reference.points));_,near=tree.query(points,workers=2)
    result=srgb_to_linear(np.asarray(reference.colors)[near]).reshape(th,tw,3).astype(np.float32)
    stack=np.asarray(weights);covered=stack.max(0)>0;labels=coherent_labels(stack)
    total=np.zeros((th,tw),np.float32);low=np.zeros((th,tw,3),np.float32);detail=np.zeros_like(low)
    for i,(im,weight,gain) in enumerate(zip(images,weights,gains)):
        fixed=im*gain;smooth=cv2.GaussianBlur(fixed,(0,0),4)
        # Feather only near the coherent label's region, not every valid view.
        region=cv2.GaussianBlur((labels==i).astype(np.float32),(0,0),4)*(weight>0)
        low+=smooth*region[:,:,None];total+=region
        chosen=(labels==i)&covered;detail[chosen]=(fixed-smooth)[chosen]
    result[covered]=(low/np.maximum(total[:,:,None],1e-8)+detail)[covered]
    result,lighting=flatten_photo_lighting(result,covered,max(w*.06/tw,h*.06/th))
    path=dest/f'continuous_{index:03d}.png';Image.fromarray(np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5)).save(path)
    mask_tex=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0
    coverage_path=dest/f'coverage_{index:03d}.png';Image.fromarray(np.uint8(np.where(mask_tex,np.where(covered,255,80),0))).save(coverage_path)
    labelpath=dest/f'photo_labels_{index:03d}.png';Image.fromarray(labels.astype(np.uint8)).save(labelpath)
    return path,{'photo_coverage_on_surface_fraction':float(covered[mask_tex].mean()),'texture_shape':[tw,th],
        'gains':{str(n):g.tolist() for n,g in zip(nodes,gains)},'coherent_labels':'Potts ICM, 6 iterations, 4px grid; not graph cuts',
        'lighting_leveling':lighting,'fallback':'selected_cloud_RGB_only_at_uncovered_texels','coverage_image':coverage_path.name}
