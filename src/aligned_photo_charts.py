"""Bounded 2D photo alignment on a fixed wall, with recorded feature evidence."""
import cv2
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image

from coherent_photo_charts import coherent_labels, flatten_photo_lighting
from reconstruct_continuous_planes import sample_view, solve_log_gains
from surface_model_utils import project
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb


def align_photos(images, weights, pixel_m):
    """SIFT correspondences -> robust translation graph, limited to 25cm.

    This adjusts only the 2D texture coordinates, never map geometry or poses.
    Require distributed inliers and retain the original depth-valid support.
    """
    n=len(images);h,w=weights[0].shape;scale=min(1.,768/max(h,w))
    size=(max(1,round(w*scale)),max(1,round(h*scale)))
    sift=cv2.SIFT_create(nfeatures=1200,contrastThreshold=.015)
    features=[]
    for im,weight in zip(images,weights):
        rgb=np.uint8(np.clip(linear_to_srgb(np.clip(im,0,1))*255,0,255))
        gray=cv2.cvtColor(cv2.resize(rgb,size),cv2.COLOR_RGB2GRAY)
        gray=cv2.createCLAHE(2.,(8,8)).apply(gray)
        mask=cv2.resize((weight>0).astype(np.uint8)*255,size,interpolation=cv2.INTER_NEAREST)
        mask=cv2.erode(mask,np.ones((3,3),np.uint8))
        kp,desc=sift.detectAndCompute(gray,mask)
        features.append((np.array([k.pt for k in kp]).reshape(-1,2),desc,mask>0))
    matcher=cv2.BFMatcher();edges=[]
    tolerance=max(1.5,.035*scale/pixel_m)
    pairs_to_try=set()
    for i in range(n):
        overlaps=sorted(((int((features[i][2]&features[j][2]).sum()),j) for j in range(n) if j!=i),reverse=True)
        for count,j in overlaps[:6]:
            if count>=400:pairs_to_try.add((max(i,j),min(i,j)))
    for i in range(n):
        pi,di,mi=features[i]
        if di is None or len(di)<12:continue
        for j in range(i):
            if (i,j) not in pairs_to_try:continue
            pj,dj,mj=features[j]
            if dj is None or len(dj)<12 or (mi & mj).sum()<400:continue
            pairs=matcher.knnMatch(di,dj,k=2)
            good=[pair[0] for pair in pairs if len(pair)==2 and pair[0].distance<.72*pair[1].distance]
            if len(good)<12:continue
            x=np.array([pi[m.queryIdx] for m in good]);y=np.array([pj[m.trainIdx] for m in good]);delta=y-x
            # Translation RANSAC: fit the modal common offset, then its median.
            residual=np.linalg.norm(delta[:,None]-delta[None,:],axis=2)
            seed=np.argmax((residual<tolerance).sum(1));inlier=residual[seed]<tolerance
            shift=np.median(delta[inlier],axis=0);inlier=np.linalg.norm(delta-shift,axis=1)<tolerance
            if inlier.sum()<12 or inlier.mean()<.5:continue
            span=np.ptp(x[inlier],axis=0)*pixel_m/scale
            if min(span)<.20 or np.linalg.norm(shift)*pixel_m/scale>.25:continue
            edges.append({'i':i,'j':j,'shift_pixels':(shift/scale).tolist(),'inliers':int(inlier.sum()),
                'matches':len(good),'span_m':span.tolist(),'fit_median_px':float(np.median(np.linalg.norm(delta[inlier]-shift,axis=1))/scale)})
    shifts=np.zeros((n,2));anchor=int(np.argmax([np.count_nonzero(v) for v in weights]))
    if edges:
        rows=[];values=[];base=[]
        for e in edges:
            row=np.zeros(n);row[e['i']]=1;row[e['j']]=-1;rows.append(row);values.append(e['shift_pixels']);base.append(min(e['inliers'],100)/20)
        mat=np.array(rows);target=np.array(values);base=np.array(base);robust=base.copy()
        for _ in range(8):
            root=np.sqrt(robust);anchor_row=np.zeros((1,n));anchor_row[0,anchor]=20
            a=np.vstack([mat*root[:,None],np.eye(n)*.05,anchor_row]);b=np.vstack([target*root[:,None],np.zeros((n+1,2))])
            shifts=np.linalg.lstsq(a,b,rcond=None)[0]
            error=np.linalg.norm(mat@shifts-target,axis=1);robust=base*np.minimum(1.,2/np.maximum(error,1e-9))
        over=np.linalg.norm(shifts,axis=1)*pixel_m>.25;shifts[over]=0.
    aligned=[];aligned_weights=[]
    for im,weight,shift in zip(images,weights,shifts):
        matrix=np.array([[1.,0.,shift[0]],[0.,1.,shift[1]]],np.float32)
        aligned.append(cv2.warpAffine(im,matrix,(w,h),flags=cv2.INTER_LINEAR))
        aligned_weights.append(cv2.warpAffine(weight,matrix,(w,h),flags=cv2.INTER_LINEAR)*(weight>0))
    return aligned,aligned_weights,{'method':'SIFT ratio matches + translation RANSAC + Huber IRLS graph',
        'anchor_index':anchor,'accepted_edges':edges,'offsets_pixels':shifts.tolist(),
        'maximum_offset_m':float(np.linalg.norm(shifts,axis=1).max()*pixel_m),'maximum_allowed_offset_m':.25,
        'geometry_and_camera_poses_changed':False}


def texture_plane(plane,mask,frames,nodes,reference,dest,index):
    h,w=mask.shape;tw=min(2048,max(2,int(w*.06/.01)));th=min(2048,max(2,int(h*.06/.01)))
    pixel_m=max(w*.06/tw,h*.06/th)
    yy,xx=np.indices((th,tw));uv=np.c_[(xx.ravel()+.5)*w*.06/tw,(yy.ravel()+.5)*h*.06/th]+plane['low']
    points=plane['centre']+uv@plane['basis'][:,:2].T;images=[];weights=[]
    for node in nodes:
        f=frames[node];valid,score=sample_view(f,points,plane['basis'][:,2])
        pixels,_=project(points,f['k_rgb'],f['world_optical']);maps=pixels.reshape(th,tw,2).astype(np.float32)
        rgb=cv2.remap(f['rgb'],maps[:,:,0],maps[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)/255.
        images.append(srgb_to_linear(rgb).astype(np.float32));weights.append(score.reshape(th,tw).astype(np.float32))
    images,weights,alignment=align_photos(images,weights,pixel_m)
    step=max(1,int(max(tw,th)/256));gains=solve_log_gains([v[::step,::step] for v in images],[v[::step,::step] for v in weights])
    tree=cKDTree(np.asarray(reference.points));_,near=tree.query(points,workers=2)
    result=srgb_to_linear(np.asarray(reference.colors)[near]).reshape(th,tw,3).astype(np.float32)
    stack=np.asarray(weights);covered=stack.max(0)>0;labels=coherent_labels(stack,step=8,potts=.6,iterations=10)
    total=np.zeros((th,tw),np.float32);low=np.zeros((th,tw,3),np.float32);detail=np.zeros_like(low)
    for i,(im,weight,gain) in enumerate(zip(images,weights,gains)):
        fixed=im*gain;smooth=cv2.GaussianBlur(fixed,(0,0),max(4.,.12/pixel_m))
        region=cv2.GaussianBlur((labels==i).astype(np.float32),(0,0),max(5.,.20/pixel_m))*(weight>0)
        low+=smooth*region[:,:,None];total+=region
        chosen=(labels==i)&covered;detail[chosen]=(fixed-smooth)[chosen]
    result[covered]=(low/np.maximum(total[:,:,None],1e-8)+detail)[covered]
    result,lighting=flatten_photo_lighting(result,covered,pixel_m)
    path=dest/f'continuous_{index:03d}.png';Image.fromarray(np.uint8(linear_to_srgb(np.clip(result,0,1))*255+.5)).save(path)
    mask_tex=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0
    coverage_path=dest/f'coverage_{index:03d}.png';Image.fromarray(np.uint8(np.where(mask_tex,np.where(covered,255,80),0))).save(coverage_path)
    Image.fromarray(labels.astype(np.uint8)).save(dest/f'photo_labels_{index:03d}.png')
    return path,{'photo_coverage_on_surface_fraction':float(covered[mask_tex].mean()),'texture_shape':[tw,th],
        'gains':{str(n):g.tolist() for n,g in zip(nodes,gains)},'coherent_labels':'Potts ICM, 10 iterations, 8px grid, penalty 0.6',
        'lighting_leveling':lighting,'texture_alignment':alignment,
        'single_source_detail_radius_m':.12,'low_frequency_feather_radius_m':.20,
        'fallback':'selected_cloud_RGB_only_at_uncovered_texels','coverage_image':coverage_path.name}
