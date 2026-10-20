"""Stronger indoor structural priors with explicit finite-boundary protection."""
import numpy as np
from scipy import ndimage


def rectangular_openings(occupied,cell):
    holes=ndimage.binary_fill_holes(occupied)&~occupied
    labels,count=ndimage.label(holes);protected=np.zeros_like(occupied)
    for label,sl in enumerate(ndimage.find_objects(labels),1):
        if sl is None:continue
        region=labels[sl]==label;h,w=region.shape;area=region.sum()*cell*cell
        # Unknown rectangular windows/doors are protected even with no depth.
        if area>=.20 and min(h,w)*cell>=.45 and region.mean()>=.60:
            protected[sl]|=region
    return protected


def strong_wall_mask(occupied,cell=.06):
    from rebuild_structural_display import bounded_mask
    protected=rectangular_openings(occupied,cell)
    candidate=bounded_mask(occupied,radius=6,maximum_hole_area=.9,cell=cell)
    candidate[protected]=False
    return candidate,protected


def regularize_height_planes(height,mask,cell=.1,strength=.95,maximum_motion=.18):
    """Robust affine plane per large component; preserve slope and boundary height."""
    labels,count=ndimage.label(mask);sizes=np.bincount(labels.ravel());out=height.copy();report=[]
    for label in range(1,count+1):
        if sizes[label]*cell*cell<5.:continue
        region=labels==label;y,x=np.nonzero(region);xy=np.c_[x*cell,y*cell];origin=xy.mean(0)
        if np.min(np.ptp(xy,axis=0))<.5:continue
        a=np.c_[xy-origin,np.ones(len(x))];z=height[y,x];weight=np.ones(len(x))
        for _ in range(10):
            root=np.sqrt(weight);coef=np.linalg.lstsq(a*root[:,None],z*root,rcond=None)[0]
            residual=a@coef-z;weight=np.minimum(1.,.04/np.maximum(abs(residual),1e-9))
        p90=float(np.percentile(abs(residual),90));slope=float(np.rad2deg(np.arctan(np.linalg.norm(coef[:2]))))
        accepted=p90<=.12 and slope<=8.
        row={'area_xy_m2':float(sizes[label]*cell*cell),'fit_residual_p90_m':p90,'slope_deg':slope,'accepted':bool(accepted)}
        if accepted:
            taper=np.clip(ndimage.distance_transform_edt(region)[y,x]*cell/.5,0,1)
            delta=np.clip((a@coef-z)*strength*taper,-maximum_motion,maximum_motion)
            out[y,x]=z+delta
            row.update(maximum_height_change_m=float(abs(delta).max()),
                median_plane_distance_before_m=float(np.median(abs(residual))),
                median_plane_distance_after_m=float(np.median(abs(out[y,x]-a@coef))))
        report.append(row)
    return out,report
