import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from reconstruct_continuous_planes import continuous_mask,compatible,group_patches,solve_log_gains
from coherent_photo_charts import coherent_labels,flatten_photo_lighting


def patch(x=0,z=0,angle=0):
    a=np.deg2rad(angle);basis=np.array([[np.cos(a),0,-np.sin(a)],[np.sin(a),0,np.cos(a)],[0,-1,0.]])
    return {'centre':[x,z,0.],'basis':basis.tolist(),'shape':[30,30],'low':[0,0],'cells':900}


def test_spatially_adjacent_coplanar_fragments_merge():
    a,b=patch(),patch(2.)
    assert compatible(a,b)
    assert len(group_patches([a,b]))==1


def test_parallel_other_wall_and_perpendicular_corner_not_merged():
    assert not compatible(patch(),patch(z=.4))
    assert not compatible(patch(),patch(angle=90))
    assert not compatible(patch(),patch(8.))


def test_complete_link_rejects_transitive_offset_drift():
    assert len(group_patches([patch(z=0),patch(z=.1),patch(z=.2)]))==2


def test_depth_supported_hole_fills_but_unknown_window_stays():
    observed=np.ones((80,80),bool);observed[15:30,15:30]=False;observed[45:60,45:60]=False
    support=np.zeros_like(observed,int);support[15:30,15:30]=3
    mask,report=continuous_mask(observed,support,np.zeros_like(support))
    assert mask[16:29,16:29].all()
    assert not mask[47:58,47:58].any()
    assert report['new_supported_cells']>0 and report['unknown_gap_cells_preserved']>0


def test_open_door_free_space_veto_survives_completion():
    observed=np.ones((80,80),bool);observed[35:,25:45]=False
    support=np.full(observed.shape,2);free=np.zeros_like(support);free[35:,25:45]=8
    mask,report=continuous_mask(observed,support,free)
    assert not mask[40:,28:42].any()
    assert report['free_space_cells']>0


def test_exposure_gains_align_overlapping_photos_without_cloud_color():
    a=np.ones((20,20,3))*.2;b=a*1.5
    gains=solve_log_gains([a,b],[np.ones((20,20)),np.ones((20,20))])
    assert np.max(abs(a*gains[0]-b*gains[1]))<.005
    assert (gains>=.5).all() and (gains<=2.).all()


def test_coherent_labels_never_choose_occluded_camera():
    weights=np.ones((2,40,40),np.float32);weights[0,:,20:]=0;weights[1,:,:20]=0
    label=coherent_labels(weights)
    assert (label[:,:20]==0).all() and (label[:,20:]==1).all()


def test_lighting_leveling_does_not_change_uncovered_texels():
    im=np.ones((40,40,3))*.04;mask=np.zeros((40,40),bool);mask[10:30,10:30]=True
    result,report=flatten_photo_lighting(im,mask)
    np.testing.assert_array_equal(result[~mask],im[~mask])
    assert result[mask].mean()>im[mask].mean() and report['global_exposure_gain']<=2.5


def test_photo_alignment_recovers_known_translation_without_stretching():
    import cv2
    from aligned_photo_charts import align_photos
    rng=np.random.default_rng(41)
    image=cv2.GaussianBlur(rng.uniform(.05,.6,(240,240,3)).astype(np.float32),(0,0),.7)
    matrix=np.array([[1,0,6],[0,1,-4]],np.float32)
    moved=cv2.warpAffine(image,matrix,(240,240))
    weights=[np.ones((240,240),np.float32),cv2.warpAffine(np.ones((240,240),np.float32),matrix,(240,240))]
    aligned,newweights,report=align_photos([image,moved],weights,.01)
    assert report['accepted_edges']
    np.testing.assert_allclose(report['offsets_pixels'][1],[-6,4],atol=.5)
    assert report['maximum_offset_m']<=.25 and not report['geometry_and_camera_poses_changed']
    valid=(newweights[0]>0)&(newweights[1]>0)
    assert np.mean(abs(aligned[0][valid]-aligned[1][valid]))<.01
