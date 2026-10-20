import numpy as np
from intrinsic_floor_material import photographic_albedo,reconstruct_floor_material
from refine_surface_display import srgb_to_linear


def test_diffuse_floor_retains_black_accents_grain_and_outside_pixels():
    y,x=np.indices((80,100));rgb=np.full((80,100,3),150,np.uint8);rgb[:,:,1]=140;rgb[:,:,2]=120;rgb[20:30,20:30]=20;rgb[40:60,50:80,0]+=(8*(x[40:60,50:80]%2)).astype(np.uint8);mask=np.ones((80,100),bool);mask[:5]=False
    cfg={'illumination_radius_m':.16,'target_linear_luminance':.2,'gain_bounds':[.05,512.],'radiometric_floor':1e-5,'low_light_denoising':True,'chromaticity_normalization':True,'reference_chromaticity':[1.1,1.,.8]}
    out=photographic_albedo(rgb,mask,.02,np.zeros((8,8,3),np.float32),cfg)
    assert np.max(abs(out[:5].astype(int)-rgb[:5].astype(int)))<=1
    assert out[25,25].mean()<out[65,65].mean()*.6
    assert np.ptp(out[45,55:75,0])>4


def test_underexposed_floor_is_not_left_dark_by_an_arbitrary_luma_clamp():
    rgb=np.full((80,100,3),5,np.uint8);rgb[35:43,45:53]=0;mask=np.ones((80,100),bool)
    cfg={'illumination_radius_m':.16,'target_linear_luminance':.2,'gain_bounds':[.05,512.],'radiometric_floor':1e-5,'low_light_denoising':True,'chromaticity_normalization':True,'reference_chromaticity':[1.1,1.,.8]}
    out=photographic_albedo(rgb,mask,.02,np.zeros((8,8,3),np.float32),cfg)
    assert out[20:30,20:30].mean()>100
    assert out[38:40,48:50].mean()<out[20:30,20:30].mean()*.7


def test_modeled_material_keeps_observed_dark_accents_and_joint_bands():
    # The accent is a finite component, separate from the long joint. Components
    # joined to a broad dark band are intentionally not promoted to black tiles.
    rng=np.random.default_rng(8);rgb=np.full((100,120,3),160,np.uint8);rgb[45:51]=100;rgb[28:38,65:75]=15;rgb[70:90,20:50]=rng.integers(135,170,(20,30,3),dtype=np.uint8);mask=np.ones((100,120),bool);mask[:4]=False
    cfg={'reference_chromaticity':[1.1,1.,.8],'target_linear_luminance':.2,'preserve_joint_bands':True,'detail_radius_m':.16,'contrast_bounds':[.65,1.25]}
    out=reconstruct_floor_material(rgb,mask,.02,np.zeros((8,8,3),np.float32),cfg)
    assert out[32,70].mean()<out[60,90].mean()*.6
    assert out[47,30].mean()<out[30,30].mean()*.95
    assert np.ptp(out[75,20:50,0])>5
    assert np.max(abs(out[:4].astype(int)-rgb[:4].astype(int)))<=1
