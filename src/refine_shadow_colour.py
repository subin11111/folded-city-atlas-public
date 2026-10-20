"""Conservative shadow correction: local noise control, then weak hue-retaining gain."""
import cv2
import numpy as np

from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

LUMA = np.array([.2126, .7152, .0722], np.float32)


def colour_shadow_lift(linear_rgb, cfg):
    rgb = np.asarray(linear_rgb, np.float32)
    if not np.isfinite(rgb).all() or rgb.min(initial=0) < 0 or rgb.max(initial=0) > 1.00001:
        raise ValueError('Expected finite linear RGB in [0,1]')
    y = np.clip(rgb @ LUMA, 0, 1)
    shadow = np.clip(1-y/cfg['shadow_end_linear'], 0, 1)**2
    reliability = y/(y+cfg['noise_floor_linear'])
    gain = np.minimum(1+cfg['maximum_extra_gain']*shadow*reliability,
                      cfg['maximum_linear_gain'])
    # A common gain, including the gamut limit, preserves RGB ratios. No grey is added.
    gain = np.minimum(gain, 1/np.maximum(rgb.max(axis=-1), 1e-8))
    return np.clip(rgb*gain[..., None], 0, 1)


def denoise_shadow_rgb(rgb8, alpha, cfg):
    """5px bilateral support, only flat dark interiors, bounded to six 8-bit steps.

    Atlas/alpha boundaries and pure black are protected rather than inpainted.
    This is not multi-view noise recovery: some real fine detail can be attenuated.
    """
    rgb8 = np.asarray(rgb8, np.uint8)
    rgb = rgb8.astype(np.float32)
    valid = (np.asarray(alpha) > 0) & np.any(rgb8 != 0, axis=-1)
    interior = cv2.erode(valid.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    y = srgb_to_linear(rgb/255.) @ LUMA
    shadow = np.clip(1-y/cfg['denoise_end_linear'], 0, 1)**2
    guide = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    gx = cv2.Sobel(guide, cv2.CV_32F, 1, 0, ksize=3, scale=1/8)
    gy = cv2.Sobel(guide, cv2.CV_32F, 0, 1, ksize=3, scale=1/8)
    edge_keep = np.clip(1-np.hypot(gx, gy)/cfg['edge_protection_srgb_steps'], 0, 1)**2
    weight = (shadow*edge_keep*interior)[..., None]
    colour = cv2.cvtColor(rgb/255., cv2.COLOR_RGB2YCrCb)
    filtered = colour.copy()
    for channel, sigma, strength in [(0, 4., cfg['luma_denoise_strength']),
                                      (1, 9., cfg['chroma_denoise_strength']),
                                      (2, 9., cfg['chroma_denoise_strength'])]:
        plane = np.ascontiguousarray(colour[:, :, channel])
        smooth = cv2.bilateralFilter(plane, 5, sigma/255., 1.2,
                                     borderType=cv2.BORDER_REFLECT_101)
        filtered[:, :, channel] += strength*(smooth-plane)
    smooth_rgb = cv2.cvtColor(filtered, cv2.COLOR_YCrCb2RGB)*255.
    delta = np.clip(smooth_rgb-rgb, -cfg['maximum_denoise_srgb_steps'],
                    cfg['maximum_denoise_srgb_steps'])
    return np.clip(rgb+weight*delta, 0, 255)


def correct_shadow_texture(rgb8, alpha, cfg):
    clean = denoise_shadow_rgb(rgb8, alpha, cfg)
    mapped = colour_shadow_lift(srgb_to_linear(clean/255.), cfg)
    result = np.uint8(np.clip(linear_to_srgb(mapped)*255+.5, 0, 255))
    # Avoid any colour-space round-trip changes to untouched highlights or absent texels.
    y = srgb_to_linear(np.asarray(rgb8)/255.) @ LUMA
    unchanged = (y >= cfg['shadow_end_linear']) | (np.asarray(alpha) == 0) | ~np.any(rgb8, axis=-1)
    result[unchanged] = np.asarray(rgb8)[unchanged]
    return result
