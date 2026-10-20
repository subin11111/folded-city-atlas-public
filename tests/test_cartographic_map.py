import numpy as np

from build_cartographic_map import mask_rings, to_geo_polygons


def test_floor_polygon_preserves_atrium_hole():
    mask=np.zeros((40,40),np.uint8);mask[3:37,3:37]=1;mask[14:25,14:25]=0
    rings=mask_rings(mask,[0,0],.1,simplify=.02)
    assert len(rings)==2 and sum(r['hole'] for r in rings)==1
    geo=to_geo_polygons(rings,lambda xy:xy)
    assert len(geo)==1 and len(geo[0])==2


def test_empty_floor_is_not_invented():
    assert mask_rings(np.zeros((10,10),np.uint8),[0,0],.1)==[]
