from pathlib import Path
import sys

from PIL import Image,ImageFont
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from save_pointcloud_surface_progression import contact_sheet


def test_contact_sheet_preserves_stage_pixels_without_resizing():
    images=[Image.new('RGB',(30,40),colour) for colour in ['red','green','blue']]
    result=contact_sheet(images,['points','mesh','texture'],'fixed camera',ImageFont.load_default())
    assert result.size==(90,130)
    for i,im in enumerate(images):
        assert result.crop((i*30,90,(i+1)*30,130)).tobytes()==im.tobytes()


def test_comparison_rejects_mismatched_stage_resolutions():
    with pytest.raises(ValueError,match='Matched image sizes'):
        contact_sheet([Image.new('RGB',(30,40)),Image.new('RGB',(31,40))],
            ['points','mesh'],'fixed camera',ImageFont.load_default())
