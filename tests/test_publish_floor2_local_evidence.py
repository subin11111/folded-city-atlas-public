import sys
from io import BytesIO
from pathlib import Path

import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from publish_floor2_local_evidence import image_item, prepare_images, publish


def test_lossless_package_preserves_rgba(tmp_path):
    path = tmp_path/"actual_test_fixture.png"
    image = Image.new("RGBA", (8, 9), (11, 29, 37, 123))
    image.putpixel((3, 4), (9, 19, 23, 0))
    image.save(path)
    row = image_item(path, "packaged.png", "test", "fixture", "unit fixture only")
    prepared, total = prepare_images([row], 1.)
    name, data, metadata = prepared[0]
    assert name == "packaged.png" and total == len(data)
    assert metadata["decoded_rgba_identical"] is True
    assert metadata["dimensions_pixels"] == [8, 9]
    with Image.open(BytesIO(data)) as decoded:
        assert decoded.convert("RGBA").tobytes() == image.tobytes()


def test_duplicate_and_unsafe_destinations_rejected(tmp_path):
    path = tmp_path/"fixture.png"
    Image.new("RGB", (2, 2)).save(path)
    row = image_item(path, "safe.png", "test", "fixture", "test")
    with pytest.raises(ValueError, match="Unsafe or duplicate"):
        prepare_images([row, row], 1.)
    with pytest.raises(ValueError, match="Unsafe or duplicate"):
        prepare_images([{**row, "destination": "../escape.png"}], 1.)


def test_budget_checked_before_publication(tmp_path):
    path = tmp_path/"fixture.png"
    Image.new("RGB", (20, 20), (11, 22, 33)).save(path)
    with pytest.raises(ValueError, match="budget"):
        prepare_images([image_item(path, "safe.png", "test", "fixture", "test")], .000001)


def test_existing_output_never_overwritten(tmp_path):
    output = tmp_path/"gallery"
    output.mkdir()
    with pytest.raises(FileExistsError, match="fresh output"):
        publish(tmp_path/"run", output)


def test_incomplete_input_does_not_create_gallery(tmp_path):
    output = tmp_path/"gallery"
    with pytest.raises(FileNotFoundError):
        publish(tmp_path/"missing_run", output)
    assert not output.exists()
