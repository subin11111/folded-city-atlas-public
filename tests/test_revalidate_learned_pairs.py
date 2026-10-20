import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from revalidate_learned_pairs import resized_shape, restore_pixels, spatial_thin, select_candidates, verify_pixels
from revalidate_loop_constraints import split_correspondences


def test_resize_mapping_preserves_independent_axes():
    shape = (1281, 963, 3)
    wh = resized_shape(shape)
    assert max(wh) <= 480 and all(x % 8 == 0 for x in wh)
    point = np.array([[wh[0] / 2, wh[1] / 2]])
    np.testing.assert_allclose(restore_pixels(point, shape, wh), [[963 / 2, 1281 / 2]])


def test_thinning_both_depth_images_keeps_strongest():
    a = np.array([[1, 1], [2, 2], [10, 10], [20, 20]])
    b = np.array([[1, 1], [10, 10], [11, 11], [30, 30]])
    selected = spatial_thin(a, b, [.1, .9, .6, .8])
    assert selected.tolist() == [1, 3]


def test_dense_match_holdout_neighborhood_never_leaks():
    pixels = np.array([[x, y] for x in range(4, 95, 6) for y in range(4, 95, 6)])
    train, held, masks = split_correspondences(pixels, pixels, (100, 100), (100, 100), 3)
    assert len(train) >= 12 and len(held) >= 5
    assert not set(train) & set(held)
    assert not masks[0][pixels[train, 1], pixels[train, 0]].any()
    assert masks[0][pixels[held, 1], pixels[held, 0]].all()


def test_candidate_selection_bounded_unique_and_deterministic():
    session = SimpleNamespace(stamps={i: float(i * 5) for i in range(100)})
    rows = [{"outside_id": i, "lobby_id": 99-i, "retrieval_score": float(100-i)} for i in range(100)]
    source = {"selected_candidates": rows, "pair_results": [{**row, "valid_depth_matches": 10 if row["outside_id"] == 40 else 0} for row in rows]}
    first = select_candidates(source, session, session, 12)
    assert first == select_candidates(source, session, session, 12)
    assert len(first) == len({(r["outside_id"], r["lobby_id"]) for r in first}) == 12
    assert first[0]["outside_id"] == 40
    assert any(r["selection_reason"].startswith("temporal") for r in first)


def test_high_learned_confidence_never_substitutes_for_depth(tmp_path):
    fake = SimpleNamespace(unproject=lambda node, pixels: np.full((len(pixels), 3), np.nan),
                           depth=lambda node: (np.zeros((32, 32)),),
                           image=lambda node: np.zeros((128, 128, 3), dtype=np.uint8))
    pixels = np.zeros((40, 2))
    result = verify_pixels(fake, fake, {"outside_id": 1, "lobby_id": 2},
                           pixels, pixels, np.ones(40), tmp_path)
    assert result["learned_confident_matches"] == 40
    assert result["accepted"] is False
    assert result["reason"] == "too_few_spatially_distinct_valid_depth_matches"
