import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from publish_submap_evidence import paired_summary


def test_missing_candidate_does_not_improve_the_baseline_support():
    def row(i, value, before):
        return dict(session="x",a=i,b=i+10,kind="wall",plane_a=0,plane_b=0,
                    stage_plane_abs_median_m={"new":value,"previous_preview":before,
                                             "selected_initial":before})
    report=dict(candidate_stages=["new"],comparisons=[row(1,.08,.04),row(2,None,5.)],
                severe_regression_threshold_m=.05)
    result=paired_summary(report,"new")
    assert result["available_patches"] == 1
    assert result["missing_patches"] == 1
    assert result["same_support_metrics"]["previous_preview"]["median_m"] == .04
    assert result["same_support_metrics"]["new"]["median_m"] == .08
