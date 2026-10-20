import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from export_joint_candidate import export_policy


def test_rejected_component_not_exported_without_explicit_preview():
    assert export_policy({'accepted': False}) is None


def test_preview_keeps_rejection_and_uses_separate_directory():
    component = {'accepted': False, 'rejection_reasons': ['heldout_error']}
    policy = export_policy(component, True)
    assert policy['directory'] == 'preview_clouds'
    assert policy['unvalidated_preview'] is True
    assert policy['accepted_by_validation'] is False
    assert policy['navigation_ready'] is False
    assert policy['original_rejection_reasons'] == ['heldout_error']
    assert component == {'accepted': False, 'rejection_reasons': ['heldout_error']}


def test_accepted_component_default_export_still_not_surveyed():
    policy = export_policy({'accepted': True})
    assert policy['directory'] == 'clouds'
    assert policy['accepted_by_validation'] is True
    assert policy['unvalidated_preview'] is False
    assert policy['navigation_ready'] is False
