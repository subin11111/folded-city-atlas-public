import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from merge_surface_continuity_runs import main
from optimize_session import sha256_file


def reports(tmp_path,overlap=False):
    raw=tmp_path/'raw';raw.write_bytes(b'unchanged original')
    target=tmp_path/'target';source=tmp_path/'worker';target.mkdir();source.mkdir()
    name='old' if overlap else 'new';(source/name).mkdir()
    artifact=source/name/'model.glb';artifact.write_bytes(b'fixture artifact')
    inputs={str(raw):sha256_file(raw)}
    old={'sessions':{'old':{'display_budget':600000}},'inputs_sha256':inputs}
    new={'sessions':{name:{'display_budget':600000,'outputs':{'model.glb':{'sha256':sha256_file(artifact)}}}},
         'inputs_sha256':inputs}
    (target/'manifest.json').write_text(json.dumps(old));(source/'manifest.json').write_text(json.dumps(new))
    return target,source


def test_merge_preserves_worker_and_checks_copied_artifact(tmp_path,monkeypatch):
    target,source=reports(tmp_path);before=sha256_file(source/'manifest.json')
    monkeypatch.setattr(sys,'argv',['merge','--target',str(target),'--source',str(source)])
    main()
    assert sha256_file(source/'manifest.json')==before
    assert (target/'new/model.glb').read_bytes()==(source/'new/model.glb').read_bytes()
    assert set(json.loads((target/'manifest.json').read_text())['sessions'])=={'old','new'}


def test_merge_cannot_overwrite_completed_session(tmp_path,monkeypatch):
    target,source=reports(tmp_path,overlap=True);before=sha256_file(target/'manifest.json')
    monkeypatch.setattr(sys,'argv',['merge','--target',str(target),'--source',str(source)])
    with pytest.raises(ValueError,match='overwrite'):main()
    assert sha256_file(target/'manifest.json')==before
