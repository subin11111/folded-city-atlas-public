import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/update_readme_evidence_index.py'
spec = importlib.util.spec_from_file_location('readme_evidence_index', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_every_saved_evidence_file_is_indexed(tmp_path):
    folder = tmp_path / 'docs/assets/failed run'
    folder.mkdir(parents=True)
    (folder / 'before.png').touch()
    (folder / 'rejected after.png').touch()
    (folder / 'audit.json').touch()
    (tmp_path / 'docs/experiments').mkdir()
    (tmp_path / 'docs/experiments/story.md').touch()
    rendered = module.render_index(tmp_path)
    assert '시각자료 **2개**' in rendered
    assert 'rejected%20after.png' in rendered
    assert 'audit.json' in rendered
    assert 'story.md' in rendered


def test_narrative_and_original_history_are_unchanged(tmp_path):
    (tmp_path / 'docs/assets').mkdir(parents=True)
    (tmp_path / 'docs/experiments').mkdir()
    source = '새 이야기\n' + module.START + '\nold index\n' + module.END + '\n원래 실패 기록\n'
    result = module.update_text(source, tmp_path)
    assert result.startswith('새 이야기\n')
    assert result.endswith('\n원래 실패 기록\n')
    assert module.update_text(result, tmp_path) == result


def test_local_only_previews_do_not_become_broken_remote_links(tmp_path, monkeypatch):
    folder = tmp_path / 'docs/assets/local-preview'
    folder.mkdir(parents=True)
    (folder / 'rejected.png').touch()
    (tmp_path / 'docs/experiments').mkdir()
    relative = 'docs/assets/local-preview/rejected.png'
    monkeypatch.setattr(module, 'ignored_paths', lambda files, root: {relative})
    rendered = module.render_index(tmp_path)
    assert f'`{relative}` — 로컬 전용·Git 제외' in rendered
    assert f']({relative})' not in rendered
    assert '기존 Git 제외 로컬 중간본 **1개**' in rendered
