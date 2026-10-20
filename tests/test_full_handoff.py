import json
from pathlib import Path

import pytest

from build_full_handoff import exclude_reason, inventory
from export_visible_conversation import visible_item, export, user_image_fallbacks
from verify_full_handoff import verify, safe_path


def record(kind, content, phase=None, identity='m1'):
    return {'type': 'event_msg', 'timestamp': '2026-10-19T00:00:00Z',
            'payload': {'type': 'item_completed', 'item':
                        {'type': kind, 'id': identity, 'phase': phase, 'content': content}}}


@pytest.mark.parametrize('kind', ['Reasoning', 'ContextCompaction', 'CommandExecution', 'McpToolCall', 'FileChange'])
def test_hidden_and_tool_items_never_export(kind):
    assert visible_item(record(kind, [{'type': 'Text', 'text': 'private'}])) is None


def test_only_visible_assistant_phases_export():
    assert visible_item(record('AgentMessage', [{'type': 'Text', 'text': 'answer'}], 'final_answer'))['role'] == 'assistant'
    assert visible_item(record('AgentMessage', [{'type': 'Text', 'text': 'secret'}], 'analysis')) is None
    assert visible_item({'type': 'response_item', 'payload': {'role': 'system', 'content': 'private'}}) is None


def test_export_keeps_repeated_text_different_ids_and_deduplicates_ids(tmp_path):
    a = record('UserMessage', [{'type': 'text', 'text': '계속\n'}])
    b = record('UserMessage', [{'type': 'text', 'text': '계속\n'}], identity='m2')
    hidden = record('Reasoning', [{'type': 'Text', 'text': 'do not expose'}])
    source = tmp_path / 'rollout.jsonl'
    source.write_text('\n'.join(json.dumps(x) for x in [a, b, a, hidden]) + '\n')
    metadata = export([source], tmp_path / 'out')
    assert metadata['user_messages'] == 2
    assert 'do not expose' not in (tmp_path / 'out/conversation.json').read_text()


@pytest.mark.parametrize('name', ['.env', 'outputs/.env.prod', 'data/auth.json', 'id_ed25519', 'x/secret.key', '.git/objects/a', '.venv/a'])
def test_private_and_environment_paths_excluded(name):
    assert exclude_reason(Path(name), Path('outputs/handoff'))


def test_raw_failed_outputs_and_pdf_are_local_handoff_inputs_not_git_files():
    for name in ['data/raw/log.db', 'outputs/failed/mesh.glb', 'docs/review.pdf', 'outputs/surface_runtime/vendor.cpp']:
        assert exclude_reason(Path(name), Path('outputs/handoff')) is None
    assert exclude_reason(Path('outputs/handoff/chat/conversation.json'), Path('outputs/handoff'))


def test_final_development_scope_omits_old_large_outputs():
    destination = Path('outputs/handoff')
    assert exclude_reason(Path('outputs/student_union_final_mesh_v3'), destination, 'final-dev')
    assert exclude_reason(Path('outputs/surface_runtime'), destination, 'final-dev')
    assert exclude_reason(Path('data/raw/rtabmap/261006-162929.db'), destination, 'final-dev')
    assert exclude_reason(Path('outputs/student_union_final_mesh_v9/floor2/clean_textured.glb'), destination, 'final-dev') is None
    assert exclude_reason(Path('outputs/student_union_20261017_session_portfolios_v1/floor2/poses/J04.csv'), destination, 'final-dev') is None
    assert exclude_reason(Path('outputs/student_union_20261017_session_portfolios_v1/floor2/clouds'), destination, 'final-dev')


def test_inventory_detects_duplicates_without_following_symlinks(tmp_path):
    (tmp_path / 'a').write_bytes(b'same')
    (tmp_path / 'b').write_bytes(b'same')
    (tmp_path / 'link').symlink_to('/outside-private-file')
    (tmp_path / '.env').write_text('secret')
    destination = tmp_path / 'outputs/handoff'
    destination.mkdir(parents=True)
    doc = inventory(tmp_path, destination)
    assert doc['logical_bytes'] == 8 and doc['unique_bytes'] == 4
    assert sum('duplicate_of' in r for r in doc['files']) == 1
    assert next(r for r in doc['files'] if r['path'] == 'link')['target'] == '/outside-private-file'
    assert not any(r['path'] == '.env' for r in doc['files'])


def test_only_explicit_user_attachment_fallbacks():
    image = {'type': 'input_image', 'image_url': 'data:image/png;base64,AAAA'}
    record = {'type': 'response_item', 'payload': {'type':'message','role':'user','content':[
        {'type':'input_text','text':'<image name=[Image #1] path="/tmp/a.png">'}, image]}}
    assert user_image_fallbacks(record) == {'/tmp/a.png': image['image_url']}
    record['payload']['role'] = 'developer'
    assert user_image_fallbacks(record) == {}


def test_rehydrate_prevents_future_edits_changing_history(tmp_path):
    import os
    (tmp_path / 'a').write_bytes(b'same')
    os.link(tmp_path / 'a', tmp_path / 'b')
    destination = tmp_path / 'outputs/handoff'
    destination.mkdir(parents=True)
    doc = inventory(tmp_path, destination)
    result = verify(tmp_path, doc, rehydrate=True)
    assert result['verified_entries'] == 2
    assert (tmp_path/'a').stat().st_ino != (tmp_path/'b').stat().st_ino
    (tmp_path/'a').write_bytes(b'new!')
    assert (tmp_path/'b').read_bytes() == b'same'


def test_restore_rejects_path_traversal_and_symlink_parents(tmp_path):
    with pytest.raises(ValueError):
        safe_path(tmp_path, '../private')
    (tmp_path/'escape').symlink_to('/tmp')
    with pytest.raises(ValueError):
        safe_path(tmp_path, 'escape/secret')


def test_archive_round_trip(tmp_path):
    import subprocess
    import tarfile
    from build_full_handoff import archive
    (tmp_path/'first.txt').write_text('exact original\n')
    (tmp_path/'second.txt').write_text('exact original\n')
    (tmp_path/'first.txt').chmod(0o640)
    (tmp_path/'second.txt').chmod(0o751)
    (tmp_path/'relative-link').symlink_to('first.txt')
    destination = tmp_path/'outputs/handoff'
    destination.mkdir(parents=True)
    doc = inventory(tmp_path, destination)
    (destination/'file_inventory.json').write_text(json.dumps(doc))
    archive(tmp_path, destination, doc, reserve_gib=0)
    restored = tmp_path/'restored'
    restored.mkdir()
    stream = subprocess.Popen(['zstd','-dc','--long=30',str(destination/'full_development.tar.zst')], stdout=subprocess.PIPE)
    with tarfile.open(mode='r|',fileobj=stream.stdout) as tar:
        tar.extractall(restored)
    assert stream.wait() == 0
    assert verify(restored/'project', doc, rehydrate=True)['verified_entries'] == 3
    for row in doc['files']:
        if row['kind'] == 'file':
            st = (restored/'project'/row['path']).stat()
            assert st.st_mode & 0o777 == row['mode']
            assert st.st_mtime_ns == row['mtime_ns']
