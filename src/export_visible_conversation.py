#!/usr/bin/env python3
"""Export only completed, user-visible messages from explicitly supplied Codex rollouts.

Never copy a rollout, account database, reasoning, tool trace or system prompt.
This is a private, readable context export, not an official session import format.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
from datetime import datetime, timezone


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def visible_item(record):
    """Use a positive type/phase whitelist, not a list of hidden types to exclude."""
    event = record.get('payload', {})
    if record.get('type') != 'event_msg' or event.get('type') != 'item_completed':
        return None
    item = event.get('item', {})
    kind = item.get('type')
    if kind not in ('UserMessage', 'AgentMessage'):
        return None
    phase = item.get('phase')
    if kind == 'AgentMessage' and phase not in ('commentary', 'final_answer', 'final'):
        return None
    if not item.get('id'):
        raise ValueError('A visible message without an ID cannot be deduplicated safely')
    # Preserve only display text and explicit image content; never arbitrary metadata.
    content = []
    for part in item.get('content', []):
        if part.get('type') in ('text', 'Text'):
            content.append({'type': 'text', 'text': part['text']})
        elif kind == 'UserMessage' and part.get('type') == 'local_image':
            content.append({'type': 'local_image', 'path': part['path']})
        elif kind == 'UserMessage' and part.get('type') == 'image':
            content.append({'type': 'image', 'image_url': part['image_url']})
    return {'id': item['id'], 'timestamp': record.get('timestamp'),
            'role': 'user' if kind == 'UserMessage' else 'assistant',
            'phase': phase, 'content': content}


def user_image_fallbacks(record):
    """Recover displayed input-image bytes only when an explicit user image path matches.

    The UI's /tmp attachment may have expired. The corresponding user input still
    contains its displayed (possibly resized) pixels. Never export other response
    item roles, text, metadata or tools through this fallback.
    """
    item = record.get('payload', {})
    if record.get('type') != 'response_item' or item.get('type') != 'message' or item.get('role') != 'user':
        return {}
    text = '\n'.join(p.get('text','') for p in item.get('content',[]) if p.get('type') == 'input_text')
    paths = re.findall(r'<image\b[^>]*\bpath="([^"]+)"[^>]*>', text)
    images = [p['image_url'] for p in item.get('content',[]) if p.get('type') == 'input_image' and 'image_url' in p]
    return dict(zip(paths, images)) if len(paths) == len(images) else {}


def export(rollouts, output):
    output.mkdir(parents=True, exist_ok=False)
    attachments = output / 'attachments'
    attachments.mkdir()
    messages, seen, missing, source_inventory = [], {}, [], []
    embedded_by_path = {}
    for source in rollouts:
        # Freeze at the opening byte length: later appended messages belong to a later export.
        limit = source.stat().st_size
        h = hashlib.sha256()
        with source.open('rb') as stream:
            while stream.tell() < limit:
                line = stream.readline(limit - stream.tell())
                h.update(line)
                if not line.endswith(b'\n'):
                    break
                record = json.loads(line)
                for path, url in user_image_fallbacks(record).items():
                    embedded_by_path.setdefault(path, url)
                msg = visible_item(record)
                if msg is None:
                    continue
                if msg['id'] in seen:
                    if seen[msg['id']]['content'] != msg['content']:
                        raise ValueError('Conflicting completed message ID: ' + msg['id'])
                    continue
                seen[msg['id']] = msg
                messages.append(msg)
        source_inventory.append({'filename': source.name, 'snapshot_bytes': limit,
                                 'snapshot_sha256': h.hexdigest()})
    for msg in messages:
        for part in msg['content']:
            if part['type'] == 'local_image':
                original = Path(part.pop('path'))
                part['original_path'] = str(original)
                # Only explicit image attachments, never a general-purpose file collector.
                allowed = original.is_absolute() and original.suffix.lower() in ('.png','.jpg','.jpeg','.webp','.gif')
                allowed = allowed and (str(original).startswith('/tmp/') or str(original).startswith('/mnt/data/'))
                if allowed and original.is_file() and not original.is_symlink():
                    raw = original.read_bytes()
                    name = hashlib.sha256(raw).hexdigest() + original.suffix.lower()
                    (attachments / name).write_bytes(raw)
                    part.update(file='attachments/' + name, sha256=hashlib.sha256(raw).hexdigest())
                else:
                    fallback = embedded_by_path.get(str(original), '')
                    match = re.fullmatch(r'data:image/(png|jpeg|webp|gif);base64,([A-Za-z0-9+/=\r\n]+)', fallback)
                    if match:
                        raw = base64.b64decode(match[2], validate=False)
                        name = hashlib.sha256(raw).hexdigest() + '.' + match[1]
                        (attachments / name).write_bytes(raw)
                        part.update(file='attachments/' + name, sha256=hashlib.sha256(raw).hexdigest(),
                                    recovery='displayed user input pixels; original /tmp binary unavailable; may be resized')
                    else:
                        part['missing'] = True
                        missing.append({'message_id': msg['id'], 'path': str(original)})
            elif part['type'] == 'image':
                url = part.pop('image_url')
                match = re.fullmatch(r'data:image/(png|jpeg|webp|gif);base64,([A-Za-z0-9+/=\r\n]+)', url)
                if match:
                    raw = base64.b64decode(match[2], validate=False)
                    name = hashlib.sha256(raw).hexdigest() + '.' + match[1]
                    (attachments / name).write_bytes(raw)
                    part.update(file='attachments/' + name, sha256=hashlib.sha256(raw).hexdigest(),
                                recovery='embedded user-visible attachment')
                else:
                    part.update(missing=True, original_reference=url if not url.startswith('data:') else 'unsupported embedded format')
                    missing.append({'message_id': msg['id'], 'path': part['original_reference']})
    metadata = {'created_utc': datetime.now(timezone.utc).isoformat(),
                'format': 'private-visible-conversation-v1', 'sources': source_inventory,
                'message_count': len(messages),
                'user_messages': sum(m['role'] == 'user' for m in messages),
                'assistant_messages': sum(m['role'] == 'assistant' for m in messages),
                'attachment_files': len(list(attachments.iterdir())), 'missing_attachments': missing,
                'excluded': ['system/developer instructions', 'reasoning', 'tool logs', 'compaction summaries', 'account state'],
                'limitations': ['Only completed messages in the two available local rollout segments.',
                               'Not an official Codex session import or hidden execution-state backup.',
                               'The initial shared ChatGPT URL was not independently exported.',
                               'The older Folded_City_Codex_인계.md remains a separate historical source.',
                               'This export cannot include future messages after its byte cutoffs.']}
    (output / 'conversation.json').write_text(json.dumps({'metadata': metadata, 'messages': messages}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    parts = ['# 이 Codex 대화의 표시 원문\n',
             '사용자/답변의 표시 텍스트를 보존한 로컬 내보내기입니다. README의 의미 요약과 다릅니다.\n'
             '시스템·개발자 지시, 내부 추론, 도구 로그, 계정 정보는 포함하지 않습니다.\n'
             '원본 저장 구간·메시지 ID·시각·복구되지 않은 첨부는 conversation.json에 기록합니다.\n'
             '공식 세션 가져오기 파일이 아니며, 초기 공유 링크의 전체 채팅까지 복구했다는 뜻은 아닙니다.\n']
    for msg in messages:
        parts.append(f"\n## {msg['role']} · {msg['timestamp']} · {msg['phase'] or 'message'}\n\nID: {msg['id']}\n")
        for part in msg['content']:
            if part['type'] == 'text':
                parts.append('\n' + part['text'] + '\n')
            elif part.get('file'):
                parts.append(f"\n![첨부]({part['file']})\n")
            else:
                parts.append('\n[첨부 복구 안 됨: ' + part.get('original_path', part.get('original_reference','unknown')) + ']\n')
    (output / 'conversation.md').write_text('\n'.join(parts), encoding='utf-8')
    (output / 'export_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2), flush=True)
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rollout', action='append', required=True, type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export(args.rollout, args.output)
