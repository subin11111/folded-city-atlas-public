#!/usr/bin/env python3
"""Index every saved evidence file without rewriting the narrative/history."""
import argparse
from collections import defaultdict
from pathlib import Path
import subprocess
from urllib.parse import quote

START = '<!-- BEGIN GENERATED EVIDENCE INDEX -->'
END = '<!-- END GENERATED EVIDENCE INDEX -->'
VISUAL = {'.png', '.jpg', '.jpeg', '.svg', '.webp', '.gif'}


def link(path, root):
    relative = path.relative_to(root).as_posix()
    label = path.name.replace('[', '\\[').replace(']', '\\]')
    return f'[{label}]({quote(relative, safe="/-._~")})'


def ignored_paths(files, root):
    """Do not publish broken links to intentionally local-only preview galleries."""
    names = [p.relative_to(root).as_posix() for p in files]
    if not names:
        return set()
    result = subprocess.run(['git', 'check-ignore', '-z', '--stdin'], cwd=root,
                            input=('\0'.join(names)+'\0').encode(), capture_output=True)
    if result.returncode == 128:  # A synthetic non-repository used by unit tests.
        return set()
    if result.returncode not in (0, 1):
        raise RuntimeError('Could not inspect Git ignore rules')
    return set(result.stdout.decode().strip('\0').split('\0')) if result.stdout else set()


def render_index(root):
    files = sorted(p for p in (root / 'docs/assets').rglob('*') if p.is_file())
    ignored = ignored_paths(files, root)
    groups = defaultdict(list)
    for path in files:
        groups[path.relative_to(root / 'docs/assets').parts[0]].append(path)
    visuals = sum(p.suffix.lower() in VISUAL for p in files)
    experiments = sorted((root / 'docs/experiments').glob('*.md'))
    lines = [START, '', f'실제 보관 파일 **{len(files):,}개**: 시각자료 **{visuals:,}개**, '
             f'수치·출처·감사 등 **{len(files)-visuals:,}개**. 실험 문서 **{len(experiments)}개**.',
             '성공·실패·중간 후보를 모두 포함한다. 같은 장면의 다른 렌더도 별도 파일로 센다.',
             f'원격 저장 대상 **{len(files)-len(ignored):,}개**, 기존 Git 제외 로컬 중간본 **{len(ignored):,}개**.',
             '사진 수는 독립 실험 수·독립 관측 수가 아니다. 원격 파일은 링크, 로컬 전용은 경로로 표시한다.',
             '로컬 중간본도 삭제하지 않았다. 대표 실패 사진은 본문과 저장 대상 갤러리에 별도로 보존한다.', '',
             '<details>', '<summary>실험 문서 전체 — 알고리즘·설정·실패·재현 명령</summary>', '']
    lines += [f'- {link(path, root)}' for path in experiments]
    lines += ['', '</details>', '']
    for name, paths in groups.items():
        count = sum(p.suffix.lower() in VISUAL for p in paths)
        lines += ['<details>', f'<summary>{name} — 시각자료 {count}개 / 전체 파일 {len(paths)}개</summary>', '']
        for path in paths:
            relative = path.relative_to(root / 'docs/assets' / name).as_posix()
            label = relative.replace('[', '\\[').replace(']', '\\]')
            url = quote(path.relative_to(root).as_posix(), safe='/-._~')
            kind = '사진/그림' if path.suffix.lower() in VISUAL else '수치/출처'
            if path.relative_to(root).as_posix() in ignored:
                lines.append(f'- {kind}: `{path.relative_to(root).as_posix()}` — 로컬 전용·Git 제외')
            else:
                lines.append(f'- {kind}: [{label}]({url})')
        lines += ['', '</details>', '']
    lines += [END]
    return '\n'.join(lines)


def update_text(text, root):
    if text.count(START) != 1 or text.count(END) != 1:
        raise ValueError('README must contain exactly one evidence index marker pair')
    before, rest = text.split(START)
    _, after = rest.split(END)
    return before + render_index(root) + after


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    path = root / 'README.md'
    original = path.read_text()
    updated = update_text(original, root)
    if args.check:
        if original != updated:
            raise SystemExit('Evidence index is stale; run scripts/update_readme_evidence_index.py')
        print('README evidence index is complete and current')
    else:
        path.write_text(updated)
        print('Updated only the generated README evidence index')


if __name__ == '__main__':
    main()
