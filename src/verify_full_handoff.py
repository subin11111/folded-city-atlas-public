#!/usr/bin/env python3
"""Verify an extracted private handoff; optionally rehydrate deduplicated hardlinks.

Run from WSL after extraction. Never changes mesh/pose contents. Rehydration makes
the original independent files independent again before any subsequent editing.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def safe_path(root, relative):
    rel = Path(relative)
    if rel.is_absolute() or '..' in rel.parts:
        raise ValueError('Unsafe manifest path')
    # Reject a symlink in any parent. The final component can itself be an inventoried link.
    current = root
    for part in rel.parts[:-1]:
        current = current / part
        if current.is_symlink():
            raise ValueError('Symlink parent in manifest: ' + relative)
    return root / rel


def verify(root, manifest, rehydrate=False):
    count = expanded = 0
    for row in manifest['files']:
        path = safe_path(root, row['path'])
        if row['kind'] == 'symlink':
            if not path.is_symlink() or os.readlink(path) != row['target']:
                raise ValueError('Link mismatch: ' + row['path'])
        else:
            if path.is_symlink() or not path.is_file() or path.stat().st_size != row['size']:
                raise ValueError('File missing or wrong size: ' + row['path'])
            h = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(16 * 1024 * 1024), b''):
                    h.update(block)
            if h.hexdigest() != row['sha256']:
                raise ValueError('Hash mismatch: ' + row['path'])
            if rehydrate and path.stat().st_nlink > 1:
                descriptor, name = tempfile.mkstemp(prefix='.rehydrate-', dir=path.parent)
                os.close(descriptor)
                temp = Path(name)
                try:
                    shutil.copy2(path, temp)
                    os.replace(temp, path)
                finally:
                    if temp.exists():
                        temp.unlink()
                expanded += 1
            if rehydrate:
                os.chmod(path, row['mode'])
                os.utime(path, ns=(row['mtime_ns'], row['mtime_ns']))
        count += 1
        if count % 500 == 0:
            print(f'VERIFIED {count}/{len(manifest["files"])}', flush=True)
    return {'verified_entries': count, 'rehydrated_files': expanded}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path, required=True)
    p.add_argument('--inventory', type=Path, required=True)
    p.add_argument('--rehydrate', action='store_true')
    a = p.parse_args()
    print(json.dumps(verify(a.project.resolve(), json.loads(a.inventory.read_text()), a.rehydrate), indent=2))
