#!/usr/bin/env python3
"""Private development handoff: source/history/chat + final artifacts, not old meshes.

Does not follow symlinks, include credentials, or overwrite any prior handoff.
Use --inventory-only before Git commit, then --archive after producing a Git bundle.
The exact exclusion ledger is itself archived. Default final-dev scope includes
the five current raw DBs, final v9, selected poses/clouds, photos, map and replay.
Use archive-all ONLY when an explicit archival backup of old outputs is wanted.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time


SKIP_DIRS = {'.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
             '.venv', 'venv', 'env', '.idea', '.vscode', 'node_modules'}
SKIP_ROOTS = {'build', 'install', 'log'}
SECRET_NAMES = {'auth.json', 'credentials.json', 'id_rsa', 'id_ed25519', '.netrc', '.npmrc', '.pypirc'}
FINAL_OUTPUTS = {'student_union_final_mesh_v9', 'student_union_selected_20261017_v1',
                 'student_union_windows_handoff_20261019', 'student_union_roadview_v1',
                 'student_union_cartographic_v3', 'student_union_slam_replay_v3'}
RAW_DATABASES = {'261016-234854-outside.db', '261017-000622-1f.db',
                 '261017-011331-bridgeout1f.db', '261017-013501-2f.db',
                 '261017-013904-bridg12_2.db'}


def support_file(relative):
    parts = relative.parts
    if len(parts) < 2 or parts[0] != 'outputs':
        return False
    if parts[1] == 'student_union_topology_fixed_v2':
        return len(parts) == 2 or relative.name in ('main_floor_1f_anchors.npz', 'shared_upper_floor_anchors.npz',
                'sub_floor_1f_existing_offset_retained_anchors.npz', 'reviewed_atrium.npz')
    if parts[1] == 'student_union_20261017_session_portfolios_v1':
        chosen = {'outside':'J01.csv','lobby':'N05.csv','bridge_out1f':'H03.csv','floor2':'J04.csv','bridge_12_sub':'H03.csv'}
        return len(parts) == 2 or (len(parts) == 3 and parts[2] in chosen) or (
            len(parts) == 4 and parts[2] in chosen and parts[3] == 'poses') or (
            len(parts) == 5 and parts[2] in chosen and parts[3] == 'poses' and parts[4] == chosen[parts[2]])
    return False


def sha256_file(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(16 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def exclude_reason(relative, destination, profile='archive-all'):
    if relative == destination or destination in relative.parents:
        return 'new handoff destination (avoid recursion)'
    if profile == 'final-dev':
        if relative.parts[0] == 'outputs' and len(relative.parts) >= 2 and relative.parts[1] not in FINAL_OUTPUTS and not support_file(relative):
            return 'older/intermediate output or Linux runtime: final-dev handoff keeps final artifacts only'
        if relative.parts[:3] == ('data','raw','rtabmap') and len(relative.parts) == 4 and relative.name not in RAW_DATABASES:
            return 'legacy capture outside the five selected student-union sessions'
    if any(part in SKIP_DIRS for part in relative.parts):
        return 'Git internals, environment, cache or editor state'
    if relative.parts[0] in SKIP_ROOTS:
        return 'ROS build/install/log products'
    name = relative.name
    if name in SECRET_NAMES or name == '.env' or (name.startswith('.env.') and name != '.env.example') or name.endswith(('.pem', '.key')):
        return 'credential-like name (never export automatically)'
    if name.endswith(('.pyc', '.pyo', '.swp')) or name in ('core', '.DS_Store', 'Thumbs.db') or name.startswith('core.'):
        return 'cache or crash artifact'
    return None


def inventory(root, destination, profile='final-dev'):
    root, destination = root.resolve(), destination.resolve()
    dest_relative = destination.relative_to(root)
    records, skipped = [], []
    total = 0
    for folder, dirs, files in os.walk(root, followlinks=False):
        for name in list(dirs):
            path = Path(folder) / name
            rel = path.relative_to(root)
            reason = exclude_reason(rel, dest_relative, profile)
            if reason:
                dirs.remove(name)
                skipped.append({'path': rel.as_posix(), 'reason': reason, 'subtree': True})
            elif path.is_symlink():
                dirs.remove(name)
                files.append(name)
        for name in files:
            path = Path(folder) / name
            rel = path.relative_to(root)
            reason = exclude_reason(rel, dest_relative, profile)
            if reason:
                skipped.append({'path': rel.as_posix(), 'reason': reason})
                continue
            st = path.lstat()
            if path.is_symlink():
                records.append({'path': rel.as_posix(), 'kind': 'symlink', 'target': os.readlink(path), 'size': 0,
                                'mtime_ns': st.st_mtime_ns, 'mode': st.st_mode & 0o777})
            elif path.is_file():
                records.append({'path': rel.as_posix(), 'kind': 'file', 'size': st.st_size,
                                'mtime_ns': st.st_mtime_ns, 'mode': st.st_mode & 0o777})
                total += st.st_size
            else:
                skipped.append({'path': rel.as_posix(), 'reason': 'special file; never dereference'})
    records.sort(key=lambda r: (Path(r['path']).suffix, Path(r['path']).name,
                                Path(r['path']).parent.name, r['path']))
    print(f'INVENTORY {len(records)} entries, {total / 1024**3:.2f} GiB logical', flush=True)
    for i, row in enumerate(records):
        if row['kind'] == 'file':
            path = root / row['path']
            row['sha256'] = sha256_file(path)
            if path.stat().st_size != row['size'] or path.stat().st_mtime_ns != row['mtime_ns']:
                raise ValueError('Input changed while hashing: ' + row['path'])
        if i % 500 == 0:
            print(f'HASH {i}/{len(records)} {row["path"]}', flush=True)
    by_digest = {}
    for row in records:
        if row['kind'] == 'file':
            key = (row['size'], row['sha256'])
            if key in by_digest:
                row['duplicate_of'] = by_digest[key]
            else:
                by_digest[key] = row['path']
    return {'created_utc': datetime.now(timezone.utc).isoformat(), 'source_root': str(root), 'profile': profile,
            'logical_bytes': total, 'unique_bytes': sum(k[0] for k in by_digest),
            'entry_count': len(records), 'files': records, 'excluded': skipped}


class CheckedReader:
    def __init__(self, source):
        self.source = source
        self.hash = hashlib.sha256()

    def read(self, size=-1):
        data = self.source.read(size)
        self.hash.update(data)
        return data


def archive(root, destination, manifest, reserve_gib=10):
    """Stream; duplicate regular files use TAR hardlinks, never unsafe path rewriting."""
    root, destination = root.resolve(), destination.resolve()
    partial = destination / 'full_development.tar.zst.partial'
    final = destination / 'full_development.tar.zst'
    if final.exists() or partial.exists():
        raise FileExistsError('Never overwrite an existing handoff archive')
    # Guard against file traversal even if a supplied manifest was modified.
    for row in manifest['files']:
        p = Path(row['path'])
        if p.is_absolute() or '..' in p.parts:
            raise ValueError('Unsafe inventory path')
    log = (destination / 'compression.log').open('xb')
    with partial.open('xb') as output:
        proc = subprocess.Popen(['zstd', '-T4', '-5', '--long=30', '-c'], stdin=subprocess.PIPE,
                                stdout=output, stderr=log)
        try:
            with tarfile.open(mode='w|', fileobj=proc.stdin, format=tarfile.PAX_FORMAT) as tar:
                # Handoff metadata outside the project subtree; no circular self-inventory.
                for file in sorted(destination.rglob('*')):
                    if file.is_file() and file.name not in ('full_development.tar.zst.partial', 'compression.log'):
                        tar.add(file, arcname='handoff/' + file.relative_to(destination).as_posix(), recursive=False)
                for i, row in enumerate(manifest['files']):
                    if shutil.disk_usage(destination).free < reserve_gib * 1024**3:
                        raise OSError('Free-space reserve reached; partial archive retained, no success claimed')
                    path = root / row['path']
                    if path.lstat().st_mtime_ns != row['mtime_ns']:
                        raise ValueError('Input changed since inventory: ' + row['path'])
                    info = tarfile.TarInfo('project/' + row['path'])
                    info.mode, info.mtime = row['mode'], row['mtime_ns'] / 1e9
                    if row['kind'] == 'symlink':
                        info.type, info.linkname = tarfile.SYMTYPE, row['target']
                        tar.addfile(info)
                    elif row.get('duplicate_of'):
                        info.type, info.linkname = tarfile.LNKTYPE, 'project/' + row['duplicate_of']
                        tar.addfile(info)
                    else:
                        info.size = row['size']
                        with path.open('rb') as source:
                            checked = CheckedReader(source)
                            tar.addfile(info, checked)
                            if checked.hash.hexdigest() != row['sha256']:
                                raise ValueError('Input changed during archive: ' + row['path'])
                    if i % 500 == 0:
                        print(f'ARCHIVE {i}/{len(manifest["files"])} {partial.stat().st_size / 1024**3:.2f} GiB compressed', flush=True)
            proc.stdin.close()
            if proc.wait() != 0:
                raise RuntimeError('zstd failed; see compression.log')
        except BaseException:
            proc.kill()
            proc.wait()
            raise
        finally:
            log.close()
    partial.rename(final)
    print('VERIFY_ZSTD_STREAM', flush=True)
    subprocess.run(['zstd', '-t', '--long=30', str(final)], check=True)
    checksum = sha256_file(final)
    (destination / 'full_development.tar.zst.sha256').write_text(checksum + '  full_development.tar.zst\n')
    result = {'archive': final.name, 'bytes': final.stat().st_size, 'sha256': checksum,
              'source_logical_bytes': manifest['logical_bytes'], 'unique_payload_bytes': manifest['unique_bytes'],
              'entries': manifest['entry_count'], 'zstd_integrity_verified': True}
    (destination / 'archive_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument('--destination', type=Path, required=True)
    p.add_argument('--inventory-only', action='store_true')
    p.add_argument('--archive', action='store_true')
    p.add_argument('--profile', choices=['final-dev','archive-all'], default='final-dev')
    a = p.parse_args()
    a.destination.mkdir(parents=True, exist_ok=True)
    mpath = a.destination / 'file_inventory.json'
    if a.inventory_only:
        if mpath.exists():
            raise FileExistsError('Inventory already exists; explicitly choose a new destination')
        doc = inventory(a.root, a.destination, a.profile)
        mpath.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + '\n')
        print(f'UNIQUE {doc["unique_bytes"] / 1024**3:.2f} GiB', flush=True)
    elif a.archive:
        archive(a.root, a.destination, json.loads(mpath.read_text()))
    else:
        p.error('Choose --inventory-only or --archive')


if __name__ == '__main__':
    main()
