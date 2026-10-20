"""Export a current Git tree, then commit it on an independent public history.

No private .git directory, branches, author history or credentials are copied.
New top-level folders require an explicit publication-policy change.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile

SENSITIVE = [
    re.compile(rb'(?<![A-Za-z0-9_])gh[pousr]_[A-Za-z0-9]{30,}'),
    re.compile(rb'github_pat_[A-Za-z0-9_]{40,}'),
    re.compile(rb'AKIA[0-9A-Z]{16}'),
    re.compile(rb'-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----'),
    re.compile(rb'(?<![A-Za-z0-9])01[016789]-[0-9]{3,4}-[0-9]{4}(?![A-Za-z0-9])'),
    re.compile(rb'(?<![0-9])[0-9]{6}-[1-8][0-9]{6}(?![0-9])'),
]
TEXT_SUFFIXES = {'.md','.txt','.json','.py','.js','.mjs','.cjs','.html','.css','.csv','.toml','.ini','.yaml','.yml','.xml','.ps1','.sh'}

def git(folder, *args, check=True):
    return subprocess.run(['git','-C',str(folder),*args],capture_output=True,check=check).stdout

def allowed(name, policy):
    path=PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts:return False
    if any(p in policy['excluded_parts'] for p in path.parts):return False
    if path.name in policy['excluded_names'] or path.name.startswith('.env'):return False
    if path.suffix.lower() in policy['excluded_suffixes']:return False
    return path.parts[0] in policy['allowed_roots'] or name in policy['allowed_root_files']

def validate_content(path, relative):
    if path.suffix.lower() not in TEXT_SUFFIXES and path.suffix:return
    if path.stat().st_size>8*1024*1024:raise ValueError(f'Text exceeds review limit: {relative}')
    data=path.read_bytes()
    if any(pattern.search(data) for pattern in SENSITIVE):
        raise ValueError(f'Publication blocked: sensitive-text pattern in {relative}')

def target_remote(remote, repository):
    remote=remote.removesuffix('.git')
    return remote in {f'https://github.com/{repository}',f'git@github.com:{repository}'}

def export_snapshot(source, output, policy, revision='HEAD'):
    source,output=Path(source).resolve(),Path(output).resolve()
    if output.exists():raise ValueError('Export output must be a new directory.')
    revision=git(source,'rev-parse','--verify',revision+'^{commit}').decode().strip()
    output.mkdir(parents=True)
    files=excluded=total=0
    digest=hashlib.sha256()
    with tempfile.TemporaryFile() as errors:
        process=subprocess.Popen(['git','-C',str(source),'archive','--format=tar',revision],stdout=subprocess.PIPE,stderr=errors)
        try:
            with tarfile.open(fileobj=process.stdout,mode='r|') as archive:
                for member in archive:
                    if member.isdir():continue
                    if not allowed(member.name,policy):excluded+=1;continue
                    if not member.isfile():raise ValueError(f'Unsupported public entry: {member.name}')
                    target=output.joinpath(*PurePosixPath(member.name).parts)
                    if output not in target.resolve().parents:raise ValueError('Unsafe export path.')
                    target.parent.mkdir(parents=True,exist_ok=True)
                    with archive.extractfile(member) as src, target.open('wb') as dst:shutil.copyfileobj(src,dst)
                    target.chmod(member.mode & 0o777)
                    validate_content(target,member.name)
                    file_hash=hashlib.sha256(target.read_bytes()).hexdigest()
                    digest.update((member.name+'\0'+file_hash+'\n').encode())
                    files+=1;total+=member.size
            if process.wait()!=0:raise RuntimeError('Git snapshot export failed.')
        except Exception:
            process.kill();process.wait();raise
        finally:
            process.stdout.close()
    for required in policy['required_files']:
        if not (output/required).is_file():raise ValueError(f'Required public file missing: {required}')
    return {'files':files,'bytes':total,'excluded':excluded,'snapshot_sha256':digest.hexdigest(),'private_history_exported':False}

def synchronise(snapshot,destination,target_repository,push=False):
    snapshot,destination=Path(snapshot).resolve(),Path(destination).resolve()
    if not snapshot.is_dir() or (snapshot/'.git').exists():raise ValueError('Expected a history-free export.')
    root=Path(git(destination,'rev-parse','--show-toplevel').decode().strip()).resolve()
    if root!=destination:raise ValueError('Destination must be the exact dedicated public checkout.')
    remote=git(destination,'remote','get-url','origin').decode().strip()
    if not target_remote(remote,target_repository):raise ValueError('Refusing to modify a checkout with another remote.')
    if git(destination,'status','--porcelain'):raise ValueError('Public checkout must be clean.')
    git(destination,'rm','-r','--ignore-unmatch','--','.')
    for item in snapshot.iterdir():
        target=destination/item.name
        if item.is_dir():shutil.copytree(item,target,dirs_exist_ok=True)
        else:shutil.copy2(item,target)
    git(destination,'-c','core.autocrlf=false','add','--all','--force')
    if subprocess.run(['git','-C',str(destination),'diff','--cached','--quiet']).returncode==0:
        return {'changed':False}
    git(destination,'-c','user.name=Folded City','-c','user.email=subin11111@users.noreply.github.com',
        'commit','-m','Update public research snapshot')
    if push:git(destination,'push','origin','HEAD:main')
    return {'changed':True,'public_commit':git(destination,'rev-parse','HEAD').decode().strip()}

def main():
    parser=argparse.ArgumentParser()
    sub=parser.add_subparsers(dest='command',required=True)
    export=sub.add_parser('export');export.add_argument('--source',required=True);export.add_argument('--output',required=True)
    sync=sub.add_parser('sync');sync.add_argument('--snapshot',required=True);sync.add_argument('--destination',required=True)
    sync.add_argument('--policy',required=True);sync.add_argument('--push',action='store_true')
    args=parser.parse_args()
    if args.command=='export':
        policy=json.loads((Path(args.source)/'.public-export.json').read_text('utf-8'))
        result=export_snapshot(args.source,args.output,policy)
    else:
        policy=json.loads(Path(args.policy).read_text('utf-8'))
        result=synchronise(args.snapshot,args.destination,policy['target_repository'],args.push)
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
