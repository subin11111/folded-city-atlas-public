#!/usr/bin/env python3
"""Combine completed independent session workers, verifying every copied artifact."""
import argparse
import json
from pathlib import Path
import shutil

from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    a=p.parse_args();target=a.target/'manifest.json';source=a.source/'manifest.json'
    merged=json.loads(target.read_text());addition=json.loads(source.read_text())
    overlap=set(merged['sessions'])&set(addition['sessions'])
    if overlap:raise ValueError('Cannot overwrite existing sessions: '+str(overlap))
    for report in [merged,addition]:
        for path,h in report['inputs_sha256'].items():
            if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)
    common=set(merged['inputs_sha256'])&set(addition['inputs_sha256'])
    if any(merged['inputs_sha256'][k]!=addition['inputs_sha256'][k] for k in common):
        raise ValueError('Worker inputs disagree')
    for name,s in addition['sessions'].items():
        if s['display_budget']!=next(iter(merged['sessions'].values()))['display_budget']:
            raise ValueError('Different display budget')
        destination=a.target/name
        if destination.exists():raise FileExistsError(destination)
        for filename,item in s['outputs'].items():
            if sha256_file(a.source/name/filename)!=item['sha256']:raise ValueError('Worker output changed')
        shutil.copytree(a.source/name,destination)
        for filename,item in s['outputs'].items():
            if sha256_file(destination/filename)!=item['sha256']:raise ValueError('Copy integrity failed')
        merged['sessions'][name]=s
    merged['inputs_sha256'].update(addition['inputs_sha256'])
    merged.setdefault('worker_merges',[]).append({'utc':utc_now(),'source':str(source.resolve()),
        'source_manifest_sha256':sha256_file(source),'sessions':list(addition['sessions']),
        'merge_source_sha256':sha256_file(Path(__file__))})
    target.write_text(json.dumps(merged,indent=2)+'\n')
    print(json.dumps({'merged':list(addition['sessions']),'complete_sessions':list(merged['sessions']),
        'source_outputs_preserved':True}))


if __name__=='__main__':main()
