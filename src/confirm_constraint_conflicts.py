#!/usr/bin/env python3
"""Higher-budget matched confirmations, without installing proposed deletions."""
import argparse
import json
from pathlib import Path
import shutil
import time

import audit_constraint_conflicts as audit
from continue_floor2_graph import continue_graph
from optimize_robust_submaps import transform_record, _edge_chi2
from optimize_session import sha256_file, utc_now
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    inputs=audit.bind_sources();audit.prepare_observations()
    inputs.add(Path(__file__));hashes={str(x.resolve()):sha256_file(x) for x in inputs}
    a.output.mkdir(parents=True);shutil.copyfile(Path(__file__),a.output/'source_snapshot.py')
    results=[]
    jobs=[('lobby','control',[]),
          ('lobby','without_18_19',['odom:lobby:s0018:lobby:s0019']),
          ('lobby','without_19_20',['odom:lobby:s0019:lobby:s0020'])]
    for domain,label,ids in jobs:
        state=audit.STATE[domain];edges=[e for e in state['edges'] if e.id not in ids]
        started=time.monotonic()
        poses,solver=continue_graph(state['nodes'],edges,state['warm'],outer_iterations=12,
            max_evaluations=400,linear_maxiter=10000,jacobian_scheme='3-point')
        residuals=[]
        for edge in state['edges']:
            if edge.id not in ids:continue
            error,chi2=_edge_chi2(edge,poses)
            residuals.append({'id':edge.id,'translation_m':float(np.linalg.norm(error[:3])),
                              'rotation_deg':float(np.degrees(np.linalg.norm(error[3:]))),'chi2':chi2})
        result={'domain':domain,'label':label,'remove_ids':ids,'elapsed_seconds':time.monotonic()-started,
            'score':audit.score(domain,poses),'excluded_factor_prediction':residuals,
            'solver':solver,'poses':{k:transform_record(v) for k,v in poses.items()}}
        audit.dump(a.output/(label+'.json'),result);results.append(result)
        print(json.dumps({'domain':domain,'label':label,'score':result['score']['fixed_patch_statistics'],
                          'excluded_factor_prediction':residuals}),flush=True)
    audit.dump(a.output/'completion.json',{'finished_utc':utc_now(),'count':len(results),
        'parameters':{'outer_iterations':12,'max_evaluations':400,'linear_maxiter':10000,'jacobian_scheme':'3-point'},
        'input_hashes':hashes,'input_hashes_unchanged':all(sha256_file(Path(x))==h for x,h in hashes.items()),
        'default_map_replaced':False})


if __name__=='__main__':main()
