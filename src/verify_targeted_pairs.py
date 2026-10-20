#!/usr/bin/env python3
"""Independent targeted LoFTR checks; never promotes or fits graph constraints."""
import argparse
import json
from pathlib import Path
import sys

from check_session_overlap import Session
from optimize_session import sha256_file, utc_now
from revalidate_cross_sessions import SESSION_DATABASES
from revalidate_learned_pairs import Matcher, verify_pixels, match_image, SETTINGS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pair',action='append',required=True,help='sessionA:nodeA:sessionB:nodeB')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model',choices=['indoor','outdoor'],default='indoor')
    args=parser.parse_args()
    if args.output.exists(): parser.error('Choose a new output directory')
    (args.output/'matches').mkdir(parents=True)
    (args.output/'correspondences').mkdir()
    model=Matcher(args.model,Path('outputs/student_union_20261017_revalidated/model_cache/checkpoints')/f'loftr_{args.model}.ckpt')
    results=[]
    for value in args.pair:
        sa,a,sb,b=value.split(':');a,b=int(a),int(b)
        first=Session(Path('data/raw/rtabmap')/SESSION_DATABASES[sa],sa)
        second=Session(Path('data/raw/rtabmap')/SESSION_DATABASES[sb],sb)
        pixa,pixb,confidence,metadata=model.match(first.image(a),second.image(b))
        match_image(first.image(a),second.image(b),pixa,pixb,args.output/'matches'/f'{sa}_{a}_{sb}_{b}.jpg')
        row=verify_pixels(first,second,dict(outside_id=a,lobby_id=b,selection_reason='predeclared_failed_joint_whole_link_holdout'),pixa,pixb,confidence,args.output)
        row.update(session_a=sa,session_b=sb,matcher_output=metadata,graph_usage='validation_only_never_training',
                   database_sha256={sa:sha256_file(first.path),sb:sha256_file(second.path)})
        results.append(row)
        first.connection.close();second.connection.close()
    result={'created_utc':utc_now(),'argv':sys.argv,'matcher':model.metadata,'settings':SETTINGS,
            'source_hashes':{str(p):sha256_file(p) for p in [Path(__file__),Path('src/revalidate_learned_pairs.py'),Path('src/revalidate_loop_constraints.py')]},'results':results}
    (args.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps([{'pair':(r['session_a'],r['outside_id'],r['session_b'],r['lobby_id']),'accepted':r['accepted'],'reason':r['reason']} for r in results],indent=2))


if __name__=='__main__':main()
