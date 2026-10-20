#!/usr/bin/env python3
"""Re-render an immutable numerical evaluation report and extract exact summaries."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from evaluate_geometry_stage import PAIRS
from optimize_session import sha256_file


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    path=args.report.resolve()
    original_hash=sha256_file(path)
    report=json.loads(path.read_text())
    stages=list(report['comparisons'][0]['stage_plane_abs_median_m'])
    names={'previous_preview':'Previous RViz','selected_initial':'Selected baseline','pgo4':'4DoF',
           'pgo6_attitude':'SE3 + attitude','pgo6_normals':'SE3 + normals','pgo6_planes':'SE3 + finite planes'}
    colors=['#7e8794','#bd655e','#9f8e35','#4d95ad','#7d63a4','#258367']
    fig,axes=plt.subplots(3,1,figsize=(16,15))
    fig.subplots_adjust(top=.86,bottom=.055,left=.065,right=.99,hspace=.33)
    for ax,session in zip(axes,['outside','lobby','floor2']):
        rows=[r for r in report['comparisons'] if r['session']==session and (r['a'],r['b']) in PAIRS[session]]
        x=np.arange(len(rows));width=.80/len(stages)
        for i,stage in enumerate(stages):
            ax.bar(x+(i-(len(stages)-1)/2)*width,
                [100*r['stage_plane_abs_median_m'][stage] for r in rows],width,
                label=names.get(stage,stage),color=colors[i%len(colors)])
        ax.set(xticks=x,xticklabels=[f"{r['a']} / {r['b']}\n{r['kind']}  p{r['plane_a']}→p{r['plane_b']}" for r in rows],
            ylabel='Median separation (cm)',title=session.upper())
        ax.tick_params(axis='x',labelsize=10)
        ax.grid(axis='y',alpha=.22);ax.set_axisbelow(True)
        ax.margins(x=.04)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.95),ncol=3,fontsize=11,frameon=False)
    fig.suptitle('Fixed historical surface patches across map stages\n'
                 'Same finite-patch samples; panels separate sessions for readable labels',fontsize=15,y=.99)
    fig.savefig(path.parent/'finite_patch_progression.png',dpi=180);plt.close(fig)
    summary={'source_report':str(path),'source_report_sha256':original_hash,
             'metric':'Distribution of per-patch symmetric median absolute plane residual; metres',
             'all_patch_count':report['fixed_patch_count'],
             'reserved_patch_count':sum(r['stage']==stages[0] and r['graph_role']=='reserved_graph_holdout' for r in report['patch_metrics']),
             'reserved_unique_pair_count':len({(r['session'],r['a'],r['b']) for r in report['patch_metrics'] if r['graph_role']=='reserved_graph_holdout'}),
             'variants':{}}
    for stage in stages:
        result={}
        for scope in ['all','reserved']:
            rows=[r for r in report['patch_metrics'] if r['stage']==stage and
                  (scope=='all' or r['graph_role']=='reserved_graph_holdout')]
            values=np.array([r['symmetric_plane_abs_median_m'] for r in rows])
            result[scope]={'count':len(values),'median_m':float(np.median(values)),
                'p90_m':float(np.percentile(values,90)),'maximum_m':float(values.max())}
            comparisons=[r for r in report['comparisons'] if scope=='all' or r['graph_role']=='reserved_graph_holdout']
            for label,reference in [('selected_initial','selected_initial'),('previous_preview','previous_preview')]:
                result[scope]['regressions_over_5cm_vs_'+label]=sum(r['stage_plane_abs_median_m'][stage]>
                    r['stage_plane_abs_median_m'][reference]+.05 for r in comparisons)
        summary['variants'][stage]=result
    if sha256_file(path)!=original_hash:raise RuntimeError('Numerical report changed')
    (path.parent/'compact_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
