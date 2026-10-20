#!/usr/bin/env python3
"""Actual fixed-observation BA numerical progression; not map accuracy."""
from pathlib import Path
import json
import hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RUN=Path(__file__).resolve().parent


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    files=[RUN/'revisits_dense/656_1055/bundle.json',
           RUN/'revisits_ba_continued/656_1055/bundle.json',
           RUN/'bundle_dense_exact/report.json',RUN/'bundle_dense_exact_previous/report.json']
    hashes={str(p):sha(p) for p in files+[Path(__file__)]}
    reports=[json.loads(p.read_text()) for p in files]
    output=RUN/'bundle_dense_exact/numerical_progress.png'
    summary=RUN/'bundle_dense_exact/numerical_progress_summary.json'
    if output.exists() or summary.exists(): raise FileExistsError(output)
    fig,axes=plt.subplots(1,3,figsize=(14,5.3))
    records=[]
    for name,color in [('visual_ransac','#28759b'),('previous_preview','#c76b32')]:
        candidates=[next(c for c in reports[i]['candidates'] if c['name']==name) for i in (0,1)]
        candidates.append(reports[2 if name=='visual_ransac' else 3]['candidate'])
        for ax,key in zip(axes,['cost','optimality','nfev']):
            ax.plot(range(3),[c[key] for c in candidates],'-o',label=name,color=color)
            ax.set_xticks(range(3),['Sparse LSMR\n250 limit','Sparse LSMR\n1500 continuation','Dense exact\nconverged ftol'])
            ax.grid(True,alpha=.2)
        records.append(dict(seed=name,stages=[{k:c[k] for k in ['cost','optimality','nfev','success','message']} for c in candidates]))
    axes[0].set_ylabel('Same Huber training objective');axes[0].legend(fontsize=9)
    axes[1].set_ylabel('Reported first-order optimality');axes[1].set_yscale('log')
    axes[2].set_ylabel('SciPy nfev in each stage (not cumulative)');axes[2].set_yscale('log')
    fig.suptitle('656 / 1055 multi-frame RGB-D BA: numerical solver comparison',fontsize=14)
    fig.text(.5,.07,'Same 324 RGB observations, 318 depth observations, 285 variables, priors, Huber loss and gauge.\n'
        'Equivalent batched residual: 31–34× faster; residual and central Jacobian differences = 0 in actual-data checks.\n'
        'Two dense solutions agree within 0.132 mm / 0.00257°. These are training and numerical checks, not heldout map accuracy.',
        ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.18,1,.93));fig.savefig(output,dpi=160);plt.close(fig)
    if not all(sha(Path(p))==h for p,h in hashes.items()): raise RuntimeError('Input changed')
    summary.write_text(json.dumps(dict(input_hashes=hashes,input_hashes_unchanged=True,
        figure_sha256=sha(output),records=records,accepted=False,
        note='Only numerical solver changes; all observations and gates fixed.'),indent=2)+'\n')
    print(output)


if __name__=='__main__':main()
