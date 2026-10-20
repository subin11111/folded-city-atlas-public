#!/usr/bin/env python3
"""Plot actual outer-step logs; no model changes or synthetic measurements."""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
SETTINGS = [('reference', '', '#c76b32', 'LSMR 500, forward FD'),
            ('lsmr10000', '_lsmr10000', '#28759b', 'LSMR 10000, forward FD'),
            ('central', '_central', '#517b4f', 'LSMR 10000, central FD')]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    output = ROOT / 'numerical_progress.png'
    summary_path = ROOT / 'numerical_progress_summary.json'
    if output.exists() or summary_path.exists():
        raise FileExistsError('Numerical evidence already published')
    reports = {}
    hashes = {str(Path(__file__).resolve()): sha(__file__)}
    for case in ('frame_baseline', 'frame_refined'):
        for key, suffix, _, _ in SETTINGS:
            path = ROOT / (case + suffix) / 'report.json'
            hashes[str(path)] = sha(path)
            report = json.loads(path.read_text())
            if not report['complete'] or not report['input_hashes_unchanged']:
                raise ValueError('Incomplete or unverified solve')
            reports[case, key] = report
        if len({reports[case, key]['graph_manifest_sha256'] for key, *_ in SETTINGS}) != 1:
            raise ValueError('Numerical comparison must have identical graph bytes')

    fig, axes = plt.subplots(2, 3, figsize=(17.5, 9.5))
    records = []
    for row, case in enumerate(('frame_baseline', 'frame_refined')):
        for key, _, color, label in SETTINGS:
            report = reports[case, key]
            v = report['variants']['baseline_huber']
            iterations = v['solver_iterations']
            steps = np.array([x['outer_iteration'] for x in iterations])
            counts = np.cumsum([x['nfev'] for x in iterations])
            cost = np.array([x['cost'] for x in iterations])
            reduction = 100 * (v['initial_weighted_cost']-cost)/v['initial_weighted_cost']
            moves = np.array([x['maximum_parameter_step'] for x in iterations])
            axes[row, 0].plot(np.r_[0, counts], np.r_[0, reduction], '-o',
                ms=3.2, color=color, label=label, alpha=.85)
            axes[row, 1].plot(steps, counts, '-o', ms=3.2, color=color, label=label)
            axes[row, 2].semilogy(steps, moves, '-o', ms=3.2, color=color, label=label)
            critical = [e for e in v['edges'] if e['id'] in
                ('floor2:native_loop:548:981', 'floor2:native_loop:656:1055')]
            records.append(dict(case=case, setting=key,
                report=str(ROOT/(case+dict((x[0],x[1]) for x in SETTINGS)[key])/'report.json'),
                graph_manifest_sha256=report['graph_manifest_sha256'],
                initial_weighted_cost=v['initial_weighted_cost'],
                final_weighted_cost=v['final_weighted_cost'],
                objective_decrease_percent=100*(v['initial_weighted_cost']-v['final_weighted_cost'])/v['initial_weighted_cost'],
                function_evaluations=int(counts[-1]),
                inner_ftol_terminations=sum(x['status']==2 for x in iterations),
                inner_evaluation_limit_terminations=sum(x['status']==0 for x in iterations),
                outer_count=len(iterations), strict_converged=v['strict_converged'],
                last_parameter_step=float(moves[-1]),
                last_scaled_optimality=iterations[-1]['optimality'],
                total_solver_seconds=sum(x['duration_seconds'] for x in iterations),
                numerical_settings=v['numerical_settings'],
                critical_relative_measurement_residuals_not_wall_gaps=critical,
                all_weights_unchanged_during_continuation=all(x['maximum_weight_change']==0 for x in iterations)))
        axes[row, 0].set_xscale('symlog', linthresh=1)
        axes[row, 0].set_xlabel('Cumulative SciPy nfev (symlog)')
        axes[row, 0].set_ylabel('Objective decrease from capped v1 (%)')
        axes[row, 1].set_xlabel('Continuation outer step')
        axes[row, 1].set_ylabel('Cumulative SciPy nfev')
        axes[row, 2].axhline(1e-7, color='#333333', linestyle='--', linewidth=1.2,
            label='Original strict outer threshold: 1e-7')
        axes[row, 2].set_xlabel('Continuation outer step')
        axes[row, 2].set_ylabel('Max component change (m or rad)')
        for col in range(3):
            axes[row, col].grid(True, alpha=.2)
            axes[row, col].set_title(case.replace('_',' ')+' — '+
                ['objective change','evaluation budget','outer step'][col], fontsize=11)
        axes[row, 0].legend(fontsize=8, loc='lower right')
    axes[0,2].legend(fontsize=8, loc='upper right')
    fig.suptitle('Floor2 numerical continuation: identical graphs, numerical settings only', fontsize=16)
    fig.text(.5,.055,
        'Every LSMR-10000 inner solve reached ftol; ALL SIX runs still fail the original strict outer convergence criterion.\n'
        'Points mark completed outer solves. SciPy nfev excludes additional numerical-Jacobian calls; runtime is recorded separately.\n'
        'Objective decrease is <0.001%; 656/1055 relative measurement residual remains about 1.152 m. This is not a wall-gap metric.',
        ha='center', va='center', fontsize=10)
    fig.tight_layout(rect=(0,.10,1,.95))
    fig.savefig(output,dpi=160)
    plt.close(fig)
    if not all(sha(path)==digest for path,digest in hashes.items()):
        raise RuntimeError('Input changed while generating evidence')
    summary=dict(accepted=False, navigation=False, complete=True,
        input_hashes=hashes, input_hashes_unchanged=True,
        figure=str(output),figure_sha256=sha(output),
        comparison_policy='Within each graph only numerical solver settings change; bounds, covariance, measurements and roles are identical.',
        convergence_note='Inner ftol objective stability and original strict outer convergence are reported separately. All six strict flags remain false.',
        nfev_note='SciPy nfev does not count extra numerical-Jacobian evaluations; central FD requires more residual calls per Jacobian.',
        statistical_note='Repeated same observations and same graph; no new measurements or independent validation.',
        records=records)
    summary_path.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(figure=str(output),summary=str(summary_path))))


if __name__=='__main__': main()
