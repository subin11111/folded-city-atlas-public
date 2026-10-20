import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from process_independent_sessions import check_comparison_coverage, load_definition, write_run_definition


def test_configuration_requires_safe_unique_named_inputs(tmp_path):
    path = tmp_path / 'config.json'
    path.write_text(json.dumps({'sessions': {'../bad': {'database': 'a.db'}}}))
    with pytest.raises(ValueError, match='Unsafe'):
        load_definition(path)
    path.write_text(json.dumps({'sessions': {'floor2': {'database': 'a.db'}}}))
    assert list(load_definition(path)['sessions']) == ['floor2']


def test_comparison_rejects_different_frame_coverage(tmp_path):
    raw, optimized, app = [tmp_path / name for name in ('raw_main_trajectory.csv', 'optimized_main_trajectory.csv', 'app.csv')]
    raw.write_text('id\n1\n2\n')
    optimized.write_text('id\n1\n2\n')
    app.write_text('id\n1\n')
    with pytest.raises(ValueError, match='coverage differs'):
        check_comparison_coverage(tmp_path, app)
    app.write_text('id\n1\n2\n')
    check_comparison_coverage(tmp_path, app)


def test_parallel_definition_initialization_does_not_clobber(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    metadata = {'sessions': ['one', 'two']}
    with ThreadPoolExecutor(max_workers=4) as workers:
        list(workers.map(lambda _: write_run_definition(tmp_path, metadata), range(8)))
    assert json.loads((tmp_path / 'run_definition.json').read_text()) == metadata
    with pytest.raises(ValueError, match='different configuration'):
        write_run_definition(tmp_path, {'sessions': ['three']})
