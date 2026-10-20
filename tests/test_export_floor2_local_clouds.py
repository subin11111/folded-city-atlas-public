import csv
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from export_floor2_local_clouds import (check_support, exporter_command,
                                        ply_vertex_count, pose_ids, verify_inventory)


def test_exact_support_required():
    check_support([3,1,2], [1,2,3])
    with pytest.raises(ValueError, match='support mismatch'):
        check_support([1,2], [1,2,3])
    with pytest.raises(ValueError, match='support mismatch'):
        check_support([1,2,3,4], [1,2,3])


def test_pose_inventory_rejects_duplicate_and_empty_rows(tmp_path):
    path=tmp_path/'poses.csv'
    path.write_text('id,x\n1,0\n1,1\n')
    with pytest.raises(ValueError, match='duplicate'): pose_ids(path)
    path.write_text('id,x\n')
    with pytest.raises(ValueError, match='Empty'): pose_ids(path)


def test_export_command_fixes_identical_dense_parameters():
    command=exporter_command(Path('/src/export.py'),Path('/raw/db.db'),Path('/poses.csv'),Path('/work'))
    expected={'--frame-stride':'1','--pixel-stride':'1','--voxel':'0.05',
              '--min-depth':'0.25','--max-depth':'4','--min-confidence':'50',
              '--batch-frames':'20','--missing-pose-policy':'skip'}
    for key,value in expected.items(): assert command[command.index(key)+1]==value


def test_decoded_inventory_requires_all_selected_frames():
    summary={'selected_depth_frames':3,'decoded_depth_frames':3,'skipped_missing_pose_ids':[4]}
    result=verify_inventory(summary,[1,2,3],[1,2,3,4],[1,2,3])
    assert result['decoded_contributing_frame_ids']==[1,2,3]
    assert result['skipped_missing_pose_count']==1
    summary['decoded_depth_frames']=2
    with pytest.raises(ValueError, match='Not every selected'): verify_inventory(summary,[1,2,3],[1,2,3,4],[1,2,3])


def test_no_silent_exportable_node_loss():
    summary={'selected_depth_frames':2,'decoded_depth_frames':2,'skipped_missing_pose_ids':[]}
    with pytest.raises(ValueError, match='support mismatch'): verify_inventory(summary,[1,2,3],[1,2],[1,2])


def test_ply_header_count(tmp_path):
    path=tmp_path/'cloud.ply'
    path.write_bytes(b'ply\nformat binary_little_endian 1.0\nelement vertex 123\nend_header\n')
    assert ply_vertex_count(path)==123
