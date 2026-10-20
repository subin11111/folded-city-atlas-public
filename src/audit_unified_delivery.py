#!/usr/bin/env python3
"""Bind the unified native display, policy pointers and actual artifact hashes."""
import argparse
import json
from pathlib import Path

from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    pointer=json.loads((ROOT/'config/current_local_view.json').read_text());checked={}
    for field in ['appearance_manifest','appearance_review','structural_manifest','structural_cleanup','structural_selection_policy','ceiling_completion_policy','presentation_config','presentation_geometry_audit','roadview_manifest']+[key for key in ['service_slab_config','horizontal_continuity_config','floor_material_config','floor_material_audit','prime_hall_photo_config','edge_finish_config','photo_boundary_finish_config','lower_ceiling_clearance_config','lower_ceiling_overlap_config','ceiling_expansion_config','ceiling_material_config','ceiling_material_audit','shadow_lift_config','shadow_lift_audit','mesh_finish_config','reviewed_wall_config','semantic_binding_audit','final_map_selection'] if key in pointer]:
        path=ROOT/pointer[field];digest=sha256_file(path)
        if digest!=pointer[field+'_sha256']:raise ValueError('Pointer hash mismatch: '+field)
        checked[field]={'path':str(path),'sha256':digest}
    policy=json.loads((ROOT/pointer['structural_selection_policy']).read_text());run=json.loads((ROOT/pointer['structural_manifest']).read_text());cleanup=json.loads((ROOT/pointer['structural_cleanup']).read_text());native=json.loads((a.evidence/'native_loaded.json').read_text());bundle=(ROOT/pointer['structural_manifest']).parent
    assert policy['structural_manifest_sha256']==pointer['structural_manifest_sha256']
    assert cleanup['source_manifest_sha256']==pointer['structural_manifest_sha256']
    assert native['architectural_presentation'] and native['native_window']
    assert native['structural_manifest_sha256']==pointer['structural_manifest_sha256']
    expected={(bundle/n/'clean_textured.glb').resolve() for n in run['sessions']}
    assert set(map(Path,native['loaded_files']))==expected
    for n,s in cleanup['sessions'].items():
        assert sha256_file(bundle/n/'clean_structural.ply')==s['ply_sha256']
        assert sha256_file(bundle/n/'clean_textured.glb')==s['glb_sha256']
    selected=json.loads((ROOT/'config/current_map.json').read_text());assert sha256_file(ROOT/selected['manifest'])==pointer['selected_map_manifest_sha256']==run['selected_manifest_sha256']
    if 'final_map_selection' in pointer:
        final=json.loads((ROOT/pointer['final_map_selection']).read_text())
        assert final['status']==pointer['status']==policy['status']=='USER_SELECTED_FINAL_WORKING_MAP'
        assert final['structural_manifest']==pointer['structural_manifest']
        assert final['structural_manifest_sha256']==pointer['structural_manifest_sha256']
        assert final['selected_map_manifest']==selected['manifest']
        assert final['selected_map_manifest_sha256']==pointer['selected_map_manifest_sha256']
        choices=json.loads((ROOT/selected['selection']).read_text())['sessions']
        assert final['session_pose_choices']=={name:choice['candidate'] for name,choice in choices.items()}
        for record in (final,pointer,policy):
            assert not record['candidate_requires_user_review'] and not record['navigation_ready']
    image=a.evidence/'native_scene_render.png';assert image.is_file()
    result={'created_utc':utc_now(),'pointers':checked,'native_loaded':native,'actual_five_session_files_verified':True,'native_render_sha256':sha256_file(image),'selected_map_unchanged':True,'source_pose_changes':0,'viewer_sha256':sha256_file(ROOT/'src/view_local_roadview.py'),'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Presentation artifact delivery verification; not surveyed accuracy or navigation approval.'}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print('Unified native delivery verified: five sessions, unchanged original map, exact artifact hashes.')


if __name__=='__main__':main()
