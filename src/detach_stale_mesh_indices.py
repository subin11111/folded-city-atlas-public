"""Invalidate historical face indices after cleanup; stable IDs/world anchors survive."""
import argparse
import json
from pathlib import Path

from optimize_session import sha256_file, utc_now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    args = parser.parse_args()
    path = args.bundle/'semantic_objects.json'
    history = args.bundle/'semantic_objects_before_finish_history.json'
    if history.exists():
        raise FileExistsError(history)
    original = path.read_bytes()
    history.write_bytes(original)
    record = json.loads(original)
    changed = []
    for obj in record['objects']:
        previous = obj.pop('current_triangle_indices', None)
        if previous is not None:
            obj['historical_face_index_count_not_current'] = len(previous)
            changed.append(obj['id'])
        obj['mesh_face_binding_status'] = 'UNBOUND_AFTER_CLEANUP_USE_STABLE_ID_AND_WORLD_BOUNDARY'
        obj['mesh_index_history_file'] = history.name
    record['mesh_face_indices_invalidated_after_finish'] = True
    record['stable_object_ids_world_boundaries_and_source_photos_preserved'] = True
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False)+'\n')
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(args.bundle/'manifest.json'),
        'source_semantic_sha256': sha256_file(history), 'output_semantic_sha256': sha256_file(path),
        'stable_ids_and_geometry_preserved': True, 'objects': len(record['objects']),
        'invalidated_historical_face_indices_on_objects': changed,
        'new_indoor_addresses_or_routing_approvals_created': False,
        'scope': 'Do not use historical triangle ordinals as permanent indoor addresses. Rebinding for click picking remains a future UI export step.'}
    (args.bundle/'semantic_binding_audit.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
