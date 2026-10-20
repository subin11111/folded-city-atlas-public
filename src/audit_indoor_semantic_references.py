"""Check stable semantic IDs and source-photo anchors after mesh reindexing."""
import argparse
import json
from pathlib import Path

import numpy as np

from optimize_session import sha256_file, utc_now


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run = json.loads((a.bundle/'manifest.json').read_text())
    path = a.bundle/'semantic_objects.json'
    record = json.loads(path.read_text())
    ids = [o['id'] for o in record['objects']]
    assert len(ids) == len(set(ids))
    checked = []
    for obj in record['objects']:
        assert 'current_triangle_indices' not in obj, 'Historical face ordinal exposed as current binding'
        assert obj['label'] and obj['session'] in run['sessions']
        geometry = obj.get('geometry', {})
        if geometry.get('accepted'):
            assert np.isfinite(np.asarray(geometry['world_boundary'])).all()
            assert np.isfinite(np.asarray(geometry['world_centre'])).all()
        photo = a.bundle/obj['session']/obj['source_rgb']
        assert photo.is_file(), 'Missing semantic original photo '+obj['id']
        digest = sha256_file(photo)
        expected = obj.get('source_rgb_sha256', obj.get('source_RGB_sha256'))
        if expected:
            assert digest == expected, 'Source photo changed '+obj['id']
        checked.append({'id': obj['id'], 'class': obj['class'], 'source_photo_sha256': digest})
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'),
        'semantic_file_sha256': sha256_file(path), 'objects': len(ids), 'checks': checked,
        'stable_ids_unique': True, 'historical_triangle_indices_not_exposed_as_current': True,
        'finite_world_anchors_and_source_photos_verified': True,
        'place_address_database_created': False, 'routing_approved': False,
        'scope': 'These are semantic objects, not an automatically inferred room/address directory. Persistent place IDs and portals must remain independent of transient mesh face indices.'}
    a.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
