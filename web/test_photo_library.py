import hashlib
import json
import math
import unittest
import photo_library
import walk
from navigation import ROOT,load_navigation

class PhotoLibraryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records=photo_library.load_library()[0]
        cls.by_id={p['id']:p for p in cls.records}
        cls.audit=json.loads((ROOT/'public/data/photo_audit.json').read_text('utf-8'))

    def test_all_rgb_records_decoded_and_preserved(self):
        self.assertEqual(self.audit['total_image_records'],2875)
        self.assertEqual(self.audit['decoded'],2875);self.assertEqual(self.audit['failures'],[])
        self.assertEqual(len(self.records),2875)
        for p in self.records:
            self.assertEqual(hashlib.sha256((ROOT/'public'/p['url']).read_bytes()).hexdigest(),p['sha256'])
            self.assertFalse(p['is_panorama'])

    def test_databases_unchanged(self):
        for session in self.audit['sessions'].values():
            self.assertEqual(session['database_sha256_before'],session['database_sha256_after'])
            path=ROOT.parent/'data'/session['database']
            if path.exists():self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),session['database_sha256_after'])

    def test_camera_and_region_candidates(self):
        candidates=[p for p in self.records if p.get('navigation_eligible')]
        self.assertEqual(len(candidates),2398)
        for p in candidates:
            self.assertTrue(p['pose_available']);self.assertTrue(load_navigation().free(*p['nav_node']))
            self.assertEqual(photo_library.region(tuple(p['nav_node'])),p['region'])

    def test_colour_derivatives_are_separate_and_complete(self):
        report=json.loads((ROOT/'public/data/photo_colour_processing.json').read_text('utf-8'))
        self.assertEqual(report['count'],2875);self.assertEqual(report['reference_commit'],'bdb800d753f31e1773f37ba3283df6c817edb894')
        for p in self.records:
            self.assertNotEqual(p['url'],p['display_url'])
            self.assertEqual(hashlib.sha256((ROOT/'public'/p['display_url']).read_bytes()).hexdigest(),p['display_sha256'])

    def test_match_does_not_cross_wall_region_or_level(self):
        for place in walk.places().values():
            node=tuple(place['navigation_node'])
            for k in range(8):
                photo=photo_library.match(node,k*math.pi/4)
                if photo:
                    original=self.by_id[photo['id']]
                    self.assertEqual(original['nav_node'][0],node[0])
                    self.assertEqual(original['region'],photo_library.region(node))
                    self.assertLessEqual(photo['heading_delta_deg'],55)
                    self.assertLessEqual(photo['distance_m'],5)

    def test_reverse_directions_can_report_no_photo(self):
        # 자료가 없는 방향을 같은 사진으로 채우는 구현은 허용하지 않는다.
        coverage=json.loads((ROOT/'public/data/photo_coverage.json').read_text('utf-8'))
        missing=next(sample for sample in coverage['grid_samples'] if sample['directions']==0)
        node=tuple(missing['node'])
        self.assertTrue(all(photo_library.match(node,k*math.pi/4) is None for k in range(8)))

    def test_transition_path_is_validated_and_all_default_samples_have_photos(self):
        state=walk.start();count=0;frames=[]
        while not state['arrived'] and count<80:
            transition=walk.transition(walk.parse_node(state['node']),state['yaw'],'recommended')
            self.assertFalse(transition['recording_trajectory_used']);frames.extend(transition['samples'])
            self.assertEqual(transition['samples'][0]['fraction'],0);self.assertEqual(transition['samples'][-1]['fraction'],1)
            state=transition['next'];count+=1
        self.assertTrue(state['arrived']);self.assertEqual(count,11);self.assertEqual(len(frames),62)
        self.assertTrue(all(f['photo'] for f in frames))
        for f in frames:
            self.assertIn(f['photo']['id'],self.by_id);self.assertFalse(f['photo']['is_exact_virtual_view'])

    def test_invalid_move_is_not_a_camera_sequence_fallback(self):
        state=walk.start()
        with self.assertRaises(ValueError):walk.transition(walk.parse_node(state['node']),state['yaw'],'next_photo')
        with self.assertRaises(ValueError):walk.transition(walk.parse_node(state['node']),float('nan'),'recommended')

if __name__=='__main__':unittest.main(verbosity=2)
