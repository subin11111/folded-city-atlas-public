import math
import unittest
import walk
from navigation import load_navigation

class WalkTests(unittest.TestCase):
    def test_start_is_at_entrance_and_not_top_down(self):
        s=walk.start();self.assertIn('입구 앞',s['location']);self.assertEqual(s['eye_height_m'],1.55)
        self.assertFalse(s['is_live_positioning']);self.assertFalse(s['field_verified'])

    def test_left_and_right_stay_on_spatial_grid(self):
        nav=load_navigation();node=tuple(walk.places()['entrance']['navigation_node'])
        for angle in [i*math.pi/8 for i in range(16)]:
            r=walk.direct_step(node,angle)
            if r:
                for a,b in zip(r['nodes'],r['nodes'][1:]):
                    allowed={n for n,_,portal,_ in nav.neighbours(a) if portal is None}
                    self.assertIn(b,allowed);self.assertEqual(a[0],b[0])

    def test_recommendations_arrive_without_camera_sequence(self):
        node=tuple(walk.places()['outside_start']['navigation_node']);remaining=[];kinds=[]
        for _ in range(80):
            s=walk.snapshot(node,0,include_photo=False);remaining.append(s['remaining_m'])
            if s['arrived']:break
            option=next(o for o in s['options'] if o['id']=='recommended');kinds.append(option['kind'])
            node=walk.parse_node(option['destination_node'])
        else:self.fail('목적지에 도착하지 못했습니다.')
        self.assertTrue(all(a>b for a,b in zip(remaining,remaining[1:])))
        self.assertIn('door',kinds);self.assertIn('stairs',kinds);self.assertEqual(node[0],'2')

    def test_no_floor_transition_except_stairs(self):
        nav=load_navigation()
        for p in walk.places().values():
            node=tuple(p['navigation_node']);s=walk.snapshot(node,0,include_photo=False)
            for o in s['options']:
                end=walk.parse_node(o['destination_node'])
                if end[0]!=node[0]:self.assertEqual(o['kind'],'stairs')

    def test_invalid_input_and_photo_truthfulness(self):
        for value in ['anything','1:1:1','2:100000:0']:
            with self.assertRaises(ValueError):walk.parse_node(value)
        with self.assertRaises(ValueError):walk.snapshot(walk.parse_node(walk.start()['node']),float('nan'))
        for photo in walk.photo_views():self.assertFalse(photo['is_panorama'])
        s=walk.start()
        if s['photo']:self.assertFalse(s['photo']['is_panorama']);self.assertLessEqual(s['photo']['heading_delta_deg'],55)

    def test_elevator_policy_reaches_music_without_stairs(self):
        node=tuple(walk.places()['outside_start']['navigation_node']);kinds=[]
        for _ in range(100):
            s=walk.snapshot(node,0,'classical_music',include_photo=False,policy='elevator')
            self.assertEqual(s['policy'],'elevator')
            if s['arrived']:break
            option=next(o for o in s['options'] if o['id']=='recommended');kinds.append(option['kind'])
            if option['kind']=='elevator':
                transition=walk.transition_for_option(option,'classical_music',policy='elevator')
                interior=[f for f in transition['samples'] if f['level']=='elevator']
                self.assertTrue(interior)
                self.assertTrue(all(f['photo'] is None and f['nearby_view'] is None and f['notice'] for f in interior))
            node=walk.parse_node(option['destination_node'])
        else:self.fail('엘리베이터 후보 경로가 목적지까지 연결되지 않았습니다.')
        self.assertIn('elevator',kinds);self.assertNotIn('stairs',kinds)

    def test_health_room_arrival_faces_the_sign_once_and_keeps_manual_look(self):
        start=walk.start('classical_music','health_room')
        option=next(o for o in start['options'] if o['recommended'])
        result=walk.transition_for_option(option,'health_room')
        arrival=result['next'];place=walk.places()['health_room']
        self.assertTrue(arrival['arrived'])
        self.assertAlmostEqual(arrival['yaw'],place['arrival_heading'])
        self.assertIn('공동 출입구',arrival['arrival_note'])
        self.assertIsNotNone(arrival['photo'])
        self.assertEqual(arrival['level'],'2')
        look=walk.snapshot(tuple(place['navigation_node']),0,'health_room',include_photo=False)
        self.assertEqual(look['yaw'],0)

if __name__=='__main__':unittest.main(verbosity=2)
