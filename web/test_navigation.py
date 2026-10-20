"""촬영 궤적을 사용하지 않는 최단 경로의 회귀 검사."""
import base64
import json
import math
import unittest
from navigation import Navigation,load_navigation,ROOT
from server import find_route

def tiny(rows,cols,free_cells,levels=('1',),portals=()):
    bits=bytearray(math.ceil(rows*cols/8))
    for r,c in free_cells:
        i=r*cols+c;bits[i//8]|=1<<(i%8)
    return Navigation({'cell_m':1.,'origin_xy':[0.,0.],'shape':[rows,cols],
        'levels':{lv:{'z':0. if lv=='1' else 4.,'walkable_bits':base64.b64encode(bits).decode()} for lv in levels},
        'portals':list(portals)})

class AlgorithmTests(unittest.TestCase):
    def test_empty_room_uses_direct_shortest_grid_route(self):
        nav=tiny(8,8,[(r,c) for r in range(8) for c in range(8)])
        r=nav.search(('1',0,0),('1',7,7))
        self.assertAlmostEqual(r['distance'],7*math.sqrt(2))

    def test_wall_requires_detour(self):
        nav=tiny(8,8,[(r,c) for r in range(8) for c in range(8) if c!=3 or r==7])
        r=nav.search(('1',0,0),('1',0,7))
        self.assertGreater(r['distance'],7)
        self.assertIn(('1',7,3),r['nodes'])
        self.assertAlmostEqual(r['distance'],nav.search(('1',0,0),('1',0,7),heuristic=False)['distance'])

    def test_diagonal_corner_cut_is_blocked(self):
        nav=tiny(2,2,[(0,0),(1,1)])
        self.assertIsNone(nav.search(('1',0,0),('1',1,1)))

    def test_same_xy_on_other_floor_is_not_adjacent(self):
        nav=tiny(2,2,[(r,c) for r in range(2) for c in range(2)],levels=('1','2'))
        self.assertIsNone(nav.search(('1',0,0),('2',0,0)))

class SiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nav=load_navigation()
        cls.places={p['id']:p for p in json.loads((ROOT/'public/data/catalog.json').read_text('utf-8'))}

    def test_astar_matches_independent_dijkstra_reference(self):
        for a,b in [('outside_start','prime_hall'),('entrance','bank'),('lobby','prime_hall')]:
            start=tuple(self.places[a]['navigation_node']);end=tuple(self.places[b]['navigation_node'])
            r=self.nav.search(start,end);reference=self.nav.search(start,end,heuristic=False)
            self.assertIsNotNone(r);self.assertAlmostEqual(r['distance'],reference['distance'],places=7)

    def test_all_floor_edges_clear_and_no_loops(self):
        r=self.nav.search(tuple(self.places['outside_start']['navigation_node']),tuple(self.places['prime_hall']['navigation_node']))
        self.assertEqual(len(r['nodes']),len(set(r['nodes'])))
        for a,b,(portal,_) in zip(r['nodes'],r['nodes'][1:],r['steps']):
            if portal:continue
            self.assertEqual(a[0],b[0]);self.assertTrue(self.nav.free(*a));self.assertTrue(self.nav.free(*b))
            dr=b[1]-a[1];dc=b[2]-a[2]
            if dr and dc:
                self.assertTrue(self.nav.free(a[0],a[1]+dr,a[2]))
                self.assertTrue(self.nav.free(a[0],a[1],a[2]+dc))

    def test_no_recording_links_are_loaded(self):
        self.assertFalse(self.nav.data['recording_trajectory_used_for_routing'])
        graph=json.loads((ROOT/'public/data/graph.json').read_text('utf-8'))
        self.assertEqual(graph['edges'],[])
        self.assertNotIn('recorded_walk',json.dumps(self.nav.data))

    def test_avoid_stairs_and_verified_mode_do_not_fabricate_route(self):
        route=find_route('outside_start','prime_hall',avoid_stairs=True)
        self.assertTrue(route['found']);self.assertTrue(route['elevator_used'])
        self.assertNotIn('stairs',[s['kind'] for s in route['segments']])
        self.assertFalse(route['field_verified'])
        self.assertFalse(find_route('outside_start','prime_hall',mode='verified')['found'])
        self.assertTrue(find_route('entrance','bank',avoid_stairs=True)['found'])

    def test_new_destinations_and_policy_minima(self):
        for goal in ['classical_music','career_office','food_market','cu']:
            a=tuple(self.places['outside_start']['navigation_node']);b=tuple(self.places[goal]['navigation_node'])
            for policy in ['shortest','main_stairs','sub_stairs','elevator']:
                route=self.nav.search(a,b,policy=policy);reference=self.nav.search(a,b,policy=policy,heuristic=False)
                self.assertIsNotNone(route);self.assertAlmostEqual(route['distance'],reference['distance'],places=7)
                self.assertFalse(find_route('outside_start',goal,policy=policy)['recording_trajectory_used'])
        music=find_route('outside_start','classical_music')
        self.assertIn('sub_stairs',[s['portal_id'] for s in music['segments']])
        self.assertLess(music['distance_m'],find_route('outside_start','classical_music',policy='main_stairs')['distance_m'])

    def test_elevator_status_and_policy_validation(self):
        elevator=next(p for p in self.nav.data['portals'] if p['kind']=='elevator')
        self.assertFalse(elevator['operation_verified']);self.assertIsNone(elevator['wait_time_s'])
        self.assertFalse(elevator['shaft_geometry_measured']);self.assertFalse(elevator['accessibility_verified'])
        with self.assertRaises(ValueError):find_route('entrance','prime_hall',policy='anything')

    def test_reverse_and_same_place(self):
        a=find_route('outside_start','prime_hall');b=find_route('prime_hall','outside_start')
        self.assertAlmostEqual(a['graph_distance_m'],b['graph_distance_m'])
        same=find_route('prime_hall','prime_hall')
        self.assertEqual(same['distance_m'],0);self.assertEqual(len(same['route']),1)

    def test_metric_and_unknown_blocking(self):
        r=find_route('outside_start','prime_hall')
        length=sum(math.dist(a['position'],b['position']) for a,b in zip(r['route'],r['route'][1:]))
        self.assertAlmostEqual(length,r['graph_distance_m'],places=3)
        self.assertGreaterEqual(length,math.dist(r['route'][0]['position'],r['route'][-1]['position']))
        with self.assertRaises(ValueError):self.nav.snap('1',[1000,1000])

if __name__=='__main__':unittest.main(verbosity=2)
