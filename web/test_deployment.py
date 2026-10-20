"""Actual HTTP tests of the Vercel Python entrypoint, without cloud writes."""
from http.server import ThreadingHTTPServer
from threading import Thread
from urllib.error import HTTPError
from urllib.request import urlopen, Request
from urllib.parse import urlencode
import json
import unittest
from api.dispatch import handler
from navigation import ROOT


class DeploymentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),handler)
        cls.thread=Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.url=f'http://127.0.0.1:{cls.server.server_port}/api/dispatch?'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join(timeout=2)

    def get(self,endpoint,**query):
        try:
            response=urlopen(self.url+urlencode({'endpoint':endpoint,**query}),timeout=30)
        except HTTPError as error:response=error
        with response:
            self.assertEqual(response.headers['Cache-Control'],'no-store')
            raw=response.read();self.assertLess(len(raw),4_500_000)
            return response.status,json.loads(raw)

    def test_health_and_rewrite_contract(self):
        status,value=self.get('health');self.assertEqual(status,200);self.assertTrue(value['ok']);self.assertFalse(value['local'])
        config=json.loads((ROOT/'vercel.json').read_text('utf-8'))
        self.assertEqual(config['outputDirectory'],'public')
        self.assertEqual(len(config['rewrites']),5)

    def test_route_api_and_photo_walk_share_policy(self):
        status,route=self.get('route',start='outside_start',end='classical_music',policy='elevator')
        self.assertEqual(status,200);self.assertTrue(route['elevator_used'])
        status,state=self.get('walk/start',place='outside_start',goal='classical_music',policy='elevator')
        self.assertEqual(status,200);self.assertEqual(state['policy'],'elevator');self.assertIn('url',state['photo'])
        status,transition=self.get('walk/move',node=state['node'],yaw=state['yaw'],choice='recommended',goal='classical_music',policy='elevator')
        self.assertEqual(status,200);self.assertEqual(transition['next']['policy'],'elevator')
        self.assertTrue(transition['samples']);self.assertFalse(transition['recording_trajectory_used'])

    def test_invalid_and_unregistered_endpoint_cannot_serve_source(self):
        self.assertEqual(self.get('../server.py')[0],404)
        self.assertEqual(self.get('walk/start',goal='bad')[0],400)
        self.assertEqual(self.get('route',policy='bad')[0],400)
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(self.url+urlencode({'endpoint':'route'}),method='HEAD'),timeout=5)
        self.assertEqual(error.exception.code,405);error.exception.close()


if __name__=='__main__':unittest.main()
