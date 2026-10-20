"""Vercel entrypoint. Same read-only routing engine as the local server."""
from pathlib import Path
import logging
import sys
from urllib.parse import parse_qs, urlsplit, urlencode

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import Handler

ENDPOINTS = frozenset(('route', 'health', 'walk/start', 'walk/options', 'walk/move'))


class handler(Handler):
    def send_json(self,status,value):
        if value.get('name')=='Folded City' and 'ok' in value:
            value={**value,'local':False,'runtime':'python_function'}
        return super().send_json(status,value)

    def do_GET(self):
        query = parse_qs(urlsplit(self.path).query)
        endpoint = query.pop('endpoint', [''])[0]
        if endpoint not in ENDPOINTS:
            self.send_json(404, {'error': '등록되지 않은 API입니다.'})
            return
        self.path = '/api/' + endpoint + '?' + urlencode(query, doseq=True)
        try:
            super().do_GET()
        except Exception:
            logging.exception('Folded City API failed: %s', endpoint)
            self.send_json(500, {'error': '지도를 처리하지 못했습니다. 서버 로그를 확인해 주세요.'})

    def do_POST(self):
        self.send_json(405, {'error': 'GET 요청만 지원합니다.'})

    def do_HEAD(self):
        # Never expose the inherited static-file handler from the function.
        self.send_response(405)
        self.end_headers()
