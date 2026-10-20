#!/usr/bin/env python3
"""Loopback-only interactive map viewer, rendering actual local GLB/point clouds."""
import argparse
from http.server import BaseHTTPRequestHandler,HTTPServer
import io
import json
from pathlib import Path
from urllib.parse import urlparse,parse_qs

import numpy as np
from PIL import Image

from optimize_session import sha256_file

ROOT=Path(__file__).resolve().parents[1]
HTML=ROOT/'src/display_map_viewer.html'


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',type=int,default=8766)
    p.add_argument('--bundle',type=Path);p.add_argument('--check-only',action='store_true');a=p.parse_args()
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    if a.bundle is None:
        display=json.loads((ROOT/'config/current_display.json').read_text());a.bundle=(ROOT/display['manifest']).parent
        if sha256_file(a.bundle/'manifest.json')!=display['manifest_sha256']:
            raise ValueError('Current display manifest changed')
        if display['selected_map_manifest_sha256']!=pointer['manifest_sha256']:
            raise ValueError('Display belongs to a different map')
    selected=json.loads(mp.read_text());bundle=json.loads((a.bundle/'manifest.json').read_text())
    if bundle['selected_manifest_sha256']!=pointer['manifest_sha256']:raise ValueError('Map/display version mismatch')
    if set(bundle['sessions'])!=set(selected['sessions']):raise ValueError('All five sessions required')
    if a.check_only:
        print(json.dumps({'checked':True,'sessions':list(bundle['sessions']),
            'selected_map_replaced':False,'display_manifest':str(a.bundle/'manifest.json')}));return
    import open3d as o3d
    renderer=o3d.visualization.rendering.OffscreenRenderer(1200,800)
    renderer.scene.set_background([.07,.09,.12,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    state={'session':None,'method':None,'loaded':[],'renders':0}
    def load(session,method):
        if (session,method)==(state['session'],state['method']):return
        renderer.scene.clear_geometry();lows=[];highs=[];loaded=[]
        names=list(selected['sessions']) if session=='all' else [session]
        for name in names:
            if method=='points':
                path=mp.parent/selected['sessions'][name]['cloud'];cloud=o3d.io.read_point_cloud(str(path))
                mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultUnlit';mat.point_size=3.;mat.sRGB_color=True
                renderer.scene.add_geometry(name,cloud,mat);bounds=cloud.get_axis_aligned_bounding_box()
            else:
                if method=='preserved':path=a.bundle/name/'preserved_textured.glb'
                elif method=='poisson':path=ROOT/'outputs/student_union_surface_continuity_v1'/name/'supported_poisson_textured.glb'
                else:path=ROOT/'outputs/student_union_surface_layers_v1'/name/'photo_textured.glb'
                model=o3d.io.read_triangle_model(str(path))
                if not model.meshes:raise ValueError('Empty model')
                for material in model.materials:material.shader='defaultUnlit'
                renderer.scene.add_model(name,model)
                bb=[info.mesh.get_axis_aligned_bounding_box() for info in model.meshes]
                bounds=o3d.geometry.AxisAlignedBoundingBox(np.min([b.min_bound for b in bb],axis=0),np.max([b.max_bound for b in bb],axis=0))
            lows.append(bounds.min_bound);highs.append(bounds.max_bound)
            loaded.append({'session':name,'file':str(path),'bytes':path.stat().st_size})
        state.update(session=session,method=method,loaded=loaded,
            centre=(np.min(lows,axis=0)+np.max(highs,axis=0))/2,
            extent=float(np.linalg.norm(np.max(highs,axis=0)-np.min(lows,axis=0))))
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            q=urlparse(self.path)
            try:
                if q.path=='/':return self.respond(HTML.read_bytes(),'text/html; charset=utf-8')
                if q.path=='/status.json':
                    public={k:v for k,v in state.items() if k not in ['centre']}
                    return self.respond(json.dumps(public).encode(),'application/json')
                if q.path!='/render.png':self.send_error(404);return
                params=parse_qs(q.query);get=lambda key,default:params.get(key,[default])[0]
                session=get('session','all');method=get('method','preserved')
                if session not in ['all',*selected['sessions']] or method not in ['preserved','previous','poisson','points']:
                    self.send_error(400);return
                load(session,method)
                yaw=float(get('yaw','-55'));pitch=float(get('pitch','60'));zoom=float(get('zoom','1'))
                if not np.isfinite([yaw,pitch,zoom]).all():raise ValueError('Nonfinite camera')
                pitch=np.clip(pitch,5,88);zoom=np.clip(zoom,.15,4.)
                az=np.deg2rad(yaw);el=np.deg2rad(pitch);centre=state['centre']
                direction=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
                renderer.setup_camera(55.,centre,centre+direction*state['extent']*1.25*zoom,[0.,0.,1.])
                pixels=np.asarray(renderer.render_to_image());out=io.BytesIO();Image.fromarray(pixels).save(out,format='PNG')
                state['renders']+=1
                return self.respond(out.getvalue(),'image/png')
            except Exception as error:self.send_error(500,str(error))
        def respond(self,body,mime):
            self.send_response(200);self.send_header('Content-Type',mime)
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def log_message(self,format,*args):print(format%args,flush=True)
    server=HTTPServer(('127.0.0.1',a.port),Handler)
    print(f'READY http://127.0.0.1:{a.port} | actual local models; selected poses unchanged',flush=True)
    server.serve_forever()


if __name__=='__main__':main()
