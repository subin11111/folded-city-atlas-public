"""Only the currently requested OSM viewport, identified requests, >=7-day cache.

Never uploads map data or edits OSM. This is an interactive local-view helper,
not an offline map download or a tile archive builder.
"""
import io
import json
import math
from pathlib import Path
import time
from urllib.request import Request,urlopen

import numpy as np
from PIL import Image,ImageDraw,ImageFont

from audit_sessions import horizontal_enu
from prepare_roadview_pack import enu_to_lonlat


def slippy_pixel(lat,lon,zoom):
    scale=256*2**zoom;phi=np.deg2rad(lat)
    return np.asarray([(lon+180)/360*scale,(1-np.arcsinh(np.tan(phi))/np.pi)/2*scale])


def pixel_lonlat(x,y,zoom):
    scale=256*2**zoom
    return x/scale*360-180,float(np.rad2deg(np.arctan(np.sinh(np.pi*(1-2*y/scale)))))


def current_viewport(lat,lon,cache,zoom=19,tile_template='https://tile.openstreetmap.org/{z}/{x}/{y}.png'):
    cache=Path(cache);cache.mkdir(parents=True,exist_ok=True)
    px=slippy_pixel(lat,lon,zoom);cx,cy=np.floor(px/256).astype(int);tiles=[]
    image=Image.new('RGB',(768,768),'#e8edef')
    for row,ty in enumerate(range(cy-1,cy+2)):
        for col,tx in enumerate(range(cx-1,cx+2)):
            path=cache/f'{zoom}_{tx}_{ty}.png';meta_path=path.with_suffix('.json')
            url=tile_template.format(z=zoom,x=tx,y=ty);now=time.time()
            metadata=json.loads(meta_path.read_text()) if meta_path.exists() else {}
            if not path.exists() or now-metadata.get('fetched_unix',0)>max(7*86400,metadata.get('max_age_s',0)):
                headers={'User-Agent':'FoldedCityAtlas/0.1 (user-requested local campus overlay)'}
                if metadata.get('ETag'):headers['If-None-Match']=metadata['ETag']
                if metadata.get('Last-Modified'):headers['If-Modified-Since']=metadata['Last-Modified']
                request=Request(url,headers=headers)
                try:
                    with urlopen(request,timeout=20) as response:
                        data=response.read();Image.open(io.BytesIO(data)).verify();path.write_bytes(data)
                        cc=response.headers.get('Cache-Control','');max_age=0
                        for item in cc.split(','):
                            if item.strip().startswith('max-age='):
                                try:max_age=int(item.split('=')[1])
                                except ValueError:pass
                        metadata={'url':url,'fetched_unix':now,'max_age_s':max_age,
                            'ETag':response.headers.get('ETag'),'Last-Modified':response.headers.get('Last-Modified')}
                except Exception as error:
                    if getattr(error,'code',None)==304 and path.exists():metadata['fetched_unix']=now
                    else:raise RuntimeError('OSM tile unavailable; no bulk retry: '+str(error)) from error
                meta_path.write_text(json.dumps(metadata,indent=2)+'\n')
            with Image.open(path) as tile:image.paste(tile.convert('RGB'),(col*256,row*256))
            tiles.append({'url':url,'cache_file':str(path),'zoom':zoom,'x':int(tx),'y':int(ty)})
    return image,{'zoom':zoom,'pixel_origin':[int((cx-1)*256),int((cy-1)*256)],'width':768,'height':768,
        'tiles':tiles,'attribution':'© OpenStreetMap contributors','copyright_url':'https://www.openstreetmap.org/copyright',
        'tile_usage_policy':'https://operations.osmfoundation.org/policies/tiles/',
        'usage':'Nine tiles for the user-requested current viewport only; no offline/prefetch feature'}


def placement_pixel_offset(east,north,latitude,zoom):
    metres_per_pixel=156543.03392804097*math.cos(math.radians(latitude))/(2**zoom)
    return np.array([east,-north],float)/metres_per_pixel


def overlay_routes(background,viewport,roadpack,clouds=None,offset_en_m=(0.,0.)):
    image=background.copy()
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',16)
    georef=json.loads((Path(roadpack)/'georeference.json').read_text());origin=georef['origin_latitude_deg'],georef['origin_longitude_deg']
    offset=placement_pixel_offset(*offset_en_m,origin[0],viewport['zoom'])
    geo=json.loads((Path(roadpack)/'trajectories.geojson').read_text())
    colours={'outside':'#ef8b2c','lobby':'#00a090','bridge_out1f':'#e34472','floor2':'#386de3','bridge_12_sub':'#9b4de2'}
    # Actual selected-map footprint, not a traced or forced OSM building outline.
    # Cloud coordinates stay unchanged; only this local preview is rigidly placed.
    if clouds:
        overlay=Image.new('RGBA',image.size,(0,0,0,0));pen=ImageDraw.Draw(overlay)
        rotation=np.asarray(georef['R_map_xy_to_EN']);translation=np.asarray(georef['t_map_xy_to_EN_m'])
        for name,points in clouds.items():
            points=np.asarray(points);en=points[:,:2]@rotation.T+translation
            lon,lat=enu_to_lonlat(en[:,0],en[:,1],origin)
            pixels=slippy_pixel(lat,lon,viewport['zoom']).T-viewport['pixel_origin']+offset
            pixels=np.unique(np.rint(pixels).astype(int),axis=0)
            rgb=tuple(int(colours[name][i:i+2],16) for i in (1,3,5))
            for x,y in pixels:
                if 0<=x<image.width and 105<=y<741:pen.point((int(x),int(y)),fill=rgb+(155,))
        image=Image.alpha_composite(image.convert('RGBA'),overlay).convert('RGB')
    draw=ImageDraw.Draw(image)
    for feature in geo['features']:
        coordinates=feature['geometry']['coordinates'];xy=[]
        for lon,lat in coordinates:xy.append(tuple(slippy_pixel(lat,lon,viewport['zoom'])-viewport['pixel_origin']+offset))
        draw.line(xy,fill=colours[feature['properties']['session']],width=3)
    uncertainty=georef['reported_accuracy_median_m'];metres_per_pixel=156543.03392804097*math.cos(math.radians(origin[0]))/(2**viewport['zoom'])
    centre=slippy_pixel(*origin,viewport['zoom'])-viewport['pixel_origin'];radius=uncertainty/metres_per_pixel
    draw.ellipse((centre[0]-radius,centre[1]-radius,centre[0]+radius,centre[1]+radius),outline='#e24b37',width=2)
    draw.rectangle((0,0,768,105),fill='white')
    draw.text((12,9),'선택 지도 궤적 · GNSS 기반 임시 배치',font=font,fill='#172538')
    draw.text((12,34),f'센서 보고 정확도 중앙값 {uncertainty:.1f}m · 건물 윤곽/방향 실측 정합 아님',font=font,fill='#b33a30')
    draw.text((12,59),'주황 외부 / 청록 로비 / 분홍 외부브리지 / 파랑 2층 / 보라 서브계단',font=font,fill='#172538')
    draw.text((12,82),f'수동 표시 이동: 동 {offset_en_m[0]:+.1f}m / 북 {offset_en_m[1]:+.1f}m · 원본 GNSS/3D 불변',font=font,fill='#b33a30')
    draw.rectangle((0,741,768,768),fill='white')
    draw.text((9,745),'© OpenStreetMap contributors · openstreetmap.org/copyright',font=font,fill='#172538')
    return image


def render_cached_manual_preview(root):
    """Re-render the already requested viewport; no network or extra tile requests."""
    import open3d as o3d
    root=Path(root);folder=root/'outputs/student_union_local_view_v1'
    viewport=json.loads((folder/'osm_viewport.json').read_text())
    background=Image.new('RGB',(viewport['width'],viewport['height']),'white')
    for tile in viewport['tiles']:
        xy=np.array([tile['x'],tile['y']])*256-viewport['pixel_origin']
        with Image.open(tile['cache_file']) as pixels:background.paste(pixels.convert('RGB'),tuple(xy))
    ptr=json.loads((root/'config/current_map.json').read_text());mp=root/ptr['manifest'];selected=json.loads(mp.read_text())
    clouds={n:np.asarray(o3d.io.read_point_cloud(str(mp.parent/s['cloud'])).voxel_down_sample(.15).points).copy() for n,s in selected['sessions'].items()}
    placement=json.loads((root/'config/osm_manual_preview.json').read_text())
    image=overlay_routes(background,viewport,root/'outputs/student_union_roadview_v1',clouds,(placement['east_m'],placement['north_m']))
    path=folder/'osm_manual_preview.png';image.save(path)
    (folder/'osm_manual_preview.json').write_text(json.dumps(placement,indent=2)+'\n')
    return path


if __name__=='__main__':
    print(render_cached_manual_preview(Path(__file__).resolve().parents[1]))
