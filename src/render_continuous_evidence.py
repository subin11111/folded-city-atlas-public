#!/usr/bin/env python3
"""Original photographs, fixed-camera models, and explicit plane support masks."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, sha256_file, utc_now
from surface_model_utils import decode_frame
from audit_display_depth import depth_image, compare_depths

ROOT = Path(__file__).resolve().parents[1]


def fit(image, width, height, background='#11151a'):
    image = image.copy(); image.thumbnail((width, height))
    canvas = Image.new('RGB', (width, height), background)
    canvas.paste(image, ((width-image.width)//2, (height-image.height)//2))
    return canvas


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--before', type=Path, required=True)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--sessions', nargs='+', default=['floor2','lobby'])
    a = p.parse_args(); a.assets.mkdir(parents=True, exist_ok=True)
    import open3d as o3d
    ptr = json.loads((ROOT/'config/current_map.json').read_text())
    mp = ROOT/ptr['manifest']; selected = json.loads(mp.read_text())
    run = json.loads((a.bundle/'manifest.json').read_text())
    renderer = o3d.visualization.rendering.OffscreenRenderer(480, 640)
    renderer.scene.set_background([.055, .065, .08, 1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,
        o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 18)
    prior=a.assets/'photo_evidence.json'
    saved=json.loads(prior.read_text()) if prior.exists() else {}
    figures=saved.get('figures',[]);completed=saved.get('completed_sessions',[])
    for name in a.sessions:
        if name in completed:continue
        dest = a.assets/name; dest.mkdir(exist_ok=True)
        spec = selected['sessions'][name]
        poses = load_pose_csv(mp.parent/spec['trajectory'])
        probes = json.loads((ROOT/'outputs/student_union_preserved_display_v1'/name/'summary.json').read_text())['camera_nodes']
        if name == 'floor2':
            probes.update(stair_flight_145=145, stair_flight_1092=1092)
        con = sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro', uri=True)
        frames = {key: decode_frame(con, node, spec['preprocessing'], poses[node]) for key, node in probes.items()}
        con.close()
        panels = {key: [fit(Image.fromarray(f['rgb']), 480, 640)] for key, f in frames.items()}
        for label, folder in [('before', a.before), ('after', a.bundle), ('neutral', a.bundle)]:
            renderer.scene.clear_geometry()
            if label == 'neutral':
                mesh = o3d.io.read_triangle_mesh(str(folder/name/'clean_structural.ply'))
                mesh.compute_vertex_normals()
                mesh.vertex_colors = o3d.utility.Vector3dVector(np.full((len(mesh.vertices), 3), .6))
                material = o3d.visualization.rendering.MaterialRecord(); material.shader = 'defaultLit'
                renderer.scene.add_geometry('map', mesh, material)
                renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS, [.4, -.5, -1.])
            else:
                model = o3d.io.read_triangle_model(str(folder/name/'clean_textured.glb'))
                for material in model.materials: material.shader = 'defaultUnlit'
                renderer.scene.add_model('map', model)
            for key, frame in frames.items():
                k = frame['k_rgb'].copy(); h, w = frame['rgb'].shape[:2]
                k[0] *= 480/w; k[1] *= 640/h
                renderer.setup_camera(k, inverse(frame['world_optical']), 480, 640)
                panels[key].append(Image.fromarray(np.asarray(renderer.render_to_image())))
        before_mesh=o3d.io.read_triangle_mesh(str(a.before/name/'clean_structural.ply'))
        after_mesh=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        for key, images in panels.items():
            sheet = Image.new('RGB', (1920, 720), 'white'); draw = ImageDraw.Draw(sheet)
            frame = frames[key]
            draw.text((10, 8), f'{name} #{frame["id"]} | 원본 RGB와 동일한 카메라 | 문·개구부·계단 육안 비교', font=font, fill='#172538')
            for i, (title, image) in enumerate(zip(['원본 사진', '직전 강한 평면 모델', '연속 면 + 원본 사진 텍스처', '색을 뺀 새 기하'], images)):
                draw.text((480*i+8, 43), title, font=font, fill='#172538'); sheet.paste(image, (480*i, 80))
            path = dest/f'photo_{key}.png'; sheet.save(path)
            figures.append({'file': str(path.relative_to(a.assets)), 'sha256': sha256_file(path), 'session': name,
                'node': frame['id'], 'K_rgb': frame['k_rgb'].tolist(), 'world_optical': frame['world_optical'].tolist(),
                'paired_depth':compare_depths(depth_image(o3d,before_mesh,frame),depth_image(o3d,after_mesh,frame),frame['depth'])})
        planes = run['sessions'][name]['continuous_planes']
        masks = list((a.bundle/name).glob('mask_*.npz'))
        # Largest three modeled walls: preserve the distinction between observed,
        # measured gap completion, free space, and unknown/unmodeled regions.
        masks.sort(key=lambda path: -int(np.load(path)['mask'].sum()))
        for path in masks[:3]:
            data = np.load(path); mask = data['mask']; observed = data['observed']
            support, free = data['support'], data['free']; index = path.stem.split('_')[-1]
            texture = Image.open(a.bundle/name/f'continuous_{index}.png').convert('RGB')
            cover = np.asarray(Image.open(a.bundle/name/f'coverage_{index}.png'))
            coverage = np.zeros((*cover.shape, 3), np.uint8); coverage[cover == 255] = [35, 170, 105]; coverage[cover == 80] = [190, 135, 45]
            evidence = np.zeros((*mask.shape, 3), np.uint8)
            evidence[observed] = [175, 175, 175]
            evidence[mask & ~observed] = [35, 170, 105]
            evidence[(free >= 2) & (free >= 1.5*(support+1))] = [50, 135, 230]
            evidence[observed & ~mask] = [210, 80, 90]
            sheet = Image.new('RGB', (1920, 620), 'white'); draw = ImageDraw.Draw(sheet)
            draw.text((10, 10), f'{name} plane {index} | 회색: 관측 / 초록: 깊이 지지 보완 / 파랑: 빈 공간 / 빨강: 제외된 관측', font=font, fill='#172538')
            for i, (title, im) in enumerate([('원본 사진에서 만든 벽 텍스처', texture), ('초록: 원본 사진 / 황색: 점군 색', Image.fromarray(coverage)), ('기하 생성 근거와 제외 영역', Image.fromarray(evidence).resize(texture.size, Image.NEAREST))]):
                draw.text((640*i+10, 45), title, font=font, fill='#172538'); sheet.paste(fit(im, 640, 540), (640*i, 80))
            target = dest/f'plane_evidence_{index}.png'; sheet.save(target)
            figures.append({'file': str(target.relative_to(a.assets)), 'sha256': sha256_file(target), 'mask_sha256': sha256_file(path)})
        print('Saved original-photo and opening evidence: '+name, flush=True)
        completed.append(name)
        prior.write_text(json.dumps({'created_utc': utc_now(), 'figures': figures,'completed_sessions':completed,
            'source_sha256': sha256_file(Path(__file__)), 'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'),
            'scope': 'Actual source photographs and renders; opening masks are geometric evidence, not semantic labels.'}, indent=2)+'\n')


if __name__ == '__main__': main()
