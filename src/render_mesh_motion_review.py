"""Actual paired tiny-camera-motion renders to inspect overlap flicker visually."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

from optimize_session import inverse, sha256_file, utc_now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--views-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    run = json.loads((args.bundle/'manifest.json').read_text())
    before = Path(run['mesh_finish_source_manifest']).parent
    profiles = json.loads(args.views_manifest.read_text())['figures']
    profiles = [p['camera'] for p in profiles if (p['camera']['session'], p['camera']['node']) in [('floor2', 828), ('lobby', 445)]]
    import open3d as o3d
    renderer = o3d.visualization.rendering.OffscreenRenderer(640, 640)
    renderer.scene.set_background([.055, .065, .08, 1.])
    renderer.scene.view.set_antialiasing(True)
    renderer.scene.view.set_sample_count(4)
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA, o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    frames = [[[], []] for _ in profiles]
    angles = np.linspace(-.8, .8, 9)
    for mode, folder in enumerate([before, args.bundle]):
        renderer.scene.clear_geometry()
        for name in run['sessions']:
            model = o3d.io.read_triangle_model(str(folder/name/'clean_textured.glb'))
            for mat in model.materials:
                mat.shader = 'defaultUnlit'
            renderer.scene.add_model(name, model)
        for row, profile in enumerate(profiles):
            k = np.asarray(profile['K'])*.8
            k[2, 2] = 1
            pose = np.asarray(profile['world_optical'])
            for angle in angles:
                moved = pose.copy()
                moved[:3, :3] = pose[:3, :3]@Rotation.from_euler('y', angle, degrees=True).as_matrix()
                moved[:3, 3] += pose[:3, 0]*(angle/.8)*.01
                renderer.setup_camera(k, inverse(moved), 640, 640)
                frames[row][mode].append(np.asarray(renderer.render_to_image()).copy())
        print('Mesh motion render: '+str(mode), flush=True)
    font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 18)
    records = []
    for profile, modes in zip(profiles, frames):
        stem = f'{profile["session"]}_{profile["node"]}'
        panels = []
        for index, (left, right) in enumerate(zip(*modes)):
            sheet = Image.new('RGB', (1280, 720), 'white')
            draw = ImageDraw.Draw(sheet)
            draw.text((10, 8), f'{stem} | 같은 미세 카메라 변화 / 새 길찾기 경로 아님', font=font, fill='#172538')
            draw.text((10, 40), '정리 전', font=font, fill='#172538')
            draw.text((650, 40), '정리 후 / 벽 보완 구간 외 기존 색 유지', font=font, fill='#172538')
            sheet.paste(Image.fromarray(left), (0, 80))
            sheet.paste(Image.fromarray(right), (640, 80))
            panels.append(sheet)
        animation = args.output/(stem+'_motion.gif')
        panels[0].save(animation, save_all=True, append_images=panels[1:]+panels[-2:0:-1], duration=150, loop=0)
        contact = Image.new('RGB', (1280, 2160), 'white')
        for row, index in enumerate([0, 4, 8]):
            contact.paste(panels[index], (0, row*720))
        still = args.output/(stem+'_motion_contact.png')
        contact.save(still)
        records.append({'session': profile['session'], 'node': profile['node'], 'camera': profile,
            'yaw_degrees': angles.tolist(), 'maximum_lateral_camera_motion_m': .01,
            'animation': animation.name, 'animation_sha256': sha256_file(animation),
            'contact': still.name, 'contact_sha256': sha256_file(still)})
    (args.output/'manifest.json').write_text(json.dumps({'created_utc': utc_now(), 'sequences': records,
        'before_manifest_sha256': sha256_file(before/'manifest.json'),
        'after_manifest_sha256': sha256_file(args.bundle/'manifest.json'),
        'source_view_manifest_sha256': sha256_file(args.views_manifest), 'renderer_sha256': sha256_file(Path(__file__)),
        'scope': 'Actual paired renders under identical virtual camera perturbations. Qualitative visual review, not all-camera temporal stability, user approval or a verified walkable route.'}, indent=2)+'\n')


if __name__ == '__main__':
    main()
