#!/usr/bin/env python3
"""Actual matched TSDF/point-cloud comparison, optionally in a native GUI.

Old polished/inferred surfaces are deliberately not mixed into this controlled
comparison. Other four sessions use unchanged original-pose depth reconstructions.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

from check_session_overlap import Session
from evaluate_submap_candidates import fixed_patches
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now
from surface_model_utils import decode_frame

ROOT = Path(__file__).resolve().parents[1]


def paths_for(rebuild, code, other=True, points=False):
    paths = {'floor2': rebuild/code/('depth_preview.ply' if points else 'tsdf_5cm.ply')}
    if other:
        for name in ['outside', 'lobby', 'bridge_out1f', 'bridge_12_sub']:
            paths[name] = ROOT/'outputs/student_union_preserved_display_v1'/name/'preserved_geometry.ply'
    return paths


def add_scene(o3d, scene, paths, *, neutral=False, points=False):
    scene.clear_geometry(); lows, highs = [], []
    for name, path in paths.items():
        is_cloud = points and name == 'floor2'
        geometry = o3d.io.read_point_cloud(str(path)) if is_cloud else o3d.io.read_triangle_mesh(str(path))
        if not is_cloud:
            geometry.compute_vertex_normals()
            if neutral: geometry.paint_uniform_color([.62, .62, .62])
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = 'defaultLit' if neutral and not is_cloud else 'defaultUnlit'
        material.sRGB_color = True; material.point_size = 2.
        scene.add_geometry(name, geometry, material)
        lows.append(geometry.get_min_bound()); highs.append(geometry.get_max_bound())
    return o3d.geometry.AxisAlignedBoundingBox(np.min(lows, 0), np.max(highs, 0))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rebuild', type=Path, required=True)
    p.add_argument('--poses', type=Path, required=True)
    p.add_argument('--variant', default='Z02')
    p.add_argument('--assets', type=Path)
    p.add_argument('--native', action='store_true')
    p.add_argument('--evidence', type=Path)
    args = p.parse_args()
    import open3d as o3d
    rebuild = json.loads((args.rebuild/'manifest.json').read_text())
    report = json.loads((args.poses/'report.json').read_text())
    mp = ROOT/report['config']['source_manifest']; selected = json.loads(mp.read_text())
    assert rebuild['source_selected_map_manifest_sha256'] == sha256_file(mp)
    assert report['variants'][args.variant]['guards_passed']
    db = Path(selected['database_integrity']['floor2']['database'])
    pp = {'J04': load_pose_csv(mp.parent/selected['sessions']['floor2']['trajectory']),
          args.variant: load_pose_csv(args.poses/(args.variant+'.csv'))}
    profile = selected['sessions']['floor2']['preprocessing']
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    views = {n: decode_frame(con, n, profile, pp['J04'][n]) for n in [282, 642, 728, 828]}
    # Camera for the photographed inter-floor problem stays in the unchanged
    # lobby session. The render alone looks up 18 degrees, clearly labelled.
    lp = load_pose_csv(mp.parent/selected['sessions']['lobby']['trajectory'])
    ldb = Path(selected['database_integrity']['lobby']['database'])
    lc = sqlite3.connect(f'file:{ldb}?mode=ro', uri=True)
    lobby = decode_frame(lc, 713, selected['sessions']['lobby']['preprocessing'], lp[713]); lc.close()
    if args.native:
        if args.evidence is None: raise ValueError('--evidence required for native mode')
        args.evidence.mkdir(parents=True, exist_ok=False)
        from open3d.visualization import gui, rendering
        app = gui.Application.instance; app.initialize()
        window = app.create_window('Folded City | Z-only candidate / XY + rotations LOCKED', 1700, 1000)
        scene = gui.SceneWidget(); scene.scene = rendering.Open3DScene(window.renderer)
        scene.scene.set_background([.055, .065, .08, 1.]); scene.enable_scene_caching(False)
        window.add_child(scene); panel = gui.Vert(8, gui.Margins(12, 12, 12, 12)); window.add_child(panel)
        panel.add_child(gui.Label('Z ONLY / XY + ALL ROTATIONS LOCKED'))
        versions = gui.Combobox(); versions.add_item('Before: J04'); versions.add_item('After: '+args.variant); versions.selected_index = 1; panel.add_child(versions)
        representations = gui.Combobox()
        for label in ['Matched 5cm TSDF / RGB', 'Neutral geometry', 'Original depth samples / exact paired XY']:
            representations.add_item(label)
        panel.add_child(representations)
        other = gui.Checkbox('Show unchanged other four sessions'); other.checked = True; panel.add_child(other)
        overview = gui.Button('All-map overview'); panel.add_child(overview)
        buttons = {}
        for label, key in [('North corridor / fixed comparison camera', 642), ('Upper restroom / fixed comparison camera', 728), ('Inter-floor gap / lobby camera +18deg', 'lobby')]:
            button = gui.Button(label); panel.add_child(button); buttons[key] = button
        capture = gui.Button('Save comparison view + camera'); panel.add_child(capture)
        info = report['variants'][args.variant]
        panel.add_child(gui.Label('Same-level floor-centre P10-P90:'))
        panel.add_child(gui.Label(f'{report["baseline_floor"]["p90_minus_p10_m"]*100:.1f} -> {info["floor"]["p90_minus_p10_m"]*100:.1f} cm'))
        panel.add_child(gui.Label(f'Full range still {info["floor"]["maximum_minus_minimum_m"]*100:.1f} cm'))
        panel.add_child(gui.Label('Stairs / lower floor / attachment fixed.'))
        panel.add_child(gui.Label('EXPERIMENT, not a selected-map replacement.'))
        panel.add_child(gui.Label('No added ceilings / no hole filling here.'))
        panel.add_child(gui.Label('Not the earlier polished flat-sheet display.'))
        title = gui.Label('Original photo; before-camera fixed for A/B'); panel.add_child(title)
        photo = gui.ImageWidget(); panel.add_child(photo)
        status = gui.Label('Loading...'); panel.add_child(status)
        state = {'first': True, 'focus': 642, 'bounds': None, 'proof': 0}
        def layout(_):
            r = window.content_rect; panel.frame = gui.Rect(r.x, r.y, 375, r.height)
            scene.frame = gui.Rect(r.x+375, r.y, r.width-375, r.height)
        window.set_on_layout(layout)
        def focus(key):
            frame = lobby if key == 'lobby' else views[key]
            pose = frame['world_optical'].copy()
            if key == 'lobby': pose[:3, :3] = pose[:3, :3] @ Rotation.from_euler('x', 18, degrees=True).as_matrix()
            pos = pose[:3, 3]; scene.scene.camera.look_at(pos+pose[:3, 2], pos, -pose[:3, 1])
            pixels = Image.fromarray(frame['rgb']); pixels.thumbnail((320, 360))
            photo.update_image(o3d.geometry.Image(np.asarray(pixels).copy()))
            title.text = ('lobby:713 (render +18deg)' if key == 'lobby' else f'floor2:{key}')+' | original RGB'
            state['focus'] = key; scene.force_redraw()
        def proof():
            code = 'J04' if versions.selected_index == 0 else args.variant
            state['proof'] += 1; suffix = f'{state["proof"]:02d}_{code}'
            meta = {'variant': code, 'focus': state['focus'], 'other_sessions': other.checked,
                'representation': representations.selected_index, 'selected_map_replaced': False,
                'camera_model_matrix': scene.scene.camera.get_model_matrix().tolist(),
                'source_rebuild_manifest_sha256': sha256_file(args.rebuild/'manifest.json')}
            (args.evidence/(suffix+'.json')).write_text(json.dumps(meta, indent=2)+'\n')
            def receive(image):
                o3d.io.write_image(str(args.evidence/(suffix+'.png')), image)
                print('NATIVE_Z_RENDER_SAVED: '+suffix, flush=True)
            scene.scene.scene.render_to_image(receive)
        def load(*_):
            code = 'J04' if versions.selected_index == 0 else args.variant
            files = paths_for(args.rebuild, code, other.checked, representations.selected_index == 2)
            state['bounds'] = add_scene(o3d, scene.scene, files, neutral=representations.selected_index == 1,
                points=representations.selected_index == 2)
            status.text = f'{code} | {len(files)} sessions | XY/R fixed'
            if state['first']:
                state['first'] = False; scene.setup_camera(55., state['bounds'], state['bounds'].get_center()); focus(642)
            scene.force_redraw()
            (args.evidence/'native_loaded.json').write_text(json.dumps({'status': status.text, 'variant': code,
                'native_window': True, 'files': {n: {'path': str(p.resolve()), 'sha256': sha256_file(p)} for n, p in files.items()},
                'source_selected_map_replaced': False, 'xy_rotations_unchanged': True,
                'rebuild_manifest_sha256': sha256_file(args.rebuild/'manifest.json')}, indent=2)+'\n')
            print('NATIVE_Z_LOADED: '+status.text, flush=True)
        versions.set_on_selection_changed(load); representations.set_on_selection_changed(load); other.set_on_checked(load)
        for key, button in buttons.items(): button.set_on_clicked(lambda k=key: focus(k))
        def reset():
            center = state['bounds'].get_center(); extent = np.linalg.norm(state['bounds'].get_extent())
            scene.scene.camera.look_at(center, center+np.array([.7, -.9, .8])*extent*.6, [0., 0., 1.]); scene.force_redraw()
        overview.set_on_clicked(reset); capture.set_on_clicked(proof)
        def tick():
            if not state.get('started'):
                state['started'] = True; load(); state['capture_pending'] = True; return True
            if state.pop('capture_pending', False): proof(); return True
            return False
        window.set_on_tick_event(tick); app.run(); con.close(); return
    if args.assets is None: raise ValueError('--assets required for static mode')
    args.assets.mkdir(parents=True, exist_ok=False)
    font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 22)
    renderer = o3d.visualization.rendering.OffscreenRenderer(800, 800)
    renderer.scene.set_background([.055, .065, .08, 1.])
    chosen_views = [('floor2', n, views[n], 0) for n in [642, 728, 828]]+[('lobby', 713, lobby, 18)]
    renders = [[] for _ in chosen_views]; cameras = []
    for name, n, f, pitch in chosen_views:
        pose = f['world_optical'].copy(); pose[:3, :3] = pose[:3, :3] @ Rotation.from_euler('x', pitch, degrees=True).as_matrix()
        k = f['k_rgb'].copy(); k[:2] *= 800/f['rgb'].shape[0]; k[0, 2] += (800-f['rgb'].shape[1]*800/f['rgb'].shape[0])/2
        cameras.append({'session': name, 'node': n, 'world_optical': pose.tolist(), 'K': k.tolist(), 'render_upward_pitch_deg': pitch})
    for code, neutral in [('J04', False), (args.variant, False), (args.variant, True)]:
        add_scene(o3d, renderer.scene, paths_for(args.rebuild, code), neutral=neutral)
        for i, cam in enumerate(cameras):
            renderer.setup_camera(np.asarray(cam['K']), inverse(np.asarray(cam['world_optical'])), 800, 800)
            renders[i].append(np.asarray(renderer.render_to_image()).copy())
        print('Rendered actual TSDF: '+code+(' neutral' if neutral else ''), flush=True)
    figures = []
    for (name, n, f, pitch), cam, images in zip(chosen_views, cameras, renders):
        source = Image.fromarray(f['rgb']); source.thumbnail((800, 800)); panel = Image.new('RGB', (800, 800), '#e9edf0'); panel.paste(source, ((800-source.width)//2, (800-source.height)//2))
        sheet = Image.new('RGB', (3200, 900), 'white'); draw = ImageDraw.Draw(sheet)
        draw.text((14, 10), f'{name}:{n} | 같은 5cm TSDF 재생성 · X/Y/회전 고정 · Z만 보정 · 빈칸 채움 없음', font=font, fill='#172538')
        labels = ['원본 사진'+(' (렌더 시선만 위로 18도)' if pitch else ''), '이전 J04: 원본 깊이 재생성', f'{args.variant}: Z-only 재생성', f'{args.variant}: 회색 기하']
        for i, (im, label) in enumerate(zip([panel, *[Image.fromarray(v) for v in images]], labels)):
            draw.text((i*800+12, 56), label, font=font, fill='#172538'); sheet.paste(im, (i*800, 100))
        filename = f'{name}_{n}_z_only.png'; sheet.save(args.assets/filename)
        figures.append({'file': filename, 'sha256': sha256_file(args.assets/filename), 'camera': cam,
            'photo_source': {'database': str(ldb if name == 'lobby' else db), 'node': n, 'original_DB_RGB': True}})
    # Matched sample indices guarantee unchanged XY in this visualization too.
    clouds = {c: o3d.io.read_point_cloud(str(args.rebuild/c/'depth_preview.ply')) for c in ['J04', args.variant]}
    xyz = {c: np.asarray(v.points) for c, v in clouds.items()}
    assert np.array_equal(xyz['J04'][:, :2], xyz[args.variant][:, :2])
    picks = np.linspace(0, len(xyz['J04'])-1, min(160000, len(xyz['J04']))).astype(int)
    bounds = np.concatenate([v[picks] for v in xyz.values()]); fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    for col, code in enumerate(['J04', args.variant]):
        q = xyz[code][picks]; colors = np.asarray(clouds[code].colors)[picks]
        for row, (a, b) in enumerate([(0, 1), (0, 2)]):
            axes[row, col].scatter(q[:, a], q[:, b], c=colors, s=.25, linewidths=0, rasterized=True)
            axes[row, col].set(xlim=(bounds[:, a].min(), bounds[:, a].max()), ylim=(bounds[:, b].min(), bounds[:, b].max()),
                xlabel='XYZ'[a]+' (m)', ylabel='XYZ'[b]+' (m)', title=code+(' / exact same XY samples' if row == 0 else ' / Z correction only'))
            axes[row, col].set_aspect('equal'); axes[row, col].grid(alpha=.12)
    fig.suptitle('Identical original RGB-D sample indices and XY / only Z differs\nNo voxel resampling in paired point preview; no alignment refit')
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(args.assets/'paired_cloud_xy_xz.png', dpi=160); plt.close(fig)
    # Finite raw-patch close-ups; same points, axes and colours in both columns.
    raw = load_pose_csv(ROOT/'outputs/student_union_20261017_added_reviewed/floor2/raw_main_trajectory.csv')
    cache = {}; comparisons = [(620, 1025), (642, 1031), (280, 726)]
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as c:
        source_clouds = Clouds(c, None)
        for n in {n for pair in comparisons for n in pair}: cache[n] = fixed_patches(source_clouds, raw, n)['horizontal', 0]
    for a, b in comparisons:
        fig, axes = plt.subplots(2, 2, figsize=(13, 8)); world = {}
        for code in pp: world[code] = [apply(cache[n]['testing'], pp[code][n]) for n in [a, b]]
        all_points = np.concatenate([v for pair in world.values() for v in pair])
        for col, code in enumerate(pp):
            metric = next(r for r in report['patch_metrics'][code] if (r['a'], r['b'], r['kind']) == (a, b, 'horizontal'))
            for row, (i, j) in enumerate([(0, 1), (0, 2)]):
                for n, q, color in zip([a, b], world[code], ['#3183b0', '#e48a33']): axes[row, col].scatter(q[:, i], q[:, j], s=4, c=color, label=str(n))
                axes[row, col].set(xlim=(all_points[:, i].min()-.1, all_points[:, i].max()+.1), ylim=(all_points[:, j].min()-.1, all_points[:, j].max()+.1),
                    xlabel='XYZ'[i]+' (m)', ylabel='XYZ'[j]+' (m)'); axes[row, col].grid(alpha=.15)
            axes[0, col].set_title(f'{code}: gap {metric["symmetric_plane_abs_median_m"]*100:.2f}cm')
        axes[0, 0].legend(); fig.suptitle(f'{a}/{b}: same frozen original-depth points | exact XY freeze\nReused/correlated development diagnostic, not surveyed accuracy')
        fig.tight_layout(rect=(0, 0, 1, .92)); filename = f'patch_{a}_{b}_xy_xz.png'; fig.savefig(args.assets/filename, dpi=160); plt.close(fig)
        figures.append({'file': filename, 'sha256': sha256_file(args.assets/filename)})
    con.close()
    (args.assets/'manifest.json').write_text(json.dumps({'created_utc': utc_now(), 'figures': figures,
        'paired_cloud_xy_sha256': rebuild['variants']['J04']['preview_xy_sha256'],
        'rebuild_manifest_sha256': sha256_file(args.rebuild/'manifest.json'), 'renderer_sha256': sha256_file(Path(__file__)),
        'scope': 'Original photos and actual matched RGB-D reconstructions; not earlier flattened/polished surfaces. No map promotion.'}, indent=2)+'\n')


if __name__ == '__main__': main()
