#!/usr/bin/env python3
"""Portable final-map viewer. Run next to release.json on Windows or Linux.

No ROS, absolute Ubuntu paths, server, CUDA/PyTorch or raw captures needed.
Original textured GLBs are displayed, not converted or simplified.
"""
import argparse
import hashlib
import json
from pathlib import Path


def file_hash(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--release',type=Path)
    p.add_argument('--check-only',action='store_true');p.add_argument('--evidence',type=Path)
    a=p.parse_args();root=a.release.resolve() if a.release else Path(__file__).resolve().parent
    manifest=json.loads((root/'release.json').read_text())
    for file,digest in manifest['files_sha256'].items():
        resolved=(root/file).resolve()
        if root not in resolved.parents:raise ValueError('Unsafe release path')
        if file_hash(resolved)!=digest:raise ValueError('Missing/changed release file: '+file)
    print('RELEASE_FILES_VERIFIED: '+str(len(manifest['files_sha256'])),flush=True)
    if a.check_only:return
    import numpy as np
    import open3d as o3d
    from open3d.visualization import gui,rendering
    app=gui.Application.instance;app.initialize()
    window=app.create_window('Folded City - final v9 / portable local viewer',1400,950)
    scene=gui.SceneWidget();scene.scene=rendering.Open3DScene(window.renderer)
    scene.scene.set_background([.045,.06,.08,1.])
    scene.scene.view.set_color_grading(rendering.ColorGrading(rendering.ColorGrading.Quality.ULTRA,
                                                           rendering.ColorGrading.ToneMapping.LINEAR))
    window.add_child(scene)
    panel=gui.Vert(.5*window.theme.font_size,gui.Margins(10,10,10,10));window.add_child(panel)
    panel.add_child(gui.Label('Final map: student_union_final_mesh_v9'))
    panel.add_child(gui.Label('Original RGB textures; no relighting.'))
    panel.add_child(gui.Label('Routing / geographic alignment unverified.'))
    status=gui.Label('Loading five original models...');panel.add_child(status)
    bounds=[]
    for name,file in manifest['meshes'].items():
        model=o3d.io.read_triangle_model(str(root/file))
        for material in model.materials:material.shader='defaultUnlit'
        scene.scene.add_model(name,model)
        bounds.extend([m.mesh.get_axis_aligned_bounding_box() for m in model.meshes])
    low=np.min([b.min_bound for b in bounds],axis=0);high=np.max([b.max_bound for b in bounds],axis=0)
    centre=(low+high)/2;extent=np.linalg.norm(high-low)
    def overview():scene.setup_camera(55.,o3d.geometry.AxisAlignedBoundingBox(low,high),centre)
    reset=gui.Button('Overview');reset.set_on_clicked(overview);panel.add_child(reset)
    for name,camera in manifest['cameras'].items():
        button=gui.Button(name)
        def select(camera=camera):
            pose=np.array(camera['world_optical']);forward=pose[:3,2];up=-pose[:3,1]
            scene.setup_camera(float(camera['vertical_fov_deg']),o3d.geometry.AxisAlignedBoundingBox(low,high),pose[:3,3]+forward*3)
            scene.look_at(pose[:3,3]+forward*3,pose[:3,3],up)
        button.set_on_clicked(select);panel.add_child(button)
    def layout(context):
        rect=window.content_rect
        panel.frame=gui.Rect(rect.x,rect.y,310,rect.height)
        scene.frame=gui.Rect(rect.x+310,rect.y,rect.width-310,rect.height)
    window.set_on_layout(layout);overview();status.text='Loaded all five selected textured meshes.'
    if a.evidence:
        a.evidence.mkdir(parents=True,exist_ok=True)
        def capture():
            scene.scene.scene.render_to_image(lambda image:o3d.io.write_image(str(a.evidence/'portable_native_scene.png'),image))
            (a.evidence/'native_loaded.json').write_text(json.dumps({'sessions':list(manifest['meshes']),
                'native_window':True,'all_release_hashes_verified':True,'OS':'current execution host; Windows testing separate'},indent=2)+'\n')
        app.post_to_main_thread(window,capture)
    print('PORTABLE_NATIVE_VIEW_READY',flush=True);app.run()


if __name__=='__main__':main()
