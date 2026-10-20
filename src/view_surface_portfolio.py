#!/usr/bin/env python3
"""Local interactive comparison, retaining the selected map and all alternatives."""
import argparse
import json
from pathlib import Path

import numpy as np

from export_rtabmap_db import load_pose_csv

ROOT=Path(__file__).resolve().parents[1]
METHODS={'Selected point cloud':'pointcloud','TSDF 5cm':'tsdf_5cm','TSDF 3cm':'tsdf_3cm',
         'Ball pivoting':'ball_pivoting','TSDF + native photo texture':'photo_textured',
         'RGB-D photo sheets':'depth_sheet_textured','Texture + detail points':'hybrid'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=ROOT/'outputs/student_union_surface_portfolio_v1')
    p.add_argument('--layers',type=Path,default=ROOT/'outputs/student_union_surface_layers_v1')
    p.add_argument('--continuity',type=Path,help='Optional independent continuity candidates; never replaces the selected map')
    p.add_argument('--preserved',type=Path,help='Full-resolution geometry-preserving display bundle')
    a=p.parse_args()
    methods=dict(METHODS)
    if a.continuity:
        repair=json.loads((a.continuity/'manifest.json').read_text())
        methods.update({'Continuity: cleaned / larger budget':'clean_high_budget',
            'Continuity: supported Poisson RGB':'supported_poisson',
            'Continuity: Poisson + photo':'supported_poisson_textured'})
    if a.preserved:
        preserved=json.loads((a.preserved/'manifest.json').read_text())
        methods['Geometry preserving + native photo']='preserved_textured'
    import open3d as o3d
    from open3d.visualization import gui,rendering
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text())
    if a.continuity and set(repair['sessions'])!=set(selected['sessions']):
        raise ValueError('All five continuity sessions must be complete')
    if a.preserved and set(preserved['sessions'])!=set(selected['sessions']):
        raise ValueError('All five preserved-display sessions must be complete')
    if not (a.run/'manifest.json').is_file() or not (a.layers/'manifest.json').is_file():
        raise ValueError('Complete reconstructions and display layers are required')
    app=gui.Application.instance;app.initialize()
    window=app.create_window('Folded City | Surface alternatives | selected map unchanged',1550,1000)
    widget=gui.SceneWidget();widget.scene=rendering.Open3DScene(window.renderer)
    widget.scene.set_background([.06,.07,.085,1.]);window.add_child(widget)
    widget.scene.view.set_color_grading(rendering.ColorGrading(rendering.ColorGrading.Quality.ULTRA,
        rendering.ColorGrading.ToneMapping.LINEAR))
    em=window.theme.font_size
    panel=gui.Vert(.6*em,gui.Margins(em,em,em,em));window.add_child(panel)
    panel.add_child(gui.Label('DISPLAY COMPARISON'))
    session_combo=gui.Combobox();session_combo.add_item('All five sessions')
    for name in selected['sessions']:session_combo.add_item(name)
    panel.add_child(session_combo)
    method_combo=gui.Combobox()
    for label in methods:method_combo.add_item(label)
    method_combo.selected_index=len(methods)-1 if (a.continuity or a.preserved) else 4;panel.add_child(method_combo)
    panel.add_child(gui.Label('Drag: orbit | wheel: zoom'))
    panel.add_child(gui.Label('Frozen selected poses.'))
    panel.add_child(gui.Label('NOT validated navigation geometry.'))
    overview=gui.Button('Overview / reset camera');panel.add_child(overview)
    source=gui.Button('Original camera (one session)');panel.add_child(source)
    status=gui.Label('Loading...');panel.add_child(status)
    state={'bounds':None,'names':list(selected['sessions'])}
    def layout(context):
        area=window.content_rect;width=23*em
        panel.frame=gui.Rect(area.x,area.y,width,area.height)
        widget.frame=gui.Rect(area.x+width,area.y,area.width-width,area.height)
    window.set_on_layout(layout)
    def reset():
        bounds=state['bounds']
        if bounds is None:return
        center=bounds.get_center();extent=np.linalg.norm(bounds.get_extent())
        widget.setup_camera(55.,bounds,center)
        widget.scene.camera.look_at(center,center+np.array([.7,-.9,.8])*extent,[0.,0.,1.])
        widget.force_redraw()
    def load(*_):
        widget.scene.clear_geometry()
        names=list(selected['sessions']) if session_combo.selected_index==0 else [list(selected['sessions'])[session_combo.selected_index-1]]
        method=list(methods.values())[method_combo.selected_index]
        lows=[];highs=[]
        for name in names:
            record=selected['sessions'][name]
            if method=='pointcloud':
                cloud=o3d.io.read_point_cloud(str(mp.parent/record['cloud']))
                mat=rendering.MaterialRecord();mat.shader='defaultUnlit';mat.point_size=3.;mat.sRGB_color=True
                widget.scene.add_geometry(name,cloud,mat)
            else:
                filename='photo_textured' if method=='hybrid' else method
                directory=a.continuity if method in ['clean_high_budget','supported_poisson','supported_poisson_textured'] else a.layers
                if method=='preserved_textured':directory=a.preserved
                model=o3d.io.read_triangle_model(str(directory/name/(filename+'.glb')))
                for material in model.materials:material.shader='defaultUnlit'
                widget.scene.add_model(name,model)
                if method=='hybrid':
                    points=o3d.io.read_point_cloud(str(a.run/name/'detail_overlay.ply'))
                    if len(points.points):
                        mat=rendering.MaterialRecord();mat.shader='defaultUnlit';mat.point_size=2.;mat.sRGB_color=True
                        widget.scene.add_geometry(name+'_details',points,mat)
            lows.append(record['bounds_min_m']);highs.append(record['bounds_max_m'])
        state['names']=names
        state['bounds']=o3d.geometry.AxisAlignedBoundingBox(np.min(lows,axis=0),np.max(highs,axis=0))
        status.text=f'{len(names)} session(s) | {method}'
        reset()
    def source_view():
        name=state['names'][0];record=selected['sessions'][name]
        metadata=json.loads((a.run/name/'calibrated_views/transforms.json').read_text())
        frame=metadata['frames'][0]
        opengl=np.asarray(frame['transform_matrix']);vision=opengl@np.diag([1.,-1.,-1.,1.])
        position=vision[:3,3];widget.scene.camera.look_at(position+vision[:3,2],position,-vision[:3,1])
        widget.force_redraw()
    session_combo.set_on_selection_changed(load);method_combo.set_on_selection_changed(load)
    overview.set_on_clicked(reset);source.set_on_clicked(source_view)
    app.post_to_main_thread(window,load)
    print(f'READY: Surface comparison viewer; {len(methods)} alternatives; no selected map changes',flush=True)
    app.run()


if __name__=='__main__':main()
