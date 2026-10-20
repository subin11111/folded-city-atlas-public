#!/usr/bin/env python3
"""Native local 3D/photograph/OSM inspector. No web server or photo upload."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from osm_local_overlay import current_viewport,overlay_routes
from optimize_session import inverse,sha256_file

ROOT=Path(__file__).resolve().parents[1]


def photo_view_intrinsics(intrinsic, width, height, source_width=720, source_height=960):
    """Fit the full source-photo vertical/horizontal field, never crop-to-fill."""
    scale=min(width/source_width,height/source_height)
    result=np.asarray(intrinsic,dtype=float).copy()
    result[:2]*=scale
    result[0,2]+=(width-source_width*scale)/2
    result[1,2]+=(height-source_height*scale)/2
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--roadpack',type=Path,default=ROOT/'outputs/student_union_roadview_v1')
    p.add_argument('--appearance',type=Path,default=ROOT/'outputs/student_union_appearance_v2')
    current_display=json.loads((ROOT/'config/current_local_view.json').read_text())
    p.add_argument('--structural',type=Path,default=(ROOT/current_display['structural_manifest']).parent)
    p.add_argument('--conservative',type=Path,default=ROOT/'outputs/student_union_photo_finish_v9')
    p.add_argument('--evidence',type=Path,default=ROOT/'outputs/student_union_local_view_v1')
    p.add_argument('--start-view',choices=['3d','osm','indoor','atrium','restroom','bank','roof2f','floor2floor','primehall','stairwall'],default='3d')
    a=p.parse_args();a.evidence.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    from open3d.visualization import gui,rendering
    from PIL import Image
    selected_pointer=json.loads((ROOT/'config/current_map.json').read_text())
    mp=ROOT/selected_pointer['manifest']
    if sha256_file(mp)!=selected_pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text())
    roadmanifest=json.loads((a.roadpack/'manifest.json').read_text())
    if roadmanifest['inputs_sha256'].get(str(mp))!=selected_pointer['manifest_sha256']:
        raise ValueError('Photo pack belongs to a different selected map')
    road=json.loads((a.roadpack/'roadview.json').read_text())
    georef=json.loads((a.roadpack/'georeference.json').read_text())
    placement_path=ROOT/'config/osm_manual_preview.json'
    placement=json.loads(placement_path.read_text()) if placement_path.exists() else {'east_m':0.,'north_m':0.}
    names=list(selected['sessions']);groups={n:[v for v in road['nodes'] if v['session']==n] for n in names}
    semantic_path=a.structural/'semantic_objects.json'
    semantic_objects=[o for o in json.loads(semantic_path.read_text())['objects'] if o.get('geometry',{}).get('accepted') and not o.get('superseded_by_flat_horizontal_sheet')] if semantic_path.exists() else []
    state={'photo':0,'session':'floor2','osm':False,'photo_full':False,'loaded':False,'semantic_photo':None,'semantic_labels':False,'ceiling_cutaway':False}
    app=gui.Application.instance;app.initialize()
    window=app.create_window('Folded City | Continuous ceilings and floors',1700,1000)
    scene=gui.SceneWidget();scene.scene=rendering.Open3DScene(window.renderer)
    scene.enable_scene_caching(False)
    scene.scene.set_background([.055,.065,.08,1.])
    scene.scene.view.set_antialiasing(True)
    scene.scene.view.set_sample_count(4)
    scene.scene.view.set_color_grading(rendering.ColorGrading(rendering.ColorGrading.Quality.ULTRA,rendering.ColorGrading.ToneMapping.LINEAR))
    window.add_child(scene)
    em=window.theme.font_size;panel=gui.Vert(.4*em,gui.Margins(em,em,em,em));window.add_child(panel)
    panel.add_child(gui.Label('LOCAL MAP / REAL PHOTO ROADVIEW'))
    sessions=gui.Combobox();sessions.add_item('All five sessions')
    for n in names:sessions.add_item(n)
    sessions.selected_index=names.index('floor2')+1;panel.add_child(sessions)
    modes=['After: balanced RGB / NO new geometry','Before: native photo surface','Inferred faces marked pink','Selected point cloud','EXPERIMENT: inferred small-hole caps']
    structural_manifest=a.structural/'manifest.json'
    structural_complete=False
    if structural_manifest.exists():
        structural_record=json.loads(structural_manifest.read_text())
        if structural_record['selected_manifest_sha256']!=selected_pointer['manifest_sha256']:raise ValueError('Structural display belongs to another map')
        structural_complete=set(structural_record['sessions'])==set(names)
    display_policy={}
    policy_path=ROOT/'config/structural_display_policy.json'
    if structural_complete and policy_path.exists():
        policy=json.loads(policy_path.read_text())
        if policy['structural_manifest_sha256']==sha256_file(structural_manifest):display_policy=policy['sessions']
    previous_policy=structural_record.get('base_display_policy',{}).get('sessions',{}) if structural_complete else {}
    ceiling_policy_path=ROOT/'config/ceiling_completion_policy.json'
    no_inferred_ceilings=structural_complete and structural_record.get('inferred_ceilings_disabled',False)
    verified_upper_walls=structural_complete and structural_record.get('verified_upper_wall_completion',False)
    architectural_presentation=structural_complete and structural_record.get('architectural_presentation',False)
    horizontal_continuity=structural_complete and structural_record.get('horizontal_continuity_completion',False)
    presentation_model=None
    if architectural_presentation:
        from presentation_geometry import LevelPresentation
        presentation_model=LevelPresentation(ROOT,structural_record['presentation_config'])
    if ceiling_policy_path.exists() and not json.loads(ceiling_policy_path.read_text())['allow_inferred_ceiling_completion'] and not no_inferred_ceilings:
        raise ValueError('User policy forbids inferred ceiling completion; select the observed-ceilings bundle')
    if architectural_presentation:
        if not json.loads(ceiling_policy_path.read_text()).get('allow_verified_photo_bounded_ceiling_patches'):raise ValueError('Photographed ceiling patches are not enabled by policy')
        if horizontal_continuity:
            modes+=['Continuous ceilings + clean floors','Continuous surfaces / neutral geometry','Continuous roof + original floor photos','Previous split ceilings and floors']
            if structural_record.get('floor_photo_preservation'):
                modes[5]='Continuous ceilings + photographic floors'
                modes[8]='REJECTED: uniform floor materials'
            if structural_record.get('photo_sign_finish'):
                modes[5]='Photo-blended floors + complete PRIME HALL'
                modes[7]='Previous photo floors / before blending'
                modes[8]='Before photo seams and PRIME HALL repair'
            if structural_record.get('modeled_floor_material'):
                modes[5]='PREVIEW: photo-derived floor material + PRIME HALL'
                modes[7]='Photo floors: original lighting retained (v10)'
                modes[8]='Before: strong floor lighting seams (v9)'
            if structural_record.get('edge_finish'):
                modes[5]='v15 colours + softened mesh / photo boundaries'
                modes[7]='Before edge finish: preferred v15'
                modes[8]='Older photo lighting seams (v9)'
            if structural_record.get('lower_ceiling_sign_clearance'):
                modes[5]='v15 colours + edges + bank sign / ceiling clearance'
                modes[7]='Before: low bank ceiling / sign clipping (v18)'
            if structural_record.get('ceiling_expansion_and_closure'):
                if not json.loads(ceiling_policy_path.read_text()).get('allow_explicit_bounded_display_ceiling_extension'):
                    raise ValueError('Bounded display ceiling extension is not enabled by policy')
                modes[5]='Extended ceilings + closed service cavities'
                modes[7]='Before ceiling extension / service cavities (v22)'
            if structural_record.get('ceiling_material_continuity'):
                modes[5]='Matched ceiling materials + closed service cavities'
                modes[7]='Before material match: grey additions (v1)'
            if structural_record.get('shadow_lift'):
                modes[5]='Brightened shadows / unchanged geometry'
                modes[7]='Before shadow lift: accepted materials (v5)'
                if structural_record['shadow_lift']['config'].get('method')=='colour_preserving_denoised':
                    modes[5]='REVIEW: retained colour / mild shadow lift'
            if structural_record.get('final_mesh_finish'):
                modes[5]='Finished mesh / retained colours'
                modes[7]='Before mesh cleanup / same colours'
        elif structural_record.get('service_slab_completion'):
            modes+=['Closed floor slabs + bank heading','Closed slabs / neutral geometry','Photographed ceilings / bounded slab edges','Previous unified display / before slabs']
        else:modes+=['Unified 1F-2F presentation','Unified presentation / neutral geometry','Photographed ceiling zones only','Previous display / before unification']
    elif verified_upper_walls:
        if not json.loads(ceiling_policy_path.read_text()).get('allow_verified_photo_bounded_ceiling_patches'):raise ValueError('Verified local ceiling patches are not enabled by policy')
        modes+=['Verified upper walls + local joints','Upper-wall repair / neutral geometry','Verified local joints / NO global ceiling','Previous observed-only surfaces']
    elif structural_complete:modes+=['Observed ceilings only / floors kept','Observed ceilings / neutral geometry','Observed ceilings only (no completion)','Previous inferred ceilings / REJECTED']
    methods=gui.Combobox()
    for m in modes:methods.add_item(m)
    appearance_manifest=a.appearance/'manifest.json'
    state['updated']=(a.appearance/'review.json').exists()
    if not (a.appearance/'review.json').exists():
        methods.selected_index=1
    if structural_complete:methods.selected_index=5
    panel.add_child(methods)
    semantic_select=gui.Combobox();semantic_select.add_item('Choose a mapped door / glazing / sign')
    for obj in semantic_objects:semantic_select.add_item(obj['session']+': '+obj['label'])
    panel.add_child(semantic_select)
    semantic_labels=gui.Button('Show / hide semantic labels');panel.add_child(semantic_labels)
    semantic_info=gui.Label('Semantic objects: '+str(len(semantic_objects)));panel.add_child(semantic_info)
    panel.add_child(gui.Label('Presentation model; original map preserved.' if architectural_presentation else 'Frozen poses; MODELED surfaces, NOT navigation.'))
    if structural_complete and structural_record.get('modeled_floor_material'):
        panel.add_child(gui.Label('Floor appearance inferred; NOT exact source photo.'))
        panel.add_child(gui.Label('User-selected v15 material; edge-only finish.' if structural_record.get('edge_finish') else 'Visual review: softer patches/joints still remain.'))
    if structural_complete and structural_record.get('ceiling_expansion_and_closure'):
        panel.add_child(gui.Label('Ceiling/floor heights separate; service void NOT walkable.'))
    if structural_complete and structural_record.get('shadow_lift'):
        panel.add_child(gui.Label('Mesh finish; compare Before cleanup.' if structural_record.get('final_mesh_finish') else 'Shadow colour candidate; compare Before (v5).'))
    reset=gui.Button('3D overview');panel.add_child(reset)
    cutaway=gui.Button('Ceilings: show / hide (cutaway)');panel.add_child(cutaway)
    if no_inferred_ceilings:
        cutaway.text=('Open atrium / photographed ceilings only' if architectural_presentation else ('Global ceiling fill: OFF / verified joints only' if verified_upper_walls else 'Ceiling completion: OFF (observed only)'));cutaway.enabled=False
        if structural_record.get('ceiling_expansion_and_closure'):
            cutaway.text='Bounded inferred ceilings; lower atrium kept open'
    source=gui.Button('3D from current photograph');panel.add_child(source)
    capture=gui.Button('Save current problem view + camera');panel.add_child(capture)
    osm=gui.Button('Show OSM + all selected map footprints');panel.add_child(osm)
    full_photo=gui.Button('Photo roadview / full-size actual photograph');panel.add_child(full_photo)
    panel.add_child(gui.Label(f'OSM preview: E {placement["east_m"]:+.1f}m / N {placement["north_m"]:+.1f}m'))
    panel.add_child(gui.Label('Manual candidate; original GPS/3D unchanged.'))
    panel.add_child(gui.Label('No OSM edits. No photo uploads.'))
    photo_title=gui.Label('Perspective photo, NOT a 360 panorama');panel.add_child(photo_title)
    buttons=gui.Horiz(.4*em);prev=gui.Button('< Previous');nxt=gui.Button('Next >');buttons.add_child(prev);buttons.add_child(nxt);panel.add_child(buttons)
    photo=gui.ImageWidget();panel.add_child(photo)
    panel.add_child(gui.Label('Capture sequence only; connectors not verified.'))
    panel.add_child(gui.Label('LOCAL ONLY: faces/plates not yet redacted.'))
    status=gui.Label('Loading...');panel.add_child(status)
    osm_image=gui.ImageWidget();window.add_child(osm_image)
    full_image=gui.ImageWidget();window.add_child(full_image)
    bounds=None;tile_state={};label_handles=[]

    def layout(_):
        area=window.content_rect;width=410
        panel.frame=gui.Rect(area.x,area.y,width,area.height)
        right=gui.Rect(area.x+width,area.y,area.width-width,area.height)
        scene.frame=right;osm_image.frame=right;full_image.frame=right
        osm_image.visible=state['osm'];scene.visible=not(state['osm'] or state['photo_full']);full_image.visible=state['photo_full']
    window.set_on_layout(layout)

    def show_photo():
        if state['semantic_photo']:
            node=state['semantic_photo'];path=a.structural/node['session']/node['source_rgb']
            pixels=np.asarray(Image.open(path).convert('RGB').resize((270,360))).copy()
            photo.update_image(o3d.geometry.Image(pixels));full_image.update_image(o3d.io.read_image(str(path)))
            photo_title.text=node['class']+' | '+node['label'];return node
        values=groups[state['session']];state['photo']=min(max(0,state['photo']),len(values)-1);node=values[state['photo']]
        pixels=np.asarray(Image.open(a.roadpack/node['photo']).convert('RGB').resize((270,360))).copy()
        photo.update_image(o3d.geometry.Image(pixels))
        full_image.update_image(o3d.io.read_image(str(a.roadpack/node['photo'])))
        photo_title.text=f'{node["id"]} | {state["photo"]+1}/{len(values)} | perspective'
        return node

    def overview():
        if bounds is None:return
        state['osm']=False;state['photo_full']=False;window.set_needs_layout();center=bounds.get_center();extent=np.linalg.norm(bounds.get_extent())
        scene.setup_camera(55.,bounds,center)
        scene.scene.camera.look_at(center,center+np.array([.7,-.9,.8])*extent*.6,[0.,0.,1.]);scene.force_redraw()

    def camera(pitch_deg=0.):
        node=show_photo();state['osm']=False;state['photo_full']=False;window.set_needs_layout()
        pose=np.asarray(node['world_optical']).copy();pos=pose[:3,3]
        if presentation_model is not None and methods.selected_index in [5,6,7]:
            # Move only the virtual display camera; original roadview poses stay unchanged.
            pos[2]=presentation_model.warp(pos[None],node['session'])[0,2]
        if pitch_deg:
            from scipy.spatial.transform import Rotation
            pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',pitch_deg,degrees=True).as_matrix()
            photo_title.text+=f' | 3D look +{pitch_deg:g}deg; photo unchanged'
        # The overview's 55deg lens previously carried over here and cropped
        # sign tops in the wide native panel. Use the original RGB calibration,
        # fitted to contain the complete photo field just like evidence renders.
        width=int(scene.frame.width) if scene.frame.width>1 else max(1,int(window.content_rect.width)-410)
        height=int(scene.frame.height) if scene.frame.height>1 else max(1,int(window.content_rect.height))
        calibrated=photo_view_intrinsics(node.get('K',node.get('K_rgb')),width,height)
        scene.setup_camera(calibrated,inverse(pose),width,height,bounds)
        scene.force_redraw()

    def load(*_,preserve_view=False):
        nonlocal bounds
        state['osm']=False;state['photo_full']=False;window.set_needs_layout();scene.scene.clear_geometry()
        chosen=names if sessions.selected_index==0 else [names[sessions.selected_index-1]]
        if not preserve_view:state['semantic_photo']=None;semantic_select.selected_index=0
        if not preserve_view:
            state['session']=state['semantic_photo']['session'] if state['semantic_photo'] else (chosen[0] if len(chosen)==1 else 'floor2');state['photo']=0
        lows=[];highs=[];files=[]
        for n in chosen:
            s=selected['sessions'][n];mode=methods.selected_index
            if architectural_presentation and mode in [5,6,7]:s=structural_record['sessions'][n]
            if mode==3:
                path=mp.parent/s['cloud'];cloud=o3d.io.read_point_cloud(str(path))
                mat=rendering.MaterialRecord();mat.shader='defaultUnlit';mat.point_size=3.;mat.sRGB_color=True
                scene.scene.add_geometry(n,cloud,mat)
            else:
                fallback=display_policy.get(n,{}).get('method')=='balanced_only'
                if mode==6:
                    path=ROOT/'outputs/student_union_preserved_display_v1'/n/'preserved_geometry.ply' if fallback else a.structural/n/'clean_structural.ply'
                    if not path.exists():path=a.structural/n/'structural.ply'
                    if state['ceiling_cutaway'] and (a.structural/n/'cutaway_structural.ply').exists():path=a.structural/n/'cutaway_structural.ply'
                    mesh=o3d.io.read_triangle_mesh(str(path));mesh.compute_vertex_normals()
                    mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.6))
                    mat=rendering.MaterialRecord();mat.shader='defaultLit';mat.sRGB_color=True
                    scene.scene.add_geometry(n,mesh,mat)
                    scene.scene.set_lighting(rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
                    files.append(str(path));lows.append(s['bounds_min_m']);highs.append(s['bounds_max_m']);continue
                if mode==5 and fallback:path=a.appearance/n/'balanced_only.glb'
                elif mode==8:
                    previous_fallback=previous_policy.get(n,{}).get('method')=='balanced_only' if previous_policy else fallback
                    path=a.appearance/n/'balanced_only.glb' if previous_fallback else a.conservative/n/'clean_textured.glb'
                elif mode in [5,7]:
                    path=a.structural/n/'clean_textured.glb'
                    if mode==7 and horizontal_continuity and structural_record.get('floor_material_source_manifest'):
                        path=Path(structural_record['floor_material_source_manifest']).parent/n/'clean_textured.glb'
                    if mode==7 and structural_record.get('shadow_source_manifest'):
                        path=Path(structural_record['shadow_source_manifest']).parent/n/'clean_textured.glb'
                    if mode==7 and structural_record.get('mesh_finish_source_manifest'):
                        path=Path(structural_record['mesh_finish_source_manifest']).parent/n/'clean_textured.glb'
                    if not path.exists():path=a.structural/n/'structural_textured.glb'
                    if mode==5 and state['ceiling_cutaway'] and (a.structural/n/'cutaway_textured.glb').exists():path=a.structural/n/'cutaway_textured.glb'
                elif mode==1:path=ROOT/'outputs/student_union_preserved_display_v1'/n/'preserved_textured.glb'
                else:path=a.appearance/n/('inferred_marked.glb' if mode==2 else ('appearance_fixed.glb' if mode==4 else 'balanced_only.glb'))
                if not path.is_file():raise FileNotFoundError(f'Incomplete session: {path}')
                model=o3d.io.read_triangle_model(str(path))
                for material in model.materials:material.shader='defaultUnlit'
                scene.scene.add_model(n,model)
            files.append(str(path));lows.append(s['bounds_min_m']);highs.append(s['bounds_max_m'])
        bounds=o3d.geometry.AxisAlignedBoundingBox(np.min(lows,axis=0),np.max(highs,axis=0))
        if not preserve_view and state['session']=='floor2' and not state['semantic_photo']:state['photo']=min(range(len(groups['floor2'])),key=lambda i:abs(groups['floor2'][i]['node']-1055))
        show_photo()
        if not preserve_view:overview()
        else:scene.force_redraw()
        status.text=f'Loaded {len(chosen)} session(s) | {modes[methods.selected_index]}'
        if state['ceiling_cutaway'] and methods.selected_index in [5,6]:status.text+=' | new ceilings hidden'
        refresh_semantic_labels()
        state['loaded']=True
        if a.start_view in ['indoor','atrium','restroom','bank','roof2f','floor2floor','primehall','stairwall'] and not state.get('initial_indoor_view'):
            state['initial_indoor_view']=True
            if a.start_view=='atrium':
                state['session']='lobby';state['photo']=min(range(len(groups['lobby'])),key=lambda i:abs(groups['lobby'][i]['node']-516))
            if a.start_view=='restroom':
                state['session']='lobby';state['photo']=min(range(len(groups['lobby'])),key=lambda i:abs(groups['lobby'][i]['node']-713))
            if a.start_view=='bank':
                state['session']='lobby';state['photo']=min(range(len(groups['lobby'])),key=lambda i:abs(groups['lobby'][i]['node']-811))
            if a.start_view=='stairwall':
                state['session']='lobby';state['photo']=min(range(len(groups['lobby'])),key=lambda i:abs(groups['lobby'][i]['node']-445))
            if a.start_view=='roof2f':
                state['session']='floor2';state['photo']=min(range(len(groups['floor2'])),key=lambda i:abs(groups['floor2'][i]['node']-905))
            if a.start_view=='floor2floor':
                state['session']='floor2';state['photo']=min(range(len(groups['floor2'])),key=lambda i:abs(groups['floor2'][i]['node']-892))
            if a.start_view=='primehall':
                state['session']='floor2';state['photo']=min(range(len(groups['floor2'])),key=lambda i:abs(groups['floor2'][i]['node']-781))
            camera(18. if a.start_view=='restroom' else (0. if a.start_view=='bank' else (8. if a.start_view=='roof2f' else (-15. if a.start_view=='floor2floor' else 0.))))
        (a.evidence/'native_loaded.json').write_text(json.dumps({'loaded_files':files,'status':status.text,
            'native_window':True,'selected_map_changed':False,'roadview_nodes':len(road['nodes']),'time_unix':time.time(),
            'structural_manifest':str(structural_manifest),'structural_manifest_sha256':sha256_file(structural_manifest) if structural_complete else None,
            'architectural_presentation':architectural_presentation,
            'start_view':a.start_view,'ceiling_cutaway':state['ceiling_cutaway'],
            'focus_photo':{'session':state['session'],'node':groups[state['session']][state['photo']]['node']},
            'camera_model_matrix_open3d':scene.scene.camera.get_model_matrix().tolist()},indent=2)+'\n')
        print('NATIVE_LOADED: '+status.text,flush=True)

    def toggle_ceilings():
        state['ceiling_cutaway']=not state['ceiling_cutaway'];load(preserve_view=True)

    def step(delta):
        state['semantic_photo']=None;semantic_select.selected_index=0
        was_full=state['photo_full'];state['photo']+=delta;show_photo()
        if not was_full:camera()

    def refresh_semantic_labels():
        for handle in label_handles:scene.remove_3d_label(handle)
        label_handles.clear()
        if state['semantic_labels']:
            chosen=names if sessions.selected_index==0 else [names[sessions.selected_index-1]]
            for obj in semantic_objects:
                if obj['session'] in chosen:label_handles.append(scene.add_3d_label(np.array(obj['geometry']['world_centre']),obj['label']))

    def toggle_semantic_labels():
        state['semantic_labels']=not state['semantic_labels'];refresh_semantic_labels();scene.force_redraw()

    def focus_semantic(*_):
        if semantic_select.selected_index==0:return
        obj=semantic_objects[semantic_select.selected_index-1]
        if sessions.selected_index and names[sessions.selected_index-1]!=obj['session']:
            sessions.selected_index=names.index(obj['session'])+1;load()
            semantic_select.selected_index=semantic_objects.index(obj)+1
        state['session']=obj['session'];state['semantic_photo']=obj
        state['photo']=min(range(len(groups[obj['session']])),key=lambda i:abs(groups[obj['session']][i]['node']-obj['node']))
        semantic_info.text=obj['class']+' | '+obj.get('state','photo-mapped planar object')
        camera();status.text='Source photo + mapped semantic surface: '+obj['label']

    def show_full_photo():
        show_photo();state['osm']=False;state['photo_full']=True;window.set_needs_layout()
        status.text='Actual perspective photo; not a synthetic 360 panorama'

    def show_osm():
        try:
            if not tile_state:
                bg,viewport=current_viewport(georef['origin_latitude_deg'],georef['origin_longitude_deg'],a.evidence/'osm_cache')
                clouds={n:np.asarray(o3d.io.read_point_cloud(str(mp.parent/s['cloud'])).voxel_down_sample(.15).points).copy() for n,s in selected['sessions'].items()}
                image=overlay_routes(bg,viewport,a.roadpack,clouds,(placement['east_m'],placement['north_m']))
                path=a.evidence/'osm_manual_preview.png';image.save(path)
                tile_state.update(viewport=viewport,path=str(path))
                (a.evidence/'osm_viewport.json').write_text(json.dumps(viewport,indent=2)+'\n')
            image=Image.open(tile_state['path']).convert('RGB');osm_image.update_image(o3d.geometry.Image(np.asarray(image).copy()))
            state['osm']=True;state['photo_full']=False;window.set_needs_layout();status.text='OSM: actual footprints + trajectories, coarse GNSS only'
            print('OSM_NATIVE_DISPLAYED: '+tile_state['path'],flush=True)
        except Exception as error:
            status.text='OSM unavailable; local map remains available';print('OSM_ERROR: '+str(error),flush=True)

    def save_scene_image():
        def received(image):
            o3d.io.write_image(str(a.evidence/'native_scene_render.png'),image)
            state['proof_saved']=True
            print('NATIVE_RENDER_CALLBACK_SAVED',flush=True)
        scene.scene.scene.render_to_image(received)

    def save_problem_view():
        if state['osm'] or state['photo_full']:
            status.text='Switch to 3D before saving a problem view';return
        stamp=str(time.time_ns());folder=a.evidence/'problem_views';folder.mkdir(exist_ok=True)
        path=folder/stamp
        record={'camera_model_matrix_open3d':scene.scene.camera.get_model_matrix().tolist(),
            'projection_matrix_open3d':scene.scene.camera.get_projection_matrix().tolist(),
            'viewport_width':scene.frame.width,'viewport_height':scene.frame.height,
            'session_selection':sessions.selected_text,'method':modes[methods.selected_index],
            'selected_manifest_sha256':selected_pointer['manifest_sha256']}
        path.with_suffix('.json').write_text(json.dumps(record,indent=2)+'\n')
        def received(image):
            o3d.io.write_image(str(path.with_suffix('.png')),image)
            status.text='Saved exact problem view + camera in problem_views/'
        scene.scene.scene.render_to_image(received);scene.force_redraw();window.post_redraw()

    # Load all five in the final window, retain direct session filtering.
    sessions.selected_index=0
    sessions.set_on_selection_changed(load);methods.set_on_selection_changed(lambda *_:load(preserve_view=True))
    reset.set_on_clicked(overview);source.set_on_clicked(camera);prev.set_on_clicked(lambda:step(-1));nxt.set_on_clicked(lambda:step(1));osm.set_on_clicked(show_osm);full_photo.set_on_clicked(show_full_photo)
    cutaway.set_on_clicked(toggle_ceilings)
    capture.set_on_clicked(save_problem_view)
    semantic_select.set_on_selection_changed(focus_semantic);semantic_labels.set_on_clicked(toggle_semantic_labels)
    app.post_to_main_thread(window,load)
    def tick():
        if state['loaded'] and not state.get('proof'):
            state['proof']=True;save_scene_image()
        if state.get('proof_saved') and not state.get('osm_started'):
            state['osm_started']=True
            if a.start_view=='osm':show_osm()
        if not state.get('updated') and time.time()-state.get('check_time',0)>5:
            state['check_time']=time.time()
            if (a.appearance/'review.json').exists():
                state['updated']=True;methods.selected_index=0;load();state['proof']=False
        if state.get('proof') and not state.get('proof_saved'):
            scene.force_redraw()
            return True
        return False
    window.set_on_tick_event(tick)
    app.run()


if __name__=='__main__':main()
