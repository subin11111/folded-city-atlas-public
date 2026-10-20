#!/usr/bin/env python3
"""Original photographs and actual before/after five-session native-engine renders."""
import argparse,json,shutil
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from optimize_session import inverse,sha256_file,utc_now
from reconcile_multilevel_surfaces import remap_mask,sample_lattice

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);a=p.parse_args();a.assets.mkdir(parents=True,exist_ok=True)
    run=json.loads((a.bundle/'manifest.json').read_text());before=Path(run['previous_flat_manifest']).parent
    road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',22)
    views=[]
    for session,node in [('lobby',516),('lobby',579),('floor2',445),('floor2',512),('floor2',1050),('floor2',145),('floor2',1092)]:
        item=min([v for v in nodes if v['session']==session],key=lambda v:abs(v['node']-node));pose=np.asarray(item['world_optical']);k=np.asarray(item['K']).copy()
        k[:2]*=800/960;k[0,2]+=(800-720*800/960)/2
        views.append({'session':session,'node':item['node'],'K':k.tolist(),'world_optical':pose.tolist(),'original_photo':item['photo']})
    import open3d as o3d
    r=o3d.visualization.rendering.OffscreenRenderer(800,800);r.scene.set_background([.055,.065,.08,1.])
    r.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    rendered=[[] for _ in views]
    for mode in ['before','after','gray']:
        r.scene.clear_geometry()
        for name in run['sessions']:
            folder=(before if mode=='before' else a.bundle)/name
            if mode=='gray':
                mesh=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'));mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.65))
                mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';r.scene.add_geometry(name,mesh,mat)
                r.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            else:
                model=o3d.io.read_triangle_model(str(folder/'clean_textured.glb'))
                for mat in model.materials:mat.shader='defaultUnlit'
                r.scene.add_model(name,model)
        for i,view in enumerate(views):
            r.setup_camera(np.asarray(view['K']),inverse(np.asarray(view['world_optical'])),800,800);rendered[i].append(np.asarray(r.render_to_image()).copy())
        print('Rendered all five sessions: '+mode,flush=True)
    figures=[]
    for view,images in zip(views,rendered):
        original=Image.open(road/view['original_photo']).convert('RGB');original.thumbnail((800,800));photo=Image.new('RGB',(800,800),'#e9edf0');photo.paste(original,((800-original.width)//2,(800-original.height)//2))
        sheet=Image.new('RGB',(3200,900),'white');draw=ImageDraw.Draw(sheet);draw.text((15,10),f'{view["session"]}:{view["node"]} | 원본 시점 · 다섯 기록 함께 표시 · 포즈 불변',font=font,fill='#172538')
        for i,(image,label) in enumerate(zip([np.asarray(photo),*images],['원본 사진','이전: 층별 고정 Z / 넓은 저층 천장','수정: 관측 높이 연결 / 복층 개방','수정 기하: 사진 색 제외'])):
            draw.text((800*i+12,57),label,font=font,fill='#172538');sheet.paste(Image.fromarray(image),(800*i,100))
        file=f'{view["session"]}_{view["node"]}_comparison.png';sheet.save(a.assets/file)
        figures.append({'file':file,'sha256':sha256_file(a.assets/file),'camera':view,'source_rgb_sha256':sha256_file(road/view['original_photo']),
            'type':'original_photo_and_same_camera_actual_five_session_render'})
    # Geometric footprint evidence makes an erroneous low ceiling visible even
    # when exterior perspectives hide it. These are model masks, not a survey.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    void=np.load(a.bundle/'reviewed_atrium.npz');low=void['low'];cell=float(void['cell']);shape=void['mask'].shape;h,w=shape
    masks={};old=json.loads((before/'manifest.json').read_text())
    for label,bundle,record in [('before',before,old),('after',a.bundle,run)]:
        for kind in ['lower_ceiling','upper_roof']:
            mask=np.zeros(shape,bool)
            for name,s in record['sessions'].items():
                for plane in s['flat_sheets']:
                    if not plane.get('accepted'):continue
                    take=plane['id'] in run['topology_config']['low_ceiling_layers'] if kind=='lower_ceiling' else plane['id'] in ['floor2_ceiling_2f','sub_ceiling_2f']
                    if take:
                        pack=np.load(bundle/name/plane['mask_file']);mask|=remap_mask(pack['mask'],pack['low'],float(pack['cell']),low,cell,shape)
            masks[label+'_'+kind]=mask
    fig,axes=plt.subplots(1,3,figsize=(15,8));titles=['Before: low ceilings close the atrium','After: low ceilings exclude reviewed void','After: upper roof remains']
    for ax,title,mask in zip(axes,titles,[masks['before_lower_ceiling'],masks['after_lower_ceiling'],masks['after_upper_roof']]):
        rgb=np.ones((*shape,3))*.95;rgb[mask]=[.35,.52,.72];rgb[void['mask']&mask]=[.88,.28,.2];rgb[void['mask']&~mask]=[.33,.73,.51]
        ax.imshow(rgb,origin='lower',extent=[low[0],low[0]+w*cell,low[1],low[1]+h*cell]);ax.set_title(title);ax.set_xlim(-24,1);ax.set_ylim(-17,18);ax.set_xlabel('selected-map X (m)');ax.set_ylabel('Y (m)')
    fig.legend(handles=[Patch(color=[.35,.52,.72],label='modeled ceiling/roof'),Patch(color=[.88,.28,.2],label='surface above atrium XY'),Patch(color=[.33,.73,.51],label='reviewed atrium left open')],loc='lower center',ncol=3)
    fig.suptitle('Display topology repair: low ceiling removed, high roof retained (not surveyed boundaries)');fig.tight_layout(rect=(0,.06,1,.96));fig.savefig(a.assets/'atrium_ceiling_footprints.png',dpi=160);plt.close(fig)
    figures.append({'file':'atrium_ceiling_footprints.png','sha256':sha256_file(a.assets/'atrium_ceiling_footprints.png'),'type':'actual_masks_common_world_XY'})
    # Show the shared upper floor at actual overlapping XY, not two unrelated
    # medians. Zero gap here is a construction invariant, not accuracy evidence.
    upper={p['id']:(n,p) for n,s in run['sessions'].items() for p in s['flat_sheets'] if p.get('accepted') and p['id'] in ['floor2_floor_2f','sub_floor_2f']}
    n,p0=upper['sub_floor_2f'];pack=np.load(a.bundle/n/p0['mask_file']);yy,xx=np.nonzero(pack['mask']);xy=pack['low']+np.c_[xx+.5,yy+.5]*float(pack['cell'])
    n1,p1=upper['floor2_floor_2f'];pck=np.load(a.bundle/n1/p1['mask_file']);ij=np.floor((xy-pck['low'])/float(pck['cell'])).astype(int);x,y=ij.T;hh,ww=pck['mask'].shape;ok=(x>=0)&(x<ww)&(y>=0)&(y<hh);ii=np.flatnonzero(ok);ii=ii[pck['mask'][y[ii],x[ii]]];xy=xy[ii]
    # A near-constant-X section avoids connecting different X positions at
    # identical Y into a misleading sawtooth curve.
    reference_x=np.median(xy[:,0]);rows=np.round(xy[:,1],4);section=[]
    for row in np.unique(rows):
        group=xy[rows==row];section.append(group[np.argmin(abs(group[:,0]-reference_x))])
    xy=np.asarray(section)
    fig,axes=plt.subplots(1,2,figsize=(13,4))
    for ax,label in zip(axes,['Before: independent constant heights','After: shared local height anchors']):
        for key,color in [('floor2_floor_2f','#4478bc'),('sub_floor_2f','#d17b33')]:
            n,p0=upper[key]
            if label.startswith('Before'):z=np.full(len(xy),next(v['height_m'] for v in old['sessions'][n]['flat_sheets'] if v['id']==key))
            else:z=sample_lattice(dict(np.load(a.bundle/p0['anchor_file'])),xy)
            ax.plot(xy[:,1],z,color=color,label=key,linestyle='-' if key.startswith('floor2') else '--',linewidth=2)
        ax.set_xlabel('Y (m), near-constant-X overlapping section');ax.set_ylabel('modeled Z (m)');ax.set_title(label);ax.set_ylim(2.25,3.15);ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('New 32cm split removed by shared construction; NOT independent registration accuracy');fig.tight_layout();fig.savefig(a.assets/'shared_upper_floor_heights.png',dpi=170);plt.close(fig)
    figures.append({'file':'shared_upper_floor_heights.png','sha256':sha256_file(a.assets/'shared_upper_floor_heights.png'),'type':'same_XY_model_height_comparison_not_accuracy'})
    shutil.copyfile(a.bundle/'topology_audit.json',a.assets/'topology_audit.json')
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'manifest_sha256':sha256_file(a.bundle/'manifest.json'),
        'renderer_sha256':sha256_file(Path(__file__)),'scope':'Original photographs and actual before/after rendered geometry, all five selected sessions; no generative images.'},indent=2)+'\n')


if __name__=='__main__':main()
