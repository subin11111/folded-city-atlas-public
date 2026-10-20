#!/usr/bin/env python3
"""Original RGB, previous model, semantic surfaces, and gray geometry per object."""
import argparse,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from optimize_session import inverse,sha256_file,utc_now
from audit_display_depth import depth_image

ROOT=Path(__file__).resolve().parents[1]


def fit(image,width=480,height=640):
    image=image.copy();image.thumbnail((width,height));out=Image.new('RGB',(width,height),'#11151a');out.paste(image,((width-image.width)//2,(height-image.height)//2));return out


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--before',type=Path,required=True);p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--sessions',nargs='+',default=['floor2','lobby']);a=p.parse_args();a.assets.mkdir(exist_ok=True,parents=True)
    import open3d as o3d
    objects=json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    renderer=o3d.visualization.rendering.OffscreenRenderer(720,960);renderer.scene.set_background([.055,.065,.08,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',18)
    target=a.assets/'semantic_review.json';previous=json.loads(target.read_text()) if target.exists() else {};figures=previous.get('figures',[]);finished=previous.get('completed_sessions',[])
    for name in a.sessions:
        if name in finished:continue
        chosen=[o for o in objects if o['session']==name and o.get('geometry',{}).get('accepted')]
        dest=a.assets/name;dest.mkdir(exist_ok=True)
        images={o['id']:[Image.open(a.bundle/name/o['source_rgb']).convert('RGB')] for o in chosen}
        for label,folder in [('before',a.before),('after',a.bundle),('gray',a.bundle)]:
            renderer.scene.clear_geometry()
            if label=='gray':
                mesh=o3d.io.read_triangle_mesh(str(folder/name/'clean_structural.ply'));mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.6))
                material=o3d.visualization.rendering.MaterialRecord();material.shader='defaultLit';renderer.scene.add_geometry('model',mesh,material)
                renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            else:
                model=o3d.io.read_triangle_model(str(folder/name/'clean_textured.glb'))
                for material in model.materials:material.shader='defaultUnlit'
                renderer.scene.add_model('model',model)
            for o in chosen:
                renderer.setup_camera(np.asarray(o['K_rgb']),inverse(np.asarray(o['world_optical'])),720,960)
                images[o['id']].append(Image.fromarray(np.asarray(renderer.render_to_image()).copy()))
        old=o3d.io.read_triangle_mesh(str(a.before/name/'clean_structural.ply'));new=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        for o in chosen:
            poly=np.asarray(o['polygon']);low=poly.min(0);high=poly.max(0);padding=np.maximum((high-low)*.18,24)
            lo=np.maximum(np.floor(low-padding).astype(int),0);hi=np.minimum(np.ceil(high+padding).astype(int),[720,960]);crop=tuple(np.r_[lo,hi])
            sheet=Image.new('RGB',(1920,735),'white');draw=ImageDraw.Draw(sheet)
            state=o.get('state','planar photo object');draw.text((10,8),o['id']+' | '+o['class']+' | '+state,font=font,fill='#172538')
            draw.text((10,35),'같은 원본 RGB 카메라·같은 영역 / 사진과 매핑 점에 근거한 의미 면',font=font,fill='#172538')
            for i,(title,im) in enumerate(zip(['원본 RGB','이전 모델','사진 의미 면 + 경계 보정','새 기하'],images[o['id']])):
                draw.text((480*i+10,65),title,font=font,fill='#172538');sheet.paste(fit(im.crop(crop)),(480*i,95))
            path=dest/(o['id']+'.png');sheet.save(path)
            # A construction visibility check at the original RGB viewpoint,
            # including pixels with no sensor depth. It is not an accuracy test.
            frame={'depth':np.ones((240,180),np.float32),'k_depth':np.asarray(o['K_rgb']).copy(),'world_optical':np.asarray(o['world_optical'])}
            frame['k_depth'][0]/=4;frame['k_depth'][1]/=4
            before=depth_image(o3d,old,frame);after=depth_image(o3d,new,frame)
            mask=np.zeros((960,720),np.uint8);cv2.fillPoly(mask,[poly.astype(np.int32)],1);mask=cv2.resize(mask,(180,240),interpolation=cv2.INTER_NEAREST)>0
            figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'object_id':o['id'],'session':name,
                'node':o['node'],'class':o['class'],'surface_added':o['surface_added'],'photo_crop_xyxy':np.asarray(crop).astype(int).tolist(),
                'render_hit_fraction_before':float(np.isfinite(before[mask]).mean()),'render_hit_fraction_after':float(np.isfinite(after[mask]).mean()),
                'newly_hit_rgb_rays':int(np.sum(mask&~np.isfinite(before)&np.isfinite(after))),
                'scope':'Actual rendered geometry coverage at reviewed RGB polygons; not independent pose accuracy or semantic ground truth.'})
        finished.append(name)
        target.write_text(json.dumps({'created_utc':utc_now(),'figures':figures,'completed_sessions':finished,'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'renderer_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
        print('Saved semantic comparison: '+name+' / '+str(len(chosen)),flush=True)


if __name__=='__main__':main()
