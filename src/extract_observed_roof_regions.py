#!/usr/bin/env python3
"""Extract actual upper-storey roof observations, not floor-footprint roof guesses."""
import argparse,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw
from build_unified_presentation import ceiling_fit,clip_halfspace
from export_rtabmap_db import load_pose_csv
from optimize_session import apply,sha256_file,utc_now
from photo_semantic_surfaces import lift_pixels
from presentation_geometry import LevelPresentation
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame,unproject

ROOT=Path(__file__).resolve().parents[1]


def bounded_plane_polygon(poly,frame,centre,normal,near=.251,far=7.95):
    k=frame['k_rgb'];rotation=frame['world_optical'][:3,:3];value=float((centre-frame['world_optical'][:3,3])@normal);sign=np.sign(value);value=abs(value)
    def denom(p):return float((np.array([(p[0]-k[0,2])/k[0,0],(p[1]-k[1,2])/k[1,1],1.])@rotation.T)@normal)*sign
    q=clip_halfspace(poly,lambda p:denom(p)-value/far)
    return clip_halfspace(q,lambda p:value/near-denom(p)) if len(q) else q


def roof_depth_mask(frame,z_range):
    local=unproject(frame['depth'],frame['k_depth']);world=apply(local.reshape(-1,3),frame['world_optical']).reshape(local.shape);valid=frame['valid'];dx=world[:,2:]-world[:,:-2];dy=world[2:]-world[:-2];normal=np.cross(dx[1:-1],dy[:,1:-1]);length=np.linalg.norm(normal,axis=2);normal/=np.maximum(length[:,:,None],1e-8)
    good=valid[1:-1,1:-1]&valid[1:-1,:-2]&valid[1:-1,2:]&valid[:-2,1:-1]&valid[2:,1:-1];good&=np.linalg.norm(dx[1:-1],axis=2)<.25;good&=np.linalg.norm(dy[:,1:-1],axis=2)<.25;good&=abs(normal[:,:,2])>.90;z=world[1:-1,1:-1,2];good&=(z>z_range[0])&(z<z_range[1])&(z>frame['world_optical'][2,3]+.35);mask=np.zeros_like(valid);mask[1:-1,1:-1]=good
    return world,mask


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/indoor_horizontal_continuity.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True);cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];source=json.loads((base/'manifest.json').read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());name=cfg['ceiling_session'];spec=selected['sessions'][name];poses=load_pose_csv(mp.parent/spec['trajectory']);db=Path(selected['database_integrity'][name]['database']);model=LevelPresentation(ROOT,source['presentation_config']);eligible=[n for n,p in poses.items() if p[2,3]>cfg['minimum_camera_z_m']];ids={eligible[i] for i in np.linspace(0,len(eligible)-1,min(len(eligible),cfg['maximum_frames'])).astype(int)};ids|={r['node'] for r in cfg['manual_roof_regions']};manual={r['node']:r for r in cfg['manual_roof_regions']};records=[];rejected=[];sampled=[]
    anchors=[v['display_joint_height_m'] for v in source['sessions']['floor2']['architectural_regions'] if v['id'] in cfg['ceiling_anchor_regions'] and v.get('accepted')];target=float(np.median(anchors));assert len(anchors)==len(cfg['ceiling_anchor_regions'])
    with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
        for number,node in enumerate(sorted(ids)):
            frame=decode_frame(con,node,{**spec['preprocessing'],'max_depth':cfg['maximum_depth_m']},poses[node]);world,mask=roof_depth_mask(frame,cfg['roof_source_z_range_m']);points=world[mask]
            if len(points)<cfg['minimum_roof_points']:
                rejected.append({'node':node,'reason':'Too few upward horizontal depth samples','points':len(points)});continue
            plane,fit=ceiling_fit(points)
            if plane is None:rejected.append({'node':node,'reason':'No coherent observed ceiling','fit':fit});continue
            centre,normal=plane;display=float(model.warp(centre[None],name)[0,2])
            if abs(display-target)>cfg['maximum_display_height_change_m']:
                rejected.append({'node':node,'reason':'Observed height is outside supported common-roof display band','height_m':display,'fit':fit});continue
            plane_mask=mask&(abs((world-centre)@normal)<.06);closed=cv2.morphologyEx(plane_mask.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));contours,_=cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);polygons=[]
            for contour in contours:
                if cv2.contourArea(contour)<20:continue
                poly=cv2.approxPolyDP(contour,1.0,True).reshape(-1,2).astype(float);poly*=np.array(frame['rgb'].shape[1::-1])/np.array(mask.shape[::-1]);polygons.append(poly)
            if node in manual:polygons.append(np.asarray(manual[node]['polygon'],float))
            boundaries=[];accepted_polygons=[]
            for poly in polygons:
                clipped=bounded_plane_polygon(poly,frame,centre,normal)
                if len(clipped)<3:continue
                xyz,t=lift_pixels(clipped,frame['k_rgb'],frame['world_optical'],centre,normal)
                if not np.isfinite(xyz).all() or not ((t>.25)&(t<8)).all():continue
                boundaries.append(xyz[:,:2].tolist());accepted_polygons.append(clipped.tolist())
            if not boundaries:continue
            record={'node':node,'fit':fit,'source_plane_centre':centre.tolist(),'source_plane_normal':normal.tolist(),'original_display_height_m':display,'target_height_m':target,'polygons_rgb':accepted_polygons,'boundaries_xy':boundaries,'K':frame['k_rgb'].tolist(),'world_optical':frame['world_optical'].tolist(),'observed_points':len(points),'manual_photo_region':node in manual};records.append(record)
            if node in manual or len(sampled)<5:
                im=Image.fromarray(frame['rgb']);draw=ImageDraw.Draw(im)
                for poly in accepted_polygons:
                    q=[tuple(v) for v in poly];draw.line(q+[q[0]],fill='cyan',width=2)
                im.save(a.output/f'original_{node:05d}_roof.png');sampled.append(node)
            if number%25==0:print(f'Roof observation {number+1}/{len(ids)}: {len(records)} accepted',flush=True)
    report={'created_utc':utc_now(),'session':name,'target_display_height_m':target,'anchor_heights_m':anchors,'attempted_nodes':len(ids),'accepted_frames':len(records),'records':records,'rejected':rejected,'inputs_sha256':{str(p.resolve()):sha256_file(p) for p in [Path(__file__),a.config,base/'manifest.json',mp,mp.parent/spec['trajectory'],db,ROOT/'src/build_unified_presentation.py',ROOT/'src/surface_model_utils.py']},'scope':'Original upward horizontal RGB-D observations and explicit photo contours only. Source poses unchanged; common display roof height anchored to previously photographed wall-roof joints, not floor-height inference.'};(a.output/'roof_observations.json').write_text(json.dumps(report,indent=2)+'\n');print(f'Accepted {len(records)}/{len(ids)} roof frames; display joint {target:.4f}m',flush=True)


if __name__=='__main__':main()
