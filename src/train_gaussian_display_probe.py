#!/usr/bin/env python3
"""CUDA Gaussian appearance probes with FIXED selected-map Gaussian centres.

This is intentionally not a full Kerbl/DefaultStrategy reconstruction: no camera
optimization, position updates, SH view-dependence, densification or pruning.
It tests learned RGB/opacity/scale display quality without changing the map.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree
import torch

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, sha256_file, utc_now
from render_session_report import read_ply
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--surfaces',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--sessions',nargs='+');p.add_argument('--steps',type=int,default=600)
    p.add_argument('--maximum-gaussians',type=int,default=160000)
    a=p.parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError('Activate teamkai-yolo with PYTHONNOUSERSITE=1; CUDA must be verified, not silently replaced by CPU')
    from gsplat import rasterization, export_splats
    if a.output.exists() or a.assets.exists():
        raise FileExistsError('Fresh Gaussian probe output/assets required')
    a.output.mkdir(parents=True);a.assets.mkdir(parents=True)
    torch.manual_seed(20261017);np.random.seed(20261017);torch.set_num_threads(2)
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:
        raise ValueError('Selected map changed')
    manifest=json.loads(mp.read_text());cfg=json.loads((ROOT/'config/student_union_surface_portfolio.json').read_text())
    hashes={str(path):sha256_file(path) for path in [pointer_path,mp,Path(__file__),
        ROOT/'src/surface_model_utils.py',ROOT/'scripts/run_gaussian_probe.sh']}
    package_root=ROOT/'outputs/surface_runtime/gsplat-source'
    commit=subprocess.check_output(['git','-C',str(package_root),'rev-parse','HEAD'],text=True).strip()
    summary={'started_utc':utc_now(),'torch':torch.__version__,'torch_file':torch.__file__,
        'cuda_runtime':torch.version.cuda,'gpu':torch.cuda.get_device_name(0),'cuda_available':True,
        'conda_environment':'teamkai-yolo','user_site_disabled':bool(sys.flags.no_user_site),'gsplat_commit':commit,
        'source_map_replaced':False,'navigation_ready':False,'sessions':{},
        'method':'Fixed selected-map centres; CUDA gsplat 3D Gaussian rendering; optimize RGB, opacity and bounded scale only',
        'limitations':['NOT a full 3DGS reconstruction or converged radiance field.',
            'No densification/pruning/position updates/camera updates/view-dependent SH.',
            'Audit RGB views excluded from gradient fitting, but initial points use the same capture/map: NOT independent truth.',
            'No semantic sign recognition or navigation geometry approval.']}
    (a.output/'started.json').write_text(json.dumps(summary,indent=2)+'\n')
    for name in a.sessions or manifest['sessions']:
        begin=time.monotonic();dest=a.output/name;dest.mkdir();assets=a.assets/name;assets.mkdir()
        record=manifest['sessions'][name];cloud_path=mp.parent/record['cloud'];pose_path=mp.parent/record['trajectory']
        db=Path(manifest['database_integrity'][name]['database'])
        for path in [db,cloud_path,pose_path]:hashes[str(path)]=sha256_file(path)
        if hashes[str(db)]!=manifest['database_integrity'][name]['sha256']:
            raise ValueError('Original DB hash differs')
        poses=load_pose_csv(pose_path)
        surface=json.loads((a.surfaces/name/'summary.json').read_text())
        ids=surface['texture']['camera_frames']
        # Training and audit separation fixed by sequence index, not by resulting score.
        audit_ids=[n for i,n in enumerate(ids) if i%8==7];train_ids=[n for n in ids if n not in audit_ids]
        connection=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        width,height=270,360;frames={}
        for node in ids:
            frame=decode_frame(connection,node,record['preprocessing'],poses[node])
            rgb=cv2.resize(frame['rgb'],(width,height),interpolation=cv2.INTER_AREA)/255.
            depth=cv2.resize(frame['depth'],(width,height),interpolation=cv2.INTER_NEAREST)
            k=frame['k_rgb'].copy();k[0]*=width/frame['rgb'].shape[1];k[1]*=height/frame['rgb'].shape[0]
            frames[node]={'rgb':torch.tensor(rgb,dtype=torch.float32,device='cuda'),
                'depth':torch.tensor(depth,dtype=torch.float32,device='cuda'),
                'k':torch.tensor(k[None],dtype=torch.float32,device='cuda'),
                'view':torch.tensor(inverse(frame['world_optical'])[None],dtype=torch.float32,device='cuda')}
        connection.close()
        raw=read_ply(cloud_path);xyz=np.column_stack([raw[k] for k in ['x','y','z']]);rgb=np.column_stack([raw[k] for k in ['r','g','b']])/255.
        rng=np.random.default_rng(20261017);chosen=np.sort(rng.choice(len(xyz),min(a.maximum_gaussians,len(xyz)),replace=False))
        nearest=cKDTree(xyz).query(xyz[chosen],k=2,workers=2)[0][:,1]
        initial_scale=np.clip(nearest*.6,.012,.055)
        means=torch.tensor(xyz[chosen],dtype=torch.float32,device='cuda')
        quats=torch.zeros((len(chosen),4),device='cuda');quats[:,0]=1.
        color_logits=torch.nn.Parameter(torch.logit(torch.tensor(rgb[chosen],dtype=torch.float32,device='cuda').clamp(.001,.999)))
        opacity_logits=torch.nn.Parameter(torch.full((len(chosen),),-1.38629436,device='cuda'))
        initial_log_scale=torch.tensor(np.repeat(np.log(initial_scale[:,None]),3,axis=1),dtype=torch.float32,device='cuda')
        log_scales=torch.nn.Parameter(initial_log_scale.clone())
        optimizer=torch.optim.Adam([{'params':[color_logits],'lr':.03},{'params':[opacity_logits],'lr':.03},
                                    {'params':[log_scales],'lr':.004}],eps=1e-8)
        def render(node):
            f=frames[node]
            result,alpha,_=rasterization(means,quats,log_scales.exp(),opacity_logits.sigmoid(),
                color_logits.sigmoid(),f['view'],f['k'],width,height,near_plane=.10,far_plane=8.,
                render_mode='RGB+ED',packed=True,rasterize_mode='antialiased')
            return result[0],alpha[0,:,:,0]
        probe=next(n for n in cfg['representative_frames'][name] if n in frames)
        audit=audit_ids[len(audit_ids)//2] if audit_ids else train_ids[-1]
        before={}
        with torch.no_grad():
            for node in [probe,audit]:before[node]=render(node)[0][:,:,:3].clamp(0,1).cpu().numpy()
        history=[];torch.cuda.reset_peak_memory_stats()
        for step in range(a.steps):
            node=train_ids[int(rng.integers(len(train_ids)))];f=frames[node]
            image,alpha=render(node);valid=f['depth']>.25
            colour_error=(image[:,:,:3]-f['rgb']).abs()[valid].mean()
            coverage=(1-alpha[valid]).mean()
            depth_error=(image[:,:,3]-f['depth']).abs()[valid&(alpha>.1)].clamp(max=1.).mean()
            if not torch.isfinite(depth_error):depth_error=torch.zeros((),device='cuda')
            scale_reg=((log_scales-initial_log_scale)**2).mean()
            loss=colour_error+.05*coverage+.02*depth_error+.001*scale_reg
            if not torch.isfinite(loss):raise RuntimeError('Nonfinite Gaussian training loss')
            optimizer.zero_grad();loss.backward();optimizer.step()
            with torch.no_grad():log_scales.clamp_(np.log(.008),np.log(.10))
            if step%100==0 or step==a.steps-1:
                row={'step':step+1,'training_view':node,'loss':float(loss.detach()),'l1':float(colour_error.detach()),
                     'coverage_loss':float(coverage.detach()),'depth_l1_clipped_m':float(depth_error.detach())}
                history.append(row);print(json.dumps({'gaussian':name,**row}),flush=True)
        scores=[];figure_files=[]
        with torch.no_grad():
            for node in [probe,audit]:
                rendered,alpha=render(node);f=frames[node];im=rendered[:,:,:3].clamp(0,1)
                valid=f['depth']>.25;err=(im-f['rgb'])[valid]
                mse=float((err**2).mean());score={'node':node,'role':'audit_RGB_not_used_in_gradients' if node in audit_ids else 'training_RGB',
                    'masked_rgb_psnr_db':float(-10*np.log10(max(mse,1e-12))),
                    'masked_rgb_l1':float(err.abs().mean()),'mean_alpha_valid_depth':float(alpha[valid].mean())}
                scores.append(score)
                panel=Image.new('RGB',(width*3,height+55),'white');draw=ImageDraw.Draw(panel)
                for i,(title,pixels) in enumerate([('Original RGB',f['rgb'].cpu().numpy()),
                    ('Initial Gaussian display',before[node]),('Trained fixed-centre display',im.cpu().numpy())]):
                    panel.paste(Image.fromarray((np.clip(pixels,0,1)*255).astype(np.uint8)),(i*width,50));draw.text((i*width+5,30),title,fill='black')
                draw.text((6,8),name+' #'+str(node)+' | '+score['role']+' | fixed selected geometry',fill='black')
                path=assets/f'gaussian_{node}.png';panel.save(path);figure_files.append({'file':path.name,'sha256':sha256_file(path)})
            sh0=((color_logits.sigmoid()-.5)/.28209479177387814)[:,None,:]
            shN=torch.empty((len(chosen),0,3),device='cuda')
            export_splats(means,log_scales,quats,opacity_logits,sh0,shN,format='ply',save_to=str(dest/'gaussians.ply'))
            export_splats(means,log_scales,quats,opacity_logits,sh0,shN,format='splat',save_to=str(dest/'gaussians.splat'))
            torch.save({'means':means.cpu(),'log_scales':log_scales.cpu(),'quats':quats.cpu(),
                'opacity_logits':opacity_logits.cpu(),'rgb_logits':color_logits.cpu(),'fixed_centres':True},dest/'checkpoint.pt')
        result={'session':name,'gaussians':len(chosen),'steps':a.steps,'width':width,'height':height,
            'train_view_ids':train_ids,'audit_view_ids':audit_ids,'history':history,'scores':scores,'figures':figure_files,
            'centre_position_updates':0,'camera_pose_updates':0,'density_control':False,'view_dependent_SH':False,
            'maximum_allocated_GPU_bytes':torch.cuda.max_memory_allocated(),'seconds':time.monotonic()-begin,
            'outputs':{str(path.name):{'bytes':path.stat().st_size,'sha256':sha256_file(path)} for path in dest.iterdir()}}
        summary['sessions'][name]=result
        (dest/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        (assets/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        del optimizer,color_logits,opacity_logits,log_scales,means,quats,frames
        torch.cuda.empty_cache()
    summary.update(finished_utc=utc_now(),inputs_sha256=hashes,
        all_inputs_unchanged=all(sha256_file(Path(path))==h for path,h in hashes.items()))
    if not summary['all_inputs_unchanged']:raise ValueError('Original input changed during Gaussian probe')
    (a.output/'manifest.json').write_text(json.dumps(summary,indent=2)+'\n')
    (a.assets/'manifest.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'complete':True,'sessions':list(summary['sessions']),'cuda':True}),flush=True)


if __name__=='__main__':main()
