#!/usr/bin/env python3
"""Audit flat-sheet height changes at the same XY and possible atrium topology."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull,Delaunay
from PIL import Image,ImageDraw,ImageFont
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def raster_medians(points,low,shape,cell):
    h,w=shape;ij=np.floor((points[:,:2]-low)/cell).astype(int);x,y=ij.T;ok=(x>=0)&(x<w)&(y>=0)&(y<h);keys=y[ok]*w+x[ok]
    order=np.argsort(keys);keys,starts,counts=np.unique(keys[order],return_index=True,return_counts=True);z=points[ok,2][order]
    result=np.full(shape,np.nan)
    for key,start,count in zip(keys,starts,counts):result.ravel()[key]=np.median(z[start:start+count])
    return result


def stats(values):
    x=np.asarray(values);x=x[np.isfinite(x)]
    return {'samples':len(x),'median_m':float(np.median(x)) if len(x) else None,'p10_m':float(np.percentile(x,10)) if len(x) else None,
        'p90_m':float(np.percentile(x,90)) if len(x) else None,'median_absolute_m':float(np.median(abs(x))) if len(x) else None}


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,default=ROOT/'outputs/student_union_flat_horizontal_v3');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());low=np.array([-36.,-31.]);shape=(255,230);cell=.2;layers={};reports={}
    for name,s in run['sessions'].items():
        mesh=o3d.io.read_triangle_mesh(s['source_mesh']);mesh.compute_triangle_normals();points=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].mean(1)
        normal=np.asarray(mesh.triangle_normals);horizontal=abs(normal[:,2])>.85
        for plane in s['flat_sheets']:
            if not plane.get('accepted'):continue
            lower,upper=plane['height_band_m'];points_on=points[horizontal&(points[:,2]>=lower)&(points[:,2]<upper)]
            old=raster_medians(points_on,low,shape,cell);grid=np.load(a.bundle/name/plane['mask_file']);mask=grid['mask'];glow=grid['low'];gcell=float(grid['cell'])
            yy,xx=np.indices(shape);world=np.c_[low[0]+(xx.ravel()+.5)*cell,low[1]+(yy.ravel()+.5)*cell];ij=np.floor((world-glow)/gcell).astype(int);x,y=ij.T;h,w=mask.shape;ok=(x>=0)&(x<w)&(y>=0)&(y<h)
            model=np.zeros(len(world),bool);ii=np.flatnonzero(ok);model[ii]=mask[y[ii],x[ii]];model=model.reshape(shape)
            layers[plane['id']]={'old':old,'model':model,'height':plane['height_m'],'kind':plane['kind'],'session':name}
            reports[plane['id']]={'flat_height_m':plane['height_m'],'height_change_at_same_XY':stats(plane['height_m']-old[model]),'model_area_on_common_grid_m2':float(model.sum()*cell*cell)}
    pairs=[('lobby_floor_1f','floor2_floor_1f'),('lobby_floor_1f','sub_floor_1f'),('floor2_floor_2f','sub_floor_2f'),('lobby_ceiling_1f','floor2_floor_2f')]
    comparisons={}
    for first,second in pairs:
        aa,bb=layers[first],layers[second];common=np.isfinite(aa['old'])&np.isfinite(bb['old'])&aa['model']&bb['model']
        comparisons[first+'__'+second]={'old_same_XY_second_minus_first':stats((bb['old']-aa['old'])[common]),
            'flat_second_minus_first_m':bb['height']-aa['height'],'scope':'Raw displayed surface medians at the same XY, not surveyed height or guaranteed matched physical surfaces.'}
    upper=layers['floor2_floor_2f'];lower=layers['lobby_floor_1f'];ceil=layers['lobby_ceiling_1f'];yy,xx=np.nonzero(upper['model']);xy=np.c_[xx,yy]
    hull=Delaunay(xy[ConvexHull(xy).vertices]);yy,xx=np.indices(shape);inside=hull.find_simplex(np.c_[xx.ravel(),yy.ravel()]).reshape(shape)>=0
    possible=inside&lower['model']&~ndimage.binary_dilation(upper['model'],iterations=2)
    candidate=possible&ceil['model'];np.savez_compressed(a.output/'multilevel_rasters.npz',low=low,cell=cell,possible_double_height=possible,ceiling_over_possible_void=candidate,
        lower_floor=lower['model'],upper_floor=upper['model'],lower_ceiling=ceil['model'],upper_source_height=upper['old'],lower_ceiling_source_height=ceil['old'])
    rgb=np.full((*shape,3),245,np.uint8);rgb[lower['model']]=[170,175,180];rgb[upper['model']]=[60,120,200];rgb[possible]=[65,175,115];rgb[candidate]=[220,85,65]
    image=Image.fromarray(np.flipud(rgb)).resize((920,1020),Image.NEAREST);sheet=Image.new('RGB',(1150,1150),'white');draw=ImageDraw.Draw(sheet);font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',21)
    draw.text((15,12),'층간 위상 점검 후보 / 현장 확정 경계가 아님',font=font,fill='#172538');draw.text((15,46),'회색 1F / 파랑 2F / 초록 상층 미지원 후보 / 빨강 그 위에 덮인 1F 천장',font=font,fill='#172538');sheet.paste(image,(100,100));sheet.save(a.output/'atrium_candidate_map.png')
    report={'created_utc':utc_now(),'planes':reports,'comparisons':comparisons,'possible_double_height_area_m2':float(possible.sum()*cell*cell),
        'first_floor_ceiling_over_candidate_area_m2':float(candidate.sum()*cell*cell),'candidate_scope':'Upper-floor envelope minus upper support; must be validated with RGB/free-space, not an automatic atrium label.',
        'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'source_sha256':sha256_file(Path(__file__))}
    (a.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
