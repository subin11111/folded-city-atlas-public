#!/usr/bin/env python3
"""Retain explicit real replay/cartography/video evidence for repository docs."""
import argparse
import json
from pathlib import Path
import shutil

from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True);p.add_argument('--video',type=Path,required=True)
    p.add_argument('--cartographic',type=Path,required=True);a=p.parse_args()
    items=[(a.video/'mapping_story.mp4','mapping_story.mp4'),(a.video/'manifest.json','video_manifest.json'),
        (a.replay/'manifest.json','replay_manifest.json'),
        (a.replay/'independent_audit.json','replay_audit.json'),
        (a.video/'actual_dds_compact.png','actual_dds_compact.png'),
        (a.video/'actual_dds_compact.dot','actual_dds_compact.dot')]
    for name in ('rqt_native_window.png','rqt_graph.png','rqt_graph.svg','rqt_graph.dot','ros_graph_inventory.json'):
        items.append((a.replay/'graph_delivery'/name,name))
    for name in ('campus_f1.png','indoor_f1.png','indoor_f2.png','indoor_f1.svg','indoor_f2.svg','map_vectors.json','campus_osm.geojson','indoor_coarse.geojson','osm_source.xml'):
        items.append((a.cartographic/name,name))
    items.append((a.cartographic/'manifest.json','cartographic_manifest.json'))
    for node,name in [(7,'outside'),(24,'lobby'),(36,'bridge_out1f'),(60,'floor2'),(71,'bridge_12_sub')]:
        items.append((a.video/'stills'/f'{node:04d}_{name}.png',f'replay_{name}.png'))
    if not all(p.is_file() for p,_ in items):raise FileNotFoundError([str(p) for p,_ in items if not p.is_file()])
    a.output.mkdir(parents=True,exist_ok=False);records=[]
    for source,name in items:
        target=a.output/name;shutil.copy2(source,target)
        assert sha256_file(target)==sha256_file(source)
        records.append({'file':name,'source':str(source),'sha256':sha256_file(target),'bytes':target.stat().st_size})
    (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'files':records,
        'all_files_copied_without_image_retouching':True,'video_scope':'edited causal recorded-data replay + historical offline finalization',
        'public_release_ready':False,'OSM_attribution':'© OpenStreetMap contributors; ODbL; https://www.openstreetmap.org/copyright'},indent=2)+'\n')
    print('DEMONSTRATION_EVIDENCE_PUBLISHED '+str(a.output),flush=True)


if __name__=='__main__':main()
