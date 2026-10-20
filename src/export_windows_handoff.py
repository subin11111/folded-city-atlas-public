#!/usr/bin/env python3
"""Copy exact final meshes and small map/video deliverables into a portable ZIP.

No original DB, working environment, private tokens or intermediate maps enter.
Every exported file uses relative paths and a SHA256 inventory.
"""
import argparse
import json
from pathlib import Path
import shutil
import zipfile

import numpy as np

from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cartographic',type=Path,required=True);p.add_argument('--video',type=Path,required=True)
    p.add_argument('--zip',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    current=json.loads((ROOT/'config/current_local_view.json').read_text());manifest=ROOT/current['structural_manifest']
    if sha256_file(manifest)!=current['structural_manifest_sha256']:raise ValueError('Final map changed')
    run=json.loads(manifest.read_text());meshes={};inventory={}
    def copy(source,relative):
        dest=a.output/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        digest=sha256_file(dest)
        if digest!=sha256_file(source):raise ValueError('Copy hash mismatch')
        inventory[relative]=digest
    for name in run['sessions']:
        file=f'meshes/{name}.glb';copy(manifest.parent/name/'clean_textured.glb',file);meshes[name]=file
    copy(ROOT/'src/native_release_viewer.py','view_release.py')
    for file in ('src/run_slam_replay.py','src/slam_replay_core.py','src/optimize_session.py',
                 'src/export_rtabmap_db.py','src/rviz_cloud_publisher.py',
                 'src/capture_rqt_replay_graph.py','scripts/with_ros.sh',
                 'config/student_union_all_sessions.json','config/final_map_selection.json',
                 'config/slam_replay.rviz'):
        copy(ROOT/file,'replay_tools/'+file)
    for file in sorted(a.cartographic.iterdir()):
        if file.is_file():copy(file,'cartographic/'+file.name)
    for file in sorted(a.video.iterdir()):
        if file.is_file():copy(file,'video/'+file.name)
    selection=json.loads((ROOT/'config/final_map_selection.json').read_text())
    copy(ROOT/'config/final_map_selection.json','provenance/final_map_selection.json')
    road=json.loads((ROOT/'outputs/student_union_roadview_v1/roadview.json').read_text())['nodes']
    model=LevelPresentation(ROOT,run['presentation_config']);cameras={}
    for label,name,node in [('Bank','lobby',811),('Stair wall','lobby',445),('Restroom','lobby',713),('PRIME HALL','floor2',781)]:
        v=min((v for v in road if v['session']==name),key=lambda v:abs(v['node']-node))
        pose=np.array(v['world_optical']);pose[:3,3]=model.warp(pose[:3,3][None],name)[0]
        k=np.array(v['K']);vfov=float(np.degrees(2*np.arctan(v['height']/(2*k[1,1]))))
        cameras[label]={'world_optical':pose.tolist(),'vertical_fov_deg':vfov,'source_session':name,'source_node':v['node']}
    instructions='''# Windows에서 이어서 작업하기

## 현재 최종 지도 보기 (ROS 불필요)

1. ZIP 전체를 한 폴더에 풉니다. meshes 폴더를 따로 옮기지 마세요.
2. Python 3.10 64-bit를 준비합니다.
3. PowerShell에서 이 폴더로 이동해 아래를 실행합니다.

    py -3.10 -m venv .venv
    .\\.venv\\Scripts\\python.exe -m pip install -r requirements-viewer.txt
    .\\.venv\\Scripts\\python.exe view_release.py --check-only
    .\\.venv\\Scripts\\python.exe view_release.py

일반 지도 초안은 cartographic/index.html을 더블클릭하면 됩니다.
영상은 video/mapping_story.mp4입니다. 인터넷 지도 API 키는 필요 없습니다.
지도 자체의 개인정보 검토가 끝난 것이 아니므로 공개 배포는 별도입니다.

## 개발 역할 분리

- Windows: 장소 이름·호실·문 등록, 2D 지도·웹 UI·검색·로드뷰·서비스 개발.
- WSL2 Ubuntu 22.04 + ROS2 Humble: 기록 재생·SLAM·rqt_graph·RViz 재처리.
- 관리자 PowerShell: wsl --install -d Ubuntu-22.04 (설치 가능 목록은 wsl --list --online)
- 현재 재생 코드 사본은 replay_tools/에 포함했습니다. WSL Linux 홈에 복사하고
  그 폴더의 data/raw/rtabmap/ 아래에 원본 DB 5개를 별도 복사해야 합니다.
- ROS2 Humble과 python3-numpy/scipy/opencv/pytest/yaml/pil 등은 WSL Ubuntu에서 준비합니다.
- replay_tools/ 폴더에서 실행 예:
  ROS_DOMAIN_ID=83 bash scripts/with_ros.sh /usr/bin/python3 -s src/run_slam_replay.py --output outputs/replay_new --max-evaluations 180
- 다른 WSL 터미널에서 같은 폴더와 ROS_DOMAIN_ID=83으로 RViz/rqt를 실행합니다.
- 이 사본은 기록 재생용 최소 도구이며 전체 오프라인 마감 소스는 프로젝트에서 관리합니다.
- 원본 data/ 및 outputs/는 Git에 없으므로 clone만으로 과거 처리 결과가 복원되지 않습니다.
- 원본 DB는 이 보기용 묶음에 넣지 않았습니다. 별도 외장 SSD 등에 보존하세요.
- WSL ROS 패키지와 Python 환경은 새로 설치합니다. Ubuntu conda 폴더를 Windows에 복사하지 않습니다.
- 과거 전체 후처리의 절대경로 manifest는 Windows용으로 바꿔 쓰지 않습니다. 원본은 보존합니다.
- 이 폴더는 상대경로 보기용 export이며 완전한 SLAM 재현 환경이나 Windows 테스트 완료 증명은 아닙니다.

기본 지도: 사용자 선택 final_mesh_v9. 센서/포즈 지도, 외관 보완, 보행 그래프는 별개입니다.
정적 파이프라인 영상의 후반은 과거 오프라인·수동 검토 결과이며 실시간 자동 생성이 아닙니다.
'''
    (a.output/'WINDOWS_README.md').write_text(instructions,encoding='utf-8')
    (a.output/'requirements-viewer.txt').write_text('open3d==0.19.0\nnumpy==1.26.4\n')
    (a.output/'OPEN_MAP.bat').write_text('@echo off\r\ncd /d "%~dp0"\r\n.venv\\Scripts\\python.exe view_release.py\r\npause\r\n')
    (a.output/'OSM_LICENSE.txt').write_text('OpenStreetMap-derived map data and local-coordinate derivatives:\n'
        'Copyright OpenStreetMap contributors. Open Database License (ODbL) 1.0.\n'
        'https://opendatacommons.org/licenses/odbl/1-0/\nhttps://www.openstreetmap.org/copyright\n'
        'The original OSM XML and way IDs are retained in cartographic/.\n'
        'Original indoor captures and textures are separate materials; this notice does not grant rights to those materials.\n')
    for name in ('WINDOWS_README.md','requirements-viewer.txt','OPEN_MAP.bat','OSM_LICENSE.txt'):
        inventory[name]=sha256_file(a.output/name)
    release={'version':'student_union_final_v9_windows_handoff','created_utc':utc_now(),
        'source_display_manifest_sha256':current['structural_manifest_sha256'],
        'source_selected_map_manifest_sha256':selection['selected_map_manifest_sha256'],
        'meshes':meshes,'cameras':cameras,'files_sha256':inventory,
        'source_geometry_or_colour_changes':False,'raw_databases_included':False,
        'Windows_execution_verified':False,'navigation_ready':False,'public_release_ready':False}
    (a.output/'release.json').write_text(json.dumps(release,ensure_ascii=False,indent=2)+'\n')
    print('PORTABLE_RELEASE_EXPORTED '+str(a.output),flush=True)
    if a.zip:
        archive=a.output.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=4,allowZip64=True) as z:
            for file in sorted(a.output.rglob('*')):
                if file.is_file():z.write(file,Path(a.output.name)/file.relative_to(a.output))
        print('WINDOWS_ZIP_READY '+str(archive)+' '+sha256_file(archive),flush=True)


if __name__=='__main__':main()
