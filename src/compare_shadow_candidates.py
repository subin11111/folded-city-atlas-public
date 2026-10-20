"""Join real, identically posed renders of baseline/rejected/new shadow colours."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from optimize_session import sha256_file, utc_now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rejected', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    previous = json.loads((args.rejected/'manifest.json').read_text())
    current = json.loads((args.candidate/'manifest.json').read_text())
    assert previous['before_manifest_sha256'] == current['before_manifest_sha256']
    args.output.mkdir(parents=True, exist_ok=False)
    font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 23)
    figures = []
    for old, new in zip(previous['figures'], current['figures']):
        assert old['camera'] == new['camera'], 'Do not compare unmatched cameras'
        camera = new['camera']
        stem = f'{camera["session"]}_{camera["node"]}'
        baseline = Image.open(args.candidate/(stem+'_before.png')).convert('RGB')
        assert np.array_equal(np.asarray(baseline), np.asarray(Image.open(args.rejected/(stem+'_before.png')).convert('RGB')))
        paths = [args.candidate/(stem+'_before.png'), args.rejected/(stem+'_after.png'), args.candidate/(stem+'_after.png')]
        sheet = Image.new('RGB', (2400, 900), 'white')
        draw = ImageDraw.Draw(sheet)
        draw.text((12, 10), stem+' | 같은 카메라·같은 기하 / 표시 색만 비교', font=font, fill='#172538')
        labels = ['기준 v5: 기존 색감', '사용자 기각 v2: 회색화·강한 암부 상승', '검토 후보 v3: 색 유지·약한 보정']
        for index, (path, label) in enumerate(zip(paths, labels)):
            draw.text((800*index+12, 55), label, font=font, fill='#172538')
            sheet.paste(Image.open(path).convert('RGB'), (800*index, 100))
        target = args.output/(stem+'_comparison.png')
        sheet.save(target)
        figures.append({'file': target.name, 'sha256': sha256_file(target), 'camera': camera,
                        'source_render_sha256': [sha256_file(p) for p in paths]})
    assert len(previous['figures']) == len(current['figures'])
    (args.output/'manifest.json').write_text(json.dumps({'created_utc': utc_now(), 'figures': figures,
        'source_review_manifests': [str((p/'manifest.json').resolve()) for p in [args.rejected, args.candidate]],
        'source_review_manifest_sha256': [sha256_file(p/'manifest.json') for p in [args.rejected, args.candidate]],
        'scope': 'Actual renders: v5 baseline, user-rejected v2, mild v3 candidate. Visual comparison is not user approval or recovered true illumination.'}, indent=2)+'\n')


if __name__ == '__main__':
    main()
