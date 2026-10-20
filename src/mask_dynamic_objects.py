#!/usr/bin/env python3
"""Offline COCO instance masks for potentially mobile classes, never motion truth."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sqlite3
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

MODEL_URL = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt'
MOBILE_CLASSES = [0, 1, 2, 3, 5, 7, 15, 16]  # person/bicycle/car/motorcycle/bus/truck/cat/dog


def polygons_to_mask(polygons: list[np.ndarray], shape: tuple[int, int], dilation: int) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    for polygon in polygons:
        points = np.rint(polygon).astype(np.int32)
        if len(points) >= 3:
            cv2.fillPoly(mask, [points], 255)
    if dilation > 0 and np.any(mask):
        mask = cv2.dilate(mask, np.ones((2 * dilation + 1,) * 2, dtype=np.uint8))
    return mask


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('db', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--confidence', type=float, default=0.35)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--dilation', type=int, default=5, help='RGB pixels')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    import torch
    import ultralytics
    from ultralytics import YOLO

    torch.set_num_threads(args.threads)
    cv2.setNumThreads(1)
    if not args.weights.exists():
        args.weights.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(MODEL_URL, args.weights)
    model = YOLO(str(args.weights))
    args.output.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(f'file:{args.db.resolve()}?mode=ro', uri=True)
    rows = connection.execute('SELECT id, image FROM Data ORDER BY id')
    records, review_tiles = [], []
    started = time.monotonic()
    for node_id, image_blob in rows:
        image = cv2.imdecode(np.frombuffer(image_blob, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Cannot decode RGB for node {node_id}')
        result = model.predict(image, classes=MOBILE_CLASSES, conf=args.confidence,
                               imgsz=args.imgsz, device=args.device, verbose=False,
                               save=False, retina_masks=True)[0]
        polygons = result.masks.xy if result.masks is not None else []
        mask = polygons_to_mask(polygons, image.shape[:2], args.dilation)
        if not cv2.imwrite(str(args.output / f'{node_id}.png'), mask):
            raise OSError('Failed to save mask')
        classes = result.boxes.cls.cpu().numpy().astype(int).tolist() if result.boxes is not None else []
        fraction = float(np.mean(mask > 0))
        records.append({'id': node_id, 'masked_fraction': fraction, 'instances': len(classes),
                        'classes': '|'.join(model.names[c] for c in classes)})
        if fraction > 0:
            overlay = image.copy()
            overlay[mask > 0] = (0.5 * image[mask > 0] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
            tile = cv2.resize(overlay, (240, 320))
            cv2.putText(tile, f'node {node_id} mask {fraction:.1%}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
            review_tiles.append((fraction, tile))
            review_tiles = sorted(review_tiles, key=lambda x: -x[0])[:12]
        if len(records) % 50 == 0:
            print(f'{args.db.name}: masked {len(records)} frames', flush=True)
    connection.close()
    with (args.output / 'frames.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['id', 'masked_fraction', 'instances', 'classes'])
        writer.writeheader()
        writer.writerows(records)
    if review_tiles:
        tiles = [tile for _, tile in review_tiles]
        tiles.extend([np.zeros_like(tiles[0])] * ((-len(tiles)) % 4))
        sheet = np.concatenate([np.concatenate(tiles[i:i + 4], axis=1) for i in range(0, len(tiles), 4)])
        cv2.imwrite(str(args.output / 'mask_review.jpg'), sheet)
    report = {
        'source_database': str(args.db.resolve()), 'method': 'YOLOv8n-seg COCO instance masks',
        'model_url': MODEL_URL, 'weights_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
        'ultralytics_version': ultralytics.__version__, 'torch_version': torch.__version__,
        'opencv_version': cv2.__version__, 'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        'classes': {c: model.names[c] for c in MOBILE_CLASSES},
        'frame_count': len(records), 'frames_with_mask': sum(r['instances'] > 0 for r in records),
        'mean_masked_fraction': float(np.mean([r['masked_fraction'] for r in records])),
        'elapsed_seconds': time.monotonic() - started,
        'semantics': '255=excluded, 0=keep, RGB resolution; nearest-neighbor resize to depth',
        'limitations': ['Potentially mobile class exclusion, not measured object motion or tracking.',
                       'COCO does not cover every cart or object. False positives and misses are possible.',
                       'Original RGB and depth are not modified.'],
    }
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
