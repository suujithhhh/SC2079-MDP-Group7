"""Fine-tune the trained model on original + synth v1 + synth v2 data,
then evaluate on the test split and the real photo."""
import os
os.environ.setdefault('PIN_MEMORY', 'False')  # /dev/shm is 1GB; default loader crashes

from pathlib import Path
import cv2
from ultralytics import YOLO

ROOT = Path(__file__).parent
CLASS_NAMES = ['11','12','13','14','15','16','17','18','19','20','21','22','23','24','25',
               '26','27','28','29','30','31','32','33','34','35','36','37','38','39','40','marker']

data_yaml = ROOT / 'data_local_v2.yaml'
data_yaml.write_text(
    f"path: {ROOT}\n"
    "train:\n  - train/images\n  - train_synth/images\n  - train_synth2/images\n"
    "val: valid/images\n"
    "test: test/images\n"
    f"nc: {len(CLASS_NAMES)}\n"
    f"names: {CLASS_NAMES}\n"
)

model = YOLO(str(ROOT / 'runs' / 'yolo26s_seg_mdp-3' / 'weights' / 'best.pt'))
results = model.train(
    data=str(data_yaml),
    epochs=30,
    patience=10,
    imgsz=640,
    batch=32,
    device=0,
    workers=2,            # /dev/shm is only 1GB — 8 workers overflow it
    lr0=0.002,            # fine-tune: lower LR than from-scratch default 0.01
    project=str(ROOT / 'runs'),
    name='yolo26s_seg_mdp_v2',
    fliplr=0.0, flipud=0.0,
    degrees=10.0, translate=0.1, scale=0.3,
    hsv_h=0.05, hsv_s=0.7, hsv_v=0.6,
    copy_paste=0.3,
    erasing=0.4,
    mosaic=1.0, close_mosaic=8,
)

best_path = Path(results.save_dir) / 'weights' / 'best.pt'
best = YOLO(str(best_path))

lines = [f'BEST: {best_path}']
tm = best.val(data=str(data_yaml), split='test')
lines.append(f'TEST  box mAP50={tm.box.map50:.4f} mAP50-95={tm.box.map:.4f}  '
             f'mask mAP50={tm.seg.map50:.4f} mAP50-95={tm.seg.map:.4f}')

# real photo check (correct upright orientation = rotate CCW)
img = cv2.imread(str(ROOT / 'real ' / 'e8ca3def-5e13-4a11-9720-3b395c26cb2a.jpeg'))
up = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
r = best.predict(up, conf=0.25, imgsz=640, verbose=False)[0]
dets = sorted([(CLASS_NAMES[int(b.cls)], round(float(b.conf), 2)) for b in r.boxes],
              key=lambda t: -t[1])
lines.append(f'REAL upright: {dets}')
cv2.imwrite(str(ROOT / 'runs' / 'yolo26s_seg_mdp_v2_real_pred.jpg'), r.plot())

(ROOT / 'runs' / 'v2_summary.txt').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
