"""MDP symbol recognition — inference module for the RPi<->PC server.

Fixes two failure modes observed on real photos:
1. EXIF rotation: phone/libcamera JPEGs often store the pixels sideways with an
   EXIF orientation tag. cv2.imread IGNORES that tag, so the model silently
   receives a rotated image and misreads every symbol. We load via PIL with
   ImageOps.exif_transpose to always feed the model an upright image.
2. Duplicate/conflicting boxes on one card: class-agnostic IoU dedup keeps the
   highest-confidence box per physical card before applying task heuristics.

Task heuristics (proven by past MDP cohorts):
- among candidates pick the LARGEST box (target obstacle = closest)
- optional C/L/R signal from the path-planning algo restricts candidates
- 'marker' (the concentric-squares card) is a valid answer: it means the
  required image is on ANOTHER FACE of that obstacle, so the robot should go
  around. Callers check `image_id == 'marker'` to trigger that behaviour;
  pass include_marker=False only if you explicitly want it ignored.
"""
from pathlib import Path

import numpy as np
import cv2
from PIL import Image, ImageOps
from ultralytics import YOLO

CLASS_NAMES = ['11','12','13','14','15','16','17','18','19','20','21','22','23','24','25',
               '26','27','28','29','30','31','32','33','34','35','36','37','38','39','40','marker']
SYMBOL = {**{str(10+i): str(i) for i in range(1, 10)},
          **dict(zip(map(str, range(20, 36)), "ABCDEFGHSTUVWXYZ")),
          '36': 'Up arrow', '37': 'Down arrow', '38': 'Right arrow',
          '39': 'Left arrow', '40': 'Stop',
          'marker': 'Square marker (image on other side)'}

_HERE = Path(__file__).parent
WEIGHT_PREFERENCE = [                       # newest first
    _HERE / 'runs' / 'yolo26s_seg_mdp_v3' / 'weights' / 'best.pt',
    _HERE / 'runs' / 'yolo26s_seg_mdp_v2' / 'weights' / 'best.pt',
    _HERE / 'runs' / 'yolo26s_seg_mdp-3' / 'weights' / 'best.pt',
]

_model = None


def get_model(weights=None):
    global _model
    if _model is None:
        w = Path(weights) if weights else next(p for p in WEIGHT_PREFERENCE if p.exists())
        _model = YOLO(str(w))
    return _model


def load_upright(image):
    """Return an upright BGR ndarray. Applies EXIF orientation for file paths/bytes."""
    if isinstance(image, np.ndarray):
        return image
    pil = Image.open(image)
    pil = ImageOps.exif_transpose(pil)          # <-- the crucial fix
    return cv2.cvtColor(np.array(pil.convert('RGB')), cv2.COLOR_RGB2BGR)


def _iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


# 1280 (not the training 640) so small / low-contrast cards keep enough pixels:
# a distant glitter-texture card that is invisible at 640 is read correctly at 1280.
IMGSZ = 1280

_ROTATIONS = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]


def _best_rotation(img, model, conf):
    """Some sources strip EXIF and store pixels sideways. Try 4 rotations and
    keep the one the model is most confident about (upright symbols score
    higher and more consistently than sideways ones)."""
    best = (-1.0, img, None)
    for rot in _ROTATIONS:
        cand = img if rot is None else cv2.rotate(img, rot)
        r = model.predict(cand, conf=conf, imgsz=IMGSZ, verbose=False)[0]
        score = sum(float(b.conf) for b in r.boxes)
        if score > best[0]:
            best = (score, cand, r)
    return best[1], best[2]


def detect(image, conf=0.35, weights=None, auto_rotate=False):
    """All deduped detections: list of dicts sorted by box area (desc).

    auto_rotate: enable when the image source may store pixels sideways
    (EXIF stripped). Not needed for a fixed upright RPi camera."""
    img = load_upright(image)
    model = get_model(weights)
    if auto_rotate:
        img, r = _best_rotation(img, model, conf)
    else:
        r = model.predict(img, conf=conf, imgsz=IMGSZ, verbose=False)[0]
    dets = []
    for b in r.boxes:
        xyxy = tuple(map(float, b.xyxy[0]))
        dets.append({'class': CLASS_NAMES[int(b.cls)],
                     'symbol': SYMBOL[CLASS_NAMES[int(b.cls)]],
                     'conf': float(b.conf),
                     'box': xyxy,
                     'area': (xyxy[2]-xyxy[0]) * (xyxy[3]-xyxy[1])})
    # class-agnostic dedup: two boxes on the same card -> keep higher conf
    dets.sort(key=lambda d: -d['conf'])
    kept = []
    for d in dets:
        if all(_iou(d['box'], k['box']) < 0.5 for k in kept):
            kept.append(d)
    kept.sort(key=lambda d: -d['area'])
    return kept, r, img


def infer(image, conf=0.35, signal=None, weights=None, auto_rotate=False,
          include_marker=True):
    """Main server entry point.

    image: path / bytes / BGR ndarray. signal: None|'C'|'L'|'R' from the algo team.
    Returns (image_id or None, confidence, annotated BGR ndarray).
    image_id == 'marker' means the square marker faces the robot: the required
    image is on another face of that obstacle.
    """
    kept, r, img = detect(image, conf=conf, weights=weights, auto_rotate=auto_rotate)
    w = img.shape[1]
    cands = []
    for d in kept:
        if d['class'] == 'marker' and not include_marker:
            continue
        cx = (d['box'][0] + d['box'][2]) / 2 / w
        if signal == 'L' and cx > 0.45: continue
        if signal == 'R' and cx < 0.55: continue
        if signal == 'C' and not (0.25 < cx < 0.75): continue
        cands.append(d)
    if not cands:
        return None, 0.0, r.plot()
    top = cands[0]                       # largest area first
    return top['class'], top['conf'], r.plot()


if __name__ == '__main__':
    import sys
    for p in sys.argv[1:]:
        image_id, c, annotated = infer(p, auto_rotate=True)
        name = SYMBOL.get(image_id, '?') if image_id else 'none'
        print(f'{p}: id={image_id} ({name}) conf={c:.2f}')
        out = Path(p).with_suffix('.pred.jpg')
        cv2.imwrite(str(out), annotated)
        print(f'  annotated -> {out}')
