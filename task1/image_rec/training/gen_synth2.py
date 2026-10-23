"""Synth v2: restyle train images to match the real-test card style.

Real-test cards (see `real /`) are light, heavily textured (glitter, knit,
wood-chip, denim) with DARK glyphs — the inverse of the training data.
Using the glyph-outline polygons we restyle both the glyph and the card:
glyph recolored (mostly dark), card area replaced with a realistic light
texture. Labels are unchanged. Confusable classes (V, W, U, X, Y) are
generated twice as often.
"""
from pathlib import Path
import random
import numpy as np
import cv2

ROOT = Path(__file__).parent
CLASS_NAMES = ['11','12','13','14','15','16','17','18','19','20','21','22','23','24','25',
               '26','27','28','29','30','31','32','33','34','35','36','37','38','39','40','marker']
# V, W, U, X, Y and 1/7 are the shape-confusable ones
BOOST = {'31', '32', '30', '33', '34', '11', '17'}

rng = np.random.default_rng(20260827)
py_rng = random.Random(20260827)

TRAIN_IMGS = sorted((ROOT / 'train' / 'images').glob('*.jpg'))


def read_polys(lbl_path, w, h):
    out = []
    for line in Path(lbl_path).read_text().splitlines():
        vals = line.split()
        if len(vals) > 5:
            pts = (np.array(vals[1:], dtype=np.float32).reshape(-1, 2) * [w, h]).astype(np.int32)
            out.append((int(vals[0]), pts))
    return out


def _base_color(light):
    if light:  # light card background
        return rng.integers(150, 256, 3, dtype=np.uint8)
    return rng.integers(0, 90, 3, dtype=np.uint8)


def texture(shape, light):
    """Realistic-ish card texture. light=True -> bright background."""
    h, w = shape[:2]
    kind = py_rng.choices(
        ['crop', 'glitter', 'knit', 'blotch', 'speckle', 'plain'],
        weights=[30, 20, 15, 15, 10, 10])[0]
    if kind == 'crop':
        # natural texture: random crop of another arena photo (floors/wood/tiles)
        src = cv2.imread(str(py_rng.choice(TRAIN_IMGS)))
        sh, sw = src.shape[:2]
        ch, cw = max(2, sh // 3), max(2, sw // 3)
        y0 = int(rng.integers(0, sh - ch)); x0 = int(rng.integers(0, sw - cw))
        t = cv2.resize(src[y0:y0 + ch, x0:x0 + cw], (w, h))
        if light:  # brighten natural crops so dark glyphs stay visible
            t = cv2.convertScaleAbs(t, alpha=0.6, beta=120)
        return t
    base = np.full((h, w, 3), _base_color(light), dtype=np.uint8)
    if kind == 'glitter':
        n = int(h * w * 0.15)
        ys, xs = rng.integers(0, h, n), rng.integers(0, w, n)
        vals = rng.integers(0, 256, (n, 3), dtype=np.uint8)
        base[ys, xs] = vals
        base = cv2.GaussianBlur(base, (3, 3), 0)
    elif kind == 'knit':
        yy, xx = np.mgrid[0:h, 0:w]
        fx, fy = rng.uniform(0.3, 1.2), rng.uniform(0.3, 1.2)
        wave = (np.sin(xx * fx) * np.sin(yy * fy) * rng.uniform(15, 45))
        base = np.clip(base.astype(np.int16) + wave[..., None], 0, 255).astype(np.uint8)
        base = base + rng.integers(-10, 10, base.shape).astype(np.int16).clip(-10, 10).astype(np.uint8)
    elif kind == 'blotch':
        small = rng.integers(0, 256, (max(2, h // 12), max(2, w // 12), 3), dtype=np.uint8)
        blot = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)
        base = cv2.addWeighted(base, 0.55, blot, 0.45, 0)
    elif kind == 'speckle':
        noise = rng.normal(0, 25, (h, w, 3))
        base = np.clip(base.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return base


def restyle(img, polys):
    out = img.copy()
    h, w = img.shape[:2]
    glyph_mask_all = np.zeros((h, w), np.uint8)
    for _, pts in polys:
        cv2.fillPoly(glyph_mask_all, [pts], 255)

    for cls_i, pts in polys:
        if CLASS_NAMES[cls_i] == 'marker':
            continue  # leave the bullseye untouched
        # card region = expanded glyph bbox
        x, y, bw, bh = cv2.boundingRect(pts)
        pad = rng.uniform(0.3, 0.55)
        px, py = int(bw * pad), int(bh * pad)
        x0, y0 = max(0, x - px), max(0, y - py)
        x1, y1 = min(w, x + bw + px), min(h, y + bh + py)
        region = out[y0:y1, x0:x1]
        keep = glyph_mask_all[y0:y1, x0:x1] > 0

        # polarity: 70% real-test look (dark glyph on light card)
        mode = py_rng.choices(['dark_on_light', 'light_on_dark', 'original_glyph'],
                              weights=[70, 15, 15])[0]
        light_bg = mode != 'light_on_dark'
        bg = texture(region.shape, light=light_bg)
        region[~keep] = bg[~keep]

        if mode != 'original_glyph':
            color = _base_color(light=not light_bg).astype(np.float32)
            # keep original luminance variation as shading so edges stay natural
            gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
            shade = 0.75 + 0.5 * gray[..., None]
            glyph_px = np.clip(color[None, None] * shade, 0, 255).astype(np.uint8)
            region[keep] = glyph_px[keep]

    # 15%: grayscale everything — forces shape reliance
    if py_rng.random() < 0.15:
        out = cv2.cvtColor(cv2.cvtColor(out, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
    return out


def main():
    img_dir = ROOT / 'train_synth2' / 'images'
    lbl_dir = ROOT / 'train_synth2' / 'labels'
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    train_lbls = sorted((ROOT / 'train' / 'labels').glob('*.txt'))
    made = 0
    for lbl in train_lbls:
        classes = {CLASS_NAMES[int(l.split()[0])] for l in lbl.read_text().splitlines() if l.strip()}
        copies = 2 if classes & BOOST else (1 if py_rng.random() < 0.45 else 0)
        if not copies:
            continue
        img = cv2.imread(str(ROOT / 'train' / 'images' / (lbl.stem + '.jpg')))
        if img is None:
            continue
        polys = read_polys(lbl, img.shape[1], img.shape[0])
        for k in range(copies):
            stem = f'{lbl.stem}_v2{k}'
            if (lbl_dir / f'{stem}.txt').exists():
                made += 1
                continue
            cv2.imwrite(str(img_dir / f'{stem}.jpg'), restyle(img, polys))
            (lbl_dir / f'{stem}.txt').write_text(lbl.read_text())
            made += 1
            if made % 500 == 0:
                print(f'{made} generated...', flush=True)
    print(f'done: {made} synthetic v2 images in {img_dir}')


if __name__ == '__main__':
    main()
