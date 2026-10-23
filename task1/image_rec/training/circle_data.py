"""Build a separate, reproducible circle/6 fine-tuning dataset; never edit sources.

Uses original photographic glyphs and the notebook's hole-preserving mask recovery.
Synthetic validation is a stress test, not an independent robot-camera benchmark.
"""
import argparse
import ast
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import random

import cv2
import numpy as np
from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parent
NAMES = [str(i) for i in range(11, 41)] + ["marker"]
TARGETS = {5, 29}  # image IDs 16 (six), 40 (circle)
SEED = 20261006


def notebook_geometry():
    """Load only pure function definitions; never execute notebook workflow cells."""
    nb = json.loads((ROOT / "model.ipynb").read_text())
    wanted = {"glyph_mask_with_holes", "card_quad"}
    definitions = []
    for cell in nb["cells"]:
        if cell["cell_type"] != "code":
            continue
        source = "".join(cell["source"])
        if not any(f"def {name}(" in source for name in wanted):
            continue
        definitions.extend(node for node in ast.parse(source).body
                           if isinstance(node, ast.FunctionDef) and node.name in wanted)
    if {node.name for node in definitions} != wanted:
        raise ValueError("Notebook glyph geometry functions missing")
    scope = {"cv2": cv2, "np": np}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), "notebook_geometry", "exec"), scope)
    return scope["glyph_mask_with_holes"], scope["card_quad"]


def label_for(image):
    return image.parent.parent / "labels" / (image.stem + ".txt")


def labels(path):
    return [(int(row[0]), np.asarray(row[1:], np.float32).reshape(-1, 2))
            for line in path.read_text().splitlines() if (row := line.split())]


def scene_key(path):
    stem = path.stem
    if stem.startswith(("s0_", "s1_", "s2_")):
        stem = stem[3:]
    return stem.split("_jpg.rf.")[0]


def hole_fraction(mask):
    """Area of enclosed holes relative to the external filled silhouette."""
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return 0.0
    outer = sum(cv2.contourArea(c) for c, h in zip(contours, hierarchy[0]) if h[3] == -1)
    holes = sum(cv2.contourArea(c) for c, h in zip(contours, hierarchy[0]) if h[3] != -1)
    return holes / max(outer, 1)


def texture(size, rng, family, photo_bank=None):
    if family == "photo":
        if not photo_bank:
            raise ValueError("Photographic texture bank is required")
        patch = photo_bank[int(rng.integers(len(photo_bank)))]
        patch = np.rot90(patch, int(rng.integers(4)))
        result = cv2.resize(patch, (size, size), interpolation=cv2.INTER_LINEAR).astype(np.float32)
        lum = float(cv2.cvtColor(result, cv2.COLOR_BGR2GRAY).mean())
        result *= rng.uniform(120, 210) / max(lum, 1)
        return np.clip(result, 0, 255)
    y, x = np.mgrid[:size, :size].astype(np.float32)
    period = rng.uniform(5, 38)
    theta = rng.uniform(0, np.pi)
    u = x * np.cos(theta) + y * np.sin(theta)
    v = -x * np.sin(theta) + y * np.cos(theta)
    if family == "weave":
        field = np.sin(u / period * 6.28) * np.sin(v / period * 6.28)
    elif family == "stripes":
        field = np.sin(u / period * 6.28)
    elif family == "checker":
        field = ((np.floor(u / period) + np.floor(v / period)) % 2) * 2 - 1
    elif family == "wood":
        field = np.sin(u / period * 6.28 + 2 * np.sin(v / (period * 2)))
    elif family == "speckle":
        field = rng.normal(0, 0.5, (size, size))
    elif family == "glitter":
        field = rng.normal(0, 0.2, (size, size))
        field += (rng.random((size, size)) > 0.93) * 2
    else:
        field = cv2.resize(rng.uniform(-1, 1, (12, 12)).astype(np.float32),
                           (size, size), interpolation=cv2.INTER_CUBIC)
    base = rng.uniform(100, 220, 3)
    amplitude = rng.uniform(25, 95)
    result = base + amplitude * field[..., None]
    result += rng.normal(0, 5, (size, size, 3))
    return np.clip(result, 20, 255).astype(np.float32)


FAMILIES = ("weave", "stripes", "checker", "wood", "speckle", "glitter", "mottle")


def other_sources(pool):
    """Interleave classes so a short sample cannot contain only the first digits."""
    groups = [paths for c, paths in sorted(pool.items()) if c not in TARGETS]
    return [paths[i] for i in range(max(map(len, groups))) for paths in groups if i < len(paths)]


def render(image, points, mask, rng, family, quad_fn, blank=False, photo_bank=None):
    """Repaint one single-symbol card, retaining glyph holes and source geometry."""
    height, width = image.shape[:2]
    quad = quad_fn(mask, points, rng)
    source_corners = np.float32([[0, 0], [255, 0], [255, 255], [0, 255]])
    matrix = cv2.getPerspectiveTransform(source_corners, quad[[1, 2, 3, 0]])
    face = cv2.warpPerspective(texture(256, rng, family, photo_bank), matrix, (width, height),
                               borderMode=cv2.BORDER_REPLICATE)
    card = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(card, quad.astype(np.int32), 255)
    # Guarantee that no old glyph pixels survive outside the new face.
    poly = np.zeros_like(card)
    cv2.fillPoly(poly, [points], 255)
    card = cv2.bitwise_or(card, cv2.dilate(poly, np.ones((7, 7), np.uint8)))
    yy, xx = np.mgrid[:height, :width]
    shade = 1 + rng.uniform(-0.22, 0.22) * (xx / width - 0.5)
    shade += rng.uniform(-0.22, 0.22) * (yy / height - 0.5)
    if not blank:
        alpha = cv2.GaussianBlur(mask.astype(np.float32) / 255, (0, 0), 0.45)[..., None]
        black = float(rng.uniform(3, 30))
        face = face * (1 - alpha) + black * alpha
    face *= shade[..., None]
    a = cv2.GaussianBlur(card.astype(np.float32) / 255, (0, 0), 0.7)[..., None]
    out = image.astype(np.float32) * (1 - a) + face * a
    if rng.random() < 0.7:
        out = cv2.GaussianBlur(out, (0, 0), rng.uniform(0.3, 1.0))
    out += rng.normal(0, rng.uniform(0.5, 3), out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def read_source(image_path, mask_fn):
    annotation = labels(label_for(image_path))
    if len(annotation) != 1 or annotation[0][0] == 30:
        return None
    image = cv2.imread(str(image_path))
    cls, polygon = annotation[0]
    points = np.rint(polygon * [image.shape[1], image.shape[0]]).astype(np.int32)
    mask = mask_fn(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), points,
                   grow=cls in (25, 26, 27, 28))
    # Reject ambiguous extraction rather than manufacture a filled-in six.
    if cls == 5 and hole_fraction(mask) < 0.025:
        return None
    return image, cls, points, mask


def choose_sources(split, mask_fn):
    groups = defaultdict(dict)
    rejected = []
    for image in sorted((ROOT / split / "images").glob("*.jpg")):
        annotation = labels(label_for(image))
        if len(annotation) != 1 or annotation[0][0] == 30:
            continue
        cls = annotation[0][0]
        key = scene_key(image)
        if key in groups[cls]:
            continue
        if cls in TARGETS and read_source(image, mask_fn) is None:
            rejected.append(str(image))
            continue
        groups[cls][key] = image
    return {c: list(group.values()) for c, group in groups.items()}, rejected


def save_preview(items, output):
    canvas = Image.new("RGB", (4 * 300, ((len(items) + 3) // 4) * 325), "white")
    draw = ImageDraw.Draw(canvas)
    for i, (title, array) in enumerate(items):
        tile = Image.fromarray(cv2.cvtColor(array, cv2.COLOR_BGR2RGB))
        tile.thumbnail((295, 290))
        x, y = (i % 4) * 300, (i // 4) * 325
        canvas.paste(tile, (x, y + 30))
        draw.text((x + 4, y + 5), title, fill="black")
    canvas.save(output)


def build(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "manifest.json").exists():
        raise ValueError("Output already managed; choose a new output directory")
    manifest = {"complete": False, "seed": SEED, "circle_class": 29, "six_class": 5,
                "limitations": ["Synthetic stress validation; no independent Pi captures supplied.",
                                "Legacy dataset has known cross-split duplicates."]}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    cv2.setNumThreads(1)
    mask_fn, quad_fn = notebook_geometry()
    sources, rejected = choose_sources("train", mask_fn)
    val_sources, val_rejected = choose_sources("valid", mask_fn)
    # Separate synthetic training/validation by source filename, before augmentation.
    val_keys = {scene_key(p) for group in val_sources.values() for p in group}
    sources = {c: [p for p in group if scene_key(p) not in val_keys] for c, group in sources.items()}
    manifest["usable_target_sources"] = {"train": {NAMES[c]: len(sources[c]) for c in TARGETS},
                                          "val": {NAMES[c]: len(val_sources[c]) for c in TARGETS}}
    manifest["rejected_target_sources"] = rejected + val_rejected
    rows, previews = [], []
    for split, pool, quantities in [
        ("train", sources, {29: args.circle_variants, 5: args.six_variants,
                             "other": args.other_variants, "blank": args.blank_variants}),
        ("val", val_sources, {29: args.val_variants, 5: args.val_variants,
                              "other": 60, "blank": 60}),
    ]:
        (out / split / "images").mkdir(parents=True, exist_ok=True)
        (out / split / "labels").mkdir(parents=True, exist_ok=True)
        for category, count in quantities.items():
            candidates = (pool[category] if isinstance(category, int) else
                          other_sources(pool)
                          if category == "other" else pool[29] + pool[5])
            if not candidates:
                raise ValueError(f"No usable sources: {split} {category}")
            for i in range(count):
                rng = np.random.default_rng(SEED + (100000 if split == "val" else 0)
                                             + (int(category) if isinstance(category, int) else
                                                40 if category == "other" else 50) * 1000 + i)
                image_path = candidates[i % len(candidates)]
                source = read_source(image_path, mask_fn)
                if source is None:
                    continue
                image, cls, points, mask = source
                family = FAMILIES[i % len(FAMILIES)]
                rendered = render(image, points, mask, rng, family, quad_fn, category == "blank")
                stem = f"{category}_{i:05d}_{image_path.stem}"
                label = "" if category == "blank" else label_for(image_path).read_text()
                for size in (640, 160):
                    small = cv2.resize(rendered, (size, size), interpolation=cv2.INTER_AREA)
                    target = out / split / "images" / f"{stem}_r{size}.jpg"
                    quality = int(rng.integers(75, 96))
                    if not cv2.imwrite(str(target), small, [cv2.IMWRITE_JPEG_QUALITY, quality]):
                        raise OSError(target)
                    label_for(target).write_text(label)
                    rows.append({"path": str(target), "source": str(image_path), "split": split,
                                 "scene": scene_key(image_path), "category": str(category),
                                 "image_id": None if category == "blank" else NAMES[cls],
                                 "texture": family, "size": size, "jpeg_quality": quality})
                if split == "train" and i < 4:
                    x, y, w, h = cv2.boundingRect(points)
                    pad = max(w, h)
                    crop = rendered[max(0,y-pad//2):y+h+pad//2,max(0,x-pad//2):x+w+pad//2]
                    previews.append((f"{NAMES[cls] if category != 'blank' else 'empty'} / {family}",crop))
                if (i + 1) % 100 == 0:
                    print(f"generated {split} {category}: {i+1}/{count}", flush=True)
    # Replay all classes, balanced across classes and original/synthetic/resolution variants.
    by_class = defaultdict(list)
    excluded = Counter()
    for source in ("train", "train_synth", "train_synth2", "lowres_data_160/train"):
        for p in sorted((ROOT / source / "images").glob("*.jpg")):
            annotation = labels(label_for(p))
            classes = {c for c, _ in annotation}
            # The old synth2 compositor closes holes. Exclude ALL its six variants.
            old_synth2 = source == "train_synth2" or (source == "lowres_data_160/train" and p.name.startswith("s2_"))
            if old_synth2 and 5 in classes:
                excluded["old_synth2_six"] += 1
                continue
            if scene_key(p) in val_keys:
                excluded["validation_source_name"] += 1
                continue
            by_class[min(classes) if classes else -1].append(p)
    chooser = random.Random(SEED)
    replay = []
    for cls, candidates in sorted(by_class.items()):
        replay.extend(chooser.sample(candidates, min(args.replay_per_class, len(candidates))))
    train_paths = [str(p) for p in replay] + [row["path"] for row in rows if row["split"] == "train"]
    synthetic_val = [row["path"] for row in rows if row["split"] == "val"]
    original_val = sorted((ROOT / "lowres_data_160/val/images").glob("s0_*_r160.jpg"))
    for name, paths in [("train", train_paths), ("stress_val", synthetic_val),
                        ("selection_val", [str(p) for p in original_val] + synthetic_val)]:
        (out / f"{name}.txt").write_text("\n".join(paths) + "\n")
    for name, val in [("data", "selection_val"), ("stress", "stress_val")]:
        config = {"path": str(out), "train": str(out / "train.txt"),
                  "val": str(out / f"{val}.txt"), "test": str(ROOT / "test/images"),
                  "nc": 31, "names": NAMES}
        (out / f"{name}.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    (out / "samples.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    save_preview(previews, out / "preview.jpg")
    manifest.update(complete=True, generated=dict(Counter(row["split"] for row in rows)),
                    generated_by_category=dict(Counter(f"{r['split']}/{r['category']}" for r in rows)),
                    replay_images=len(replay), training_images=len(train_paths),
                    selection_validation_images=len(original_val)+len(synthetic_val),
                    excluded_replay=dict(excluded), generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2), flush=True)


def refresh_other(out):
    """Rebuild only this generator's matching-background other-symbol samples."""
    out = out.resolve()
    manifest = json.loads((out / "manifest.json").read_text())
    manifest["complete"] = False
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    records = [json.loads(s) for s in (out / "samples.jsonl").read_text().splitlines()]
    mask_fn, quad_fn = notebook_geometry()
    cv2.setNumThreads(1)
    pool, _ = choose_sources("valid", mask_fn)
    val_keys = {scene_key(p) for group in pool.values() for p in group}
    replacement = {}
    previews = []
    for split, source_split in (("train", "train"), ("val", "valid")):
        pool, _ = choose_sources(source_split, mask_fn)
        if split == "train":
            pool = {c: [p for p in group if scene_key(p) not in val_keys] for c, group in pool.items()}
        candidates = other_sources(pool)
        old = [r for r in records if r["split"] == split and r["category"] == "other"]
        for i in range(len(old) // 2):
            rng = np.random.default_rng(SEED + (100000 if split == "val" else 0) + 40000 + i)
            source_path = candidates[i % len(candidates)]
            image, cls, points, mask = read_source(source_path, mask_fn)
            family = FAMILIES[i % len(FAMILIES)]
            rendered = render(image, points, mask, rng, family, quad_fn)
            for j, size in enumerate((640, 160)):
                previous = old[2*i+j]
                target = out / split / "images" / f"other_{i:05d}_{source_path.stem}_r{size}.jpg"
                quality = int(rng.integers(75, 96))
                small = cv2.resize(rendered, (size,size), interpolation=cv2.INTER_AREA)
                if not cv2.imwrite(str(target), small, [cv2.IMWRITE_JPEG_QUALITY,quality]):
                    raise OSError(target)
                label_for(target).write_text(label_for(source_path).read_text())
                replacement[previous["path"]] = str(target)
                if Path(previous["path"]) != target:
                    Path(previous["path"]).unlink()
                    label_for(Path(previous["path"])).unlink()
                previous.update(path=str(target),source=str(source_path),scene=scene_key(source_path),
                                image_id=NAMES[cls],texture=family,jpeg_quality=quality)
            if split == "train" and i < 12:
                x,y,w,h=cv2.boundingRect(points);pad=max(w,h)//2
                previews.append((NAMES[cls],rendered[max(0,y-pad):y+h+pad,max(0,x-pad):x+w+pad]))
    for name in ("train", "stress_val", "selection_val"):
        path=out/f"{name}.txt"
        path.write_text("\n".join(replacement.get(s,s) for s in path.read_text().splitlines())+"\n")
    (out/"samples.jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    save_preview(previews,out/"other_symbols_preview.jpg")
    manifest.update(complete=True,other_symbols_interleaved=True,
                    generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("Matching background samples refreshed across all other symbol classes",flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "circle_data_v1")
    parser.add_argument("--circle-variants", type=int, default=400)
    parser.add_argument("--six-variants", type=int, default=200)
    parser.add_argument("--other-variants", type=int, default=150)
    parser.add_argument("--blank-variants", type=int, default=200)
    parser.add_argument("--val-variants", type=int, default=100)
    parser.add_argument("--replay-per-class", type=int, default=200)
    parser.add_argument("--refresh-other", action="store_true")
    args=parser.parse_args()
    if args.refresh_other:
        refresh_other(args.output)
    else:
        build(args)
