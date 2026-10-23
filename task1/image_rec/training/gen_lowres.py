"""Add lower-resolution copies to the existing MDP segmentation dataset.

The full image is resized without cropping, padding or rotation, so normalized
polygon labels remain valid. Originals are retained in the combined dataset.
Run ``python gen_lowres.py --help`` for options.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parent
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def atomic_text(path, text):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text)
    temporary.replace(path)


def resolve_sources(base_yaml):
    data = yaml.safe_load(base_yaml.read_text())
    root = Path(data.get("path", base_yaml.parent))
    if not root.is_absolute():
        root = base_yaml.parent / root
    sources = {}
    for split in ("train", "val", "test"):
        entries = data[split]
        if isinstance(entries, str):
            entries = [entries]
        sources[split] = []
        for entry in entries:
            path = Path(entry)
            path = (path if path.is_absolute() else root / path).resolve()
            if not path.is_dir() or path.name != "images":
                raise ValueError(f"Expected an existing images/ directory: {path}")
            sources[split].append(path)
    return data, sources


def image_label_pairs(image_dir):
    images = sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    labels = image_dir.parent / "labels"
    stems = [p.stem for p in images]
    if not images or len(stems) != len(set(stems)):
        raise ValueError(f"Empty image directory or duplicate image stems: {image_dir}")
    label_stems = {p.stem for p in labels.glob("*.txt")}
    if set(stems) != label_stems:
        raise ValueError(f"Image/label mismatch in {image_dir.parent}")
    return [(p, labels / f"{p.stem}.txt") for p in images]


def resize_image(source, size):
    # Label geometry follows stored pixels. Reject EXIF rotation rather than
    # silently rotating the image without also transforming its polygons.
    with Image.open(source) as pil:
        if pil.getexif().get(274, 1) not in (None, 1):
            raise ValueError(f"Normalize EXIF orientation AND labels first: {source}")
    image = cv2.imread(str(source), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None:
        raise ValueError(f"Cannot decode image: {source}")
    height, width = image.shape[:2]
    if size >= max(height, width):
        raise ValueError(f"--sizes must be smaller than the source ({width}x{height}): {source}")
    scale = size / max(height, width)
    target = (max(1, round(width * scale)), max(1, round(height * scale)))
    return cv2.resize(image, target, interpolation=cv2.INTER_AREA)


def write_pair(source_image, source_label, target_image, target_label, size, quality):
    small = resize_image(source_image, size)
    ok, encoded = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError(f"Cannot encode image: {source_image}")
    temporary = target_image.with_suffix(".jpg.tmp")
    temporary.write_bytes(encoded.tobytes())
    temporary.replace(target_image)
    temporary_label = target_label.with_suffix(".txt.tmp")
    temporary_label.write_bytes(source_label.read_bytes())
    temporary_label.replace(target_label)


def draw_polygons(image, label_path, names):
    annotated = image.copy()
    height, width = annotated.shape[:2]
    for line in label_path.read_text().splitlines():
        values = line.split()
        if not values:
            continue
        if len(values) < 7 or len(values) % 2 != 1:
            raise ValueError(f"Expected polygon annotations: {label_path}")
        points = np.array(values[1:], dtype=np.float32).reshape(-1, 2)
        points = np.rint(points * [width, height]).astype(np.int32)
        cv2.polylines(annotated, [points], True, (0, 255, 0), 1, cv2.LINE_AA)
        x, y = points.min(axis=0)
        cv2.putText(annotated, f"ID {names[int(values[0])]}", (max(0, x), max(16, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
    return annotated


def generate_preview(args, data, sources, output):
    # One labeled source from the original training directory, one resolution.
    pair = next((pair for pair in image_label_pairs(sources["train"][0])
                 if pair[1].read_text().strip()), None)
    if pair is None:
        raise ValueError("No annotated image in the first training source.")
    source_image, source_label = pair
    if output.exists() and any(output.iterdir()) and not (output / "preview.json").exists():
        raise ValueError(f"Choose an empty preview output directory: {output}")
    (output / "images").mkdir(parents=True, exist_ok=True)
    (output / "labels").mkdir(parents=True, exist_ok=True)
    target_image, target_label = output / "images/sample.jpg", output / "labels/sample.txt"
    write_pair(source_image, source_label, target_image, target_label, args.sizes[0], args.jpeg_quality)
    # Read the saved JPEG so the visual includes the actual compression artifacts.
    small = cv2.imread(str(target_image))
    original = cv2.imread(str(source_image), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    height, width = original.shape[:2]
    display_size = (round(width * min(1, 640 / max(height, width))),
                    round(height * min(1, 640 / max(height, width))))
    left = cv2.resize(original, display_size, interpolation=cv2.INTER_AREA)
    right = cv2.resize(small, display_size, interpolation=cv2.INTER_NEAREST)
    left = draw_polygons(left, source_label, data["names"])
    right = draw_polygons(right, target_label, data["names"])
    comparison = np.hstack([left, right])
    comparison = cv2.copyMakeBorder(comparison, 46, 0, 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    titles = [f"Original {width}x{height}", f"Lower resolution {small.shape[1]}x{small.shape[0]} (enlarged)"]
    for index, title in enumerate(titles):
        cv2.putText(comparison, title, (index * display_size[0] + 12, 29),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 1, cv2.LINE_AA)
    for path, image in [(output / "annotated.png", draw_polygons(small, target_label, data["names"])),
                        (output / "comparison.png", comparison)]:
        if not cv2.imwrite(str(path), image):
            raise ValueError(f"Cannot write preview: {path}")
    atomic_text(output / "preview.json", json.dumps({
        "source_image": str(source_image), "source_label": str(source_label),
        "size": args.sizes[0], "jpeg_quality": args.jpeg_quality,
    }, indent=2) + "\n")
    print(f"Generated exactly one image/label pair: {target_image}")
    print(f"Annotation overlay: {output / 'annotated.png'}")
    print(f"Comparison: {output / 'comparison.png'}")


def generate(args):
    base_yaml = args.base_data.resolve()
    output = args.output.resolve()
    data, sources = resolve_sources(base_yaml)
    # Never generate into, over, or around the original source directories.
    for directories in sources.values():
        for directory in directories:
            source_root = directory.parent
            if output == source_root or source_root in output.parents or output in source_root.parents:
                raise ValueError(f"Output must be separate from source datasets: {output}")

    if args.preview:
        generate_preview(args, data, sources, output)
        return

    pairs = {split: [image_label_pairs(p) for p in sources[split]] for split in ("train", "val")}
    if args.limit is not None:
        pairs = {split: [group[:args.limit] for group in groups] for split, groups in pairs.items()}
    sizes = sorted(set(args.sizes), reverse=True)
    counts = {split: sum(map(len, groups)) * len(sizes) for split, groups in pairs.items()}
    config = {
        "schema": 1,
        "base_data": str(base_yaml),
        "sources": {split: list(map(str, paths)) for split, paths in sources.items()},
        "sizes": sizes,
        "jpeg_quality": args.jpeg_quality,
        "limit_per_source": args.limit,
    }
    print(f"New copies: train={counts['train']:,}, val={counts['val']:,}; sizes={sizes}", flush=True)
    print(f"Output: {output}", flush=True)
    if args.dry_run:
        print("Dry run: no files written.")
        return

    manifest_path = output / "manifest.json"
    if output.exists() and any(output.iterdir()):
        if not manifest_path.exists():
            raise ValueError(f"Output is not a managed low-resolution dataset: {output}")
        previous = json.loads(manifest_path.read_text())
        if previous.get("config") != config:
            raise ValueError("Generation settings changed. Choose a different --output directory.")
    output.mkdir(parents=True, exist_ok=True)
    manifest = {"config": config, "complete": False, "generated": counts}
    atomic_text(manifest_path, json.dumps(manifest, indent=2) + "\n")

    for split, groups in pairs.items():
        image_dir, label_dir = output / split / "images", output / split / "labels"
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        expected = set()
        done = 0
        for source_index, group in enumerate(groups):
            for source_image, source_label in group:
                for size in sizes:
                    stem = f"s{source_index}_{source_image.stem}_r{size}"
                    expected.add(stem)
                    target_image = image_dir / f"{stem}.jpg"
                    target_label = label_dir / f"{stem}.txt"
                    # Always regenerate: changed source images/labels cannot leave stale copies.
                    write_pair(source_image, source_label, target_image, target_label,
                               size, args.jpeg_quality)
                    done += 1
                    if done % 1000 == 0:
                        print(f"{split}: {done:,}/{counts[split]:,}", flush=True)
        actual_images = {p.stem for p in image_dir.glob("*.jpg")}
        actual_labels = {p.stem for p in label_dir.glob("*.txt")}
        if actual_images != expected or actual_labels != expected:
            raise ValueError("Stale files found in output. Generate into a new --output directory.")
        # Ultralytics caches labels using file sizes; same-length edits may otherwise be missed.
        (output / split / "labels.cache").unlink(missing_ok=True)
        print(f"{split}: wrote {done:,} image/label pairs", flush=True)

    combined = {
        "path": str(base_yaml.parent),
        "train": [*map(str, sources["train"]), str(output / "train" / "images")],
        "val": [*map(str, sources["val"]), str(output / "val" / "images")],
        "test": list(map(str, sources["test"])),
        "nc": data["nc"],
        "names": data["names"],
    }
    atomic_text(output / "data.yaml", yaml.safe_dump(combined, sort_keys=False))
    manifest["complete"] = True
    atomic_text(manifest_path, json.dumps(manifest, indent=2) + "\n")
    print(f"Combined dataset: {output / 'data.yaml'}")
    if args.limit is not None:
        print("Preview dataset only (--limit): the training script will reject it.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-data", type=Path, default=ROOT / "data_local_v3.yaml")
    parser.add_argument("--output", type=Path, help="default: lowres_data/ (lowres_preview/ in preview mode)")
    parser.add_argument("--sizes", type=positive_int, nargs="+", default=[480, 320],
                        help="longest side of each saved copy; aspect ratio is preserved")
    parser.add_argument("--jpeg-quality", type=int, default=95,
                        help="JPEG quality 1-100; lower only to simulate camera compression")
    parser.add_argument("--limit", type=positive_int, help="preview only: images per source directory")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="show counts without generating files")
    mode.add_argument("--preview", action="store_true", help="write one image/label pair plus visual overlays; pass one --sizes value")
    args = parser.parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    if args.preview and (len(args.sizes) != 1 or args.limit is not None):
        parser.error("--preview requires exactly one --sizes value and cannot be combined with --limit")
    args.output = args.output or ROOT / ("lowres_preview" if args.preview else "lowres_data")
    try:
        generate(args)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()
