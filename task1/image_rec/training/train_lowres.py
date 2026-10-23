"""Train a new YOLO26s-seg run on original, textured, and lower-resolution data.

Default: initialize from the original COCO checkpoint, not an earlier MDP run.
Use --init random for literal training from scratch (random model weights).
The default training input remains 640: this experiment simulates lower camera
resolution, independently of the network's input size.
"""
import argparse
import json
import os
from pathlib import Path

import yaml

from gen_lowres import ROOT, image_label_pairs, positive_int, resolve_sources

os.environ.setdefault("PIN_MEMORY", "False")


def check_dataset(data_path):
    manifest_path = data_path.parent / "manifest.json"
    if not data_path.is_file() or not manifest_path.is_file():
        raise ValueError("Combined dataset is missing. Run python gen_lowres.py first.")
    manifest = json.loads(manifest_path.read_text())
    if not manifest.get("complete"):
        raise ValueError("Low-resolution generation is incomplete. Rerun gen_lowres.py.")
    if manifest["config"].get("limit_per_source") is not None:
        raise ValueError("This is a --limit preview. Generate the full dataset into a new --output directory.")
    _, sources = resolve_sources(data_path)
    counts = {}
    for split, directories in sources.items():
        counts[split] = sum(len(image_label_pairs(p)) for p in directories)
        if split in ("train", "val"):
            generated = data_path.parent / split / "images"
            if generated not in directories:
                raise ValueError(f"Combined YAML does not include lower-resolution {split} images.")
            if len(image_label_pairs(generated)) != manifest["generated"][split]:
                raise ValueError(f"Generated {split} image count does not match manifest. Rerun generation.")
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT / "lowres_data" / "data.yaml")
    parser.add_argument("--init", choices=("pretrained", "random"), default="pretrained")
    parser.add_argument("--epochs", type=positive_int, default=60)
    parser.add_argument("--patience", type=int, default=12, help="early stopping patience; 0 disables it")
    parser.add_argument("--imgsz", type=positive_int, default=640)
    parser.add_argument("--batch", type=positive_int, default=32)
    parser.add_argument("--device", default="0", help="training device, e.g. 0 or cpu")
    parser.add_argument("--workers", type=int, default=2, help="keep at 2 for this container's 1 GB shared memory")
    parser.add_argument("--name", default="yolo26s_seg_mdp_lowres")
    parser.add_argument("--dry-run", action="store_true", help="validate data and print settings without training")
    args = parser.parse_args()
    if args.workers < 0 or args.patience < 0:
        parser.error("--workers and --patience must be nonnegative")
    data_path = args.data.resolve()
    try:
        counts = check_dataset(data_path)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")

    if args.init == "pretrained":
        model_source = ROOT / "yolo26s-seg.pt"
        if not model_source.is_file():
            parser.exit(1, f"Base COCO checkpoint missing: {model_source}\n")
    else:
        model_source = "yolo26s-seg.yaml"  # architecture bundled with Ultralytics

    settings = dict(
        data=str(data_path),
        epochs=args.epochs,
        patience=args.patience,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(ROOT / "runs"),
        name=args.name,
        exist_ok=False,
        pretrained=args.init == "pretrained",
        resume=False,
        seed=0,
        optimizer="auto",
        fliplr=0.0,
        flipud=0.0,
        # Copy-paste's default flip mode mirrors arrows even with fliplr=0.
        copy_paste=0.0,
        degrees=5.0,
        translate=0.1,
        scale=0.3,
        hsv_h=0.03,
        hsv_s=0.6,
        hsv_v=0.5,
        mosaic=1.0,
        close_mosaic=min(10, args.epochs),
    )
    print(f"Dataset image counts: {counts}", flush=True)
    print(f"Initialization: {args.init} ({model_source})", flush=True)
    print(yaml.safe_dump(settings, sort_keys=False), flush=True)
    if args.dry_run:
        print("Dry run: dataset checks passed; no model loaded or training started.")
        return

    try:
        from ultralytics import YOLO
    except ImportError:
        parser.exit(1, 'Ultralytics is missing. Use the training environment or install "ultralytics==8.4.128".\n')
    model = YOLO(str(model_source), task="segment")
    model.train(**settings)
    save_dir = Path(model.trainer.save_dir)
    best_path = save_dir / "weights" / "best.pt"
    # Clean original test split; lower-resolution validation already participates
    # in checkpoint selection. A separate robot-camera holdout is still needed.
    metrics = YOLO(str(best_path)).val(
        data=str(data_path), split="test", imgsz=args.imgsz,
        batch=args.batch, device=args.device, workers=args.workers,
        project=str(save_dir), name="test", exist_ok=False,
    )
    summary = {
        "best": str(best_path),
        "initialization": args.init,
        "images": counts,
        "box_map50": float(metrics.box.map50),
        "box_map50_95": float(metrics.box.map),
        "mask_map50": float(metrics.seg.map50),
        "mask_map50_95": float(metrics.seg.map),
    }
    (save_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
