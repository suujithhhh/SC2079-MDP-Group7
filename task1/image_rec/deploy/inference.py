"""Portable MDP recognizer using the lowres160 NCNN model in this folder."""
import argparse
import io
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageOps
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent


def load_upright(image):
    """Accept an image path, encoded JPEG/PNG bytes, file object, or BGR ndarray."""
    if isinstance(image, np.ndarray):
        return image
    if isinstance(image, (bytes, bytearray)):
        image = io.BytesIO(image)
    with Image.open(image) as source:
        rgb = np.array(ImageOps.exif_transpose(source).convert("RGB"))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def iou(a, b):
    intersection = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.0


class Recognizer:
    """Create once per server worker and reuse. Calls within a worker should be serial."""

    def __init__(self, weights=HERE, device="cpu", imgsz=640):
        if imgsz != 640:
            raise ValueError("This NCNN export requires imgsz=640")
        self.model = YOLO(str(weights), task="segment")
        self.device = device
        self.imgsz = imgsz

    def infer(self, image, conf=0.35, signal=None, include_marker=True, annotate=False):
        """Return (image_id or None, confidence, annotated BGR image or None).

        signal is None, 'C', 'L', or 'R'. 'marker' is a valid result meaning the
        required symbol is on another face. Camera images must be upright;
        EXIF orientation is honored, but we do not guess using four rotations.
        """
        if signal not in (None, "C", "L", "R"):
            raise ValueError("signal must be None, 'C', 'L', or 'R'")
        image = load_upright(image)
        result = self.model.predict(image, conf=conf, imgsz=self.imgsz,
                                    device=self.device, verbose=False)[0]
        rows = result.boxes.data.cpu().tolist()
        kept = []
        for row in sorted(rows, key=lambda row: -row[4]):
            if all(iou(row[:4], previous[:4]) < 0.5 for previous in kept):
                kept.append(row)
        candidates = []
        for x0, y0, x1, y1, score, cls in kept:
            name = self.model.names[int(cls)]
            if name == "marker" and not include_marker:
                continue
            center = (x0 + x1) / (2 * image.shape[1])
            if signal == "L" and center > 0.45:
                continue
            if signal == "R" and center < 0.55:
                continue
            if signal == "C" and not 0.25 < center < 0.75:
                continue
            candidates.append(((x1 - x0) * (y1 - y0), name, score))
        annotated = result.plot() if annotate else None
        if not candidates:
            return None, 0.0, annotated
        _, image_id, confidence = max(candidates, key=lambda candidate: candidate[0])
        return image_id, confidence, annotated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--signal", choices=("C", "L", "R"))
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--annotated", type=Path, help="optional debug overlay output path")
    args = parser.parse_args()
    recognizer = Recognizer(device=args.device, imgsz=args.imgsz)
    image_id, confidence, annotated = recognizer.infer(
        args.image, conf=args.conf, signal=args.signal, annotate=args.annotated is not None)
    if args.annotated is not None:
        args.annotated.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.annotated), annotated):
            raise OSError(f"Could not save {args.annotated}")
    print(json.dumps({"image_id": image_id, "confidence": confidence}))


if __name__ == "__main__":
    main()
