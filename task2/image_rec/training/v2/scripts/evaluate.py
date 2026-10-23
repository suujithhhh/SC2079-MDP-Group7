"""Evaluate Task 2 deployment heads with explicit misses and wrong directions.

Ground-truth and prediction boxes use xyxy pixels in each source image. Normal
Task 2 datasets are 640x640; image dimensions are retained to make this explicit.
The confidence threshold is applied after inference so cached records can be
rescored without running the network again.
"""
from pathlib import Path
from collections import defaultdict
import argparse
import json
import math
import os
import time

TASK2 = Path(__file__).resolve().parents[2]
os.environ.setdefault("YOLO_CONFIG_DIR", str(TASK2 / "v2" / "config"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
NAMES = ("bullseye", "left", "right")
INFERENCE_CONFIDENCE = 0.01
SIZE_BANDS = ("<12", "12-20", "20-32", "32-48", "48-80", "80+")


def box_iou(a, b):
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / max(union, 1e-12)


def size_band(side):
    for edge, name in zip((12, 20, 32, 48, 80), SIZE_BANDS):
        if side < edge:
            return name
    return SIZE_BANDS[-1]


def dimensions(obj):
    box = obj["box"]
    if len(box) != 4 or not all(math.isfinite(float(value)) for value in box):
        raise ValueError(f"Invalid box: {box}")
    width, height = box[2] - box[0], box[3] - box[1]
    if width <= 0 or height <= 0:
        raise ValueError(f"Non-positive box size: {box}")
    if int(obj["class"]) not in range(len(NAMES)):
        raise ValueError(f"Unknown Task 2 class: {obj['class']}")
    return min(width, height), max(width, height)


def empty_counts():
    return dict(targets=0, predictions=0, correct=0, missed=0, wrong_class=0,
                wrong_direction=0, false_positives=0, images=0,
                negative_images=0, negative_images_with_fp=0)


def summarize(records, conf=0.85, iou=0.5):
    """Match boxes greedily by IoU without filtering on their predicted class.

    Wrong-class matches are reported separately from unmatched false positives.
    Recall is correct/targets; precision is correct/all retained predictions.
    Size slices use a matched target's size, or an unmatched prediction's size.
    Class slices count targets by truth and predictions by predicted class.
    """
    if not 0 <= conf <= 1 or not 0 < iou <= 1:
        raise ValueError("Expected confidence in [0,1] and matching IoU in (0,1]")
    all_counts = empty_counts()
    by_kind = defaultdict(empty_counts)
    by_class = {name: empty_counts() for name in NAMES}
    by_size = {band: empty_counts() for band in SIZE_BANDS}
    by_min_size = {band: empty_counts() for band in SIZE_BANDS}
    errors = []
    for record in records:
        truth = record["objects"]
        predictions = [p for p in record["predictions"] if p["confidence"] >= conf]
        for obj in [*truth, *predictions]:
            dimensions(obj)
        kind = record.get("kind", "unspecified")
        basic = [all_counts, by_kind[kind]]
        candidates = sorted(
            ((box_iou(g["box"], p["box"]), gi, pi)
             for gi, g in enumerate(truth) for pi, p in enumerate(predictions)),
            key=lambda value: (-value[0], value[1], value[2]),
        )
        matched, used = {}, set()
        for overlap, gi, pi in candidates:
            if overlap < iou:
                break
            if gi not in matched and pi not in used:
                matched[gi] = (pi, overlap)
                used.add(pi)
        image_groups = {id(group): group for group in basic}

        def touch(groups):
            for group in groups:
                image_groups[id(group)] = group

        for prediction in predictions:
            group = by_class[NAMES[int(prediction["class"])]]
            group["predictions"] += 1
            touch([group])
        for group in basic:
            group["predictions"] += len(predictions)

        object_results = []
        for gi, target in enumerate(truth):
            match = matched.get(gi)
            prediction = predictions[match[0]] if match is not None else None
            correct = prediction is not None and int(prediction["class"]) == int(target["class"])
            wrong = prediction is not None and not correct
            wrong_direction = wrong and int(target["class"]) in (1, 2) and int(prediction["class"]) in (1, 2)
            minimum, maximum = dimensions(target)
            size_groups = [by_size[size_band(maximum)], by_min_size[size_band(minimum)]]
            groups = [*basic, by_class[NAMES[int(target["class"])]], *size_groups]
            touch(groups)
            for group in groups:
                group["targets"] += 1
                group["correct"] += int(correct)
                group["missed"] += int(prediction is None)
                group["wrong_class"] += int(wrong)
                group["wrong_direction"] += int(wrong_direction)
            for group in size_groups:
                group["predictions"] += int(prediction is not None)
            object_results.append({
                "target": target, "prediction": prediction,
                "status": "correct" if correct else "wrong_direction" if wrong_direction else "wrong_class" if wrong else "missed",
                "iou": match[1] if match is not None else None,
                "source_min_side": minimum, "source_max_side": maximum,
            })

        extras = [p for pi, p in enumerate(predictions) if pi not in used]
        for prediction in extras:
            minimum, maximum = dimensions(prediction)
            size_groups = [by_size[size_band(maximum)], by_min_size[size_band(minimum)]]
            groups = [*basic, by_class[NAMES[int(prediction["class"])]], *size_groups]
            touch(groups)
            for group in groups:
                group["false_positives"] += 1
            for group in size_groups:
                group["predictions"] += 1

        for group in image_groups.values():
            group["images"] += 1
            if not truth:
                group["negative_images"] += 1
                group["negative_images_with_fp"] += int(bool(predictions))
        if extras or any(result["status"] != "correct" for result in object_results):
            errors.append({"path": record["path"], "kind": kind,
                           "objects": object_results, "false_positive_predictions": extras})

    for group in [all_counts, *by_kind.values(), *by_class.values(), *by_size.values(), *by_min_size.values()]:
        group["recall"] = group["correct"] / group["targets"] if group["targets"] else None
        group["precision"] = group["correct"] / group["predictions"] if group["predictions"] else None
    return {
        "confidence": conf, "matching_iou": iou,
        "matching": "Class-independent, descending IoU, one-to-one greedy assignment",
        "size_reference": "Target min/max side in original source-image pixels; Task 2 data use 640x640 frames. Matched predictions use target size; unmatched predictions use predicted size.",
        "count_semantics": "correct + missed + wrong_class = targets; wrong_direction is a subset of wrong_class; false_positives counts unmatched predictions; precision = correct/predictions.",
        "all": all_counts, "by_kind": dict(by_kind), "by_class": by_class,
        "by_size": by_size, "by_min_size": by_min_size, "errors": errors,
    }


def predict_records(model_path, records, imgsz=640, device="0", output=None):
    """Return input records extended with deployment-head predictions.

    NCNN always uses CPU and four threads. Optional output writes a metadata
    object containing the returned records, suitable for later rescoring.
    """
    import cv2
    import numpy as np
    import torch
    from ultralytics import YOLO

    cv2.setNumThreads(1)
    torch.set_num_threads(4)
    records = list(records)
    is_ncnn = Path(model_path).is_dir()
    original_net = None
    if is_ncnn:
        import ncnn
        original_net = ncnn.Net

        def limited_net(*args, **kwargs):
            net = original_net(*args, **kwargs)
            net.opt.num_threads = 4
            return net

        ncnn.Net = limited_net
        device = "cpu"
    try:
        model = YOLO(str(model_path), task="detect")
        if is_ncnn:
            model.predict(np.zeros((imgsz, imgsz, 3), dtype=np.uint8), imgsz=imgsz,
                          device="cpu", conf=INFERENCE_CONFIDENCE, verbose=False,
                          rect=False, end2end=False)
    finally:
        if original_net is not None:
            ncnn.Net = original_net
    batch = 1 if str(device) == "cpu" else 32
    rows = []
    started = time.perf_counter()
    for offset in range(0, len(records), batch):
        selected = records[offset:offset + batch]
        results = model.predict(
            [str(record["path"]) for record in selected], imgsz=imgsz, device=device,
            conf=INFERENCE_CONFIDENCE, iou=0.5, agnostic_nms=False, max_det=50,
            batch=batch, verbose=False, rect=False, end2end=False,
        )
        if len(results) != len(selected):
            raise RuntimeError("Inference returned an unexpected number of images")
        for record, result in zip(selected, results):
            if [result.names[i] for i in range(3)] != list(NAMES):
                raise ValueError(f"Unexpected Task 2 class mapping: {result.names}")
            predictions = [
                {"class": int(cls), "confidence": float(confidence), "box": list(map(float, box))}
                for cls, confidence, box in zip(result.boxes.cls.cpu().tolist(),
                                                result.boxes.conf.cpu().tolist(),
                                                result.boxes.xyxy.cpu().tolist())
            ]
            rows.append({**record, "predictions": predictions,
                         "image_width": int(result.orig_shape[1]),
                         "image_height": int(result.orig_shape[0])})
        if offset % 128 == 0 or offset + batch >= len(records):
            print("EVAL_PROGRESS", Path(model_path).name, imgsz,
                  min(offset + batch, len(records)), len(records), flush=True)
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"model": str(model_path), "imgsz": imgsz,
                                   "device": str(device), "end2end": False,
                                   "inference_confidence": INFERENCE_CONFIDENCE,
                                   "seconds": time.perf_counter() - started,
                                   "records": rows}, indent=2) + "\n")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--split", default="val")
    parser.add_argument("--output", required=True)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.85)
    parser.add_argument("--iou", type=float, default=0.5)
    args = parser.parse_args()
    manifest = json.loads(Path(args.manifest).read_text())
    records = manifest["records"] if isinstance(manifest, dict) else manifest
    records = [record for record in records if args.split == "all" or record["split"] == args.split]
    if not records:
        raise ValueError(f"No records found for split {args.split!r}")
    started = time.perf_counter()
    predictions = predict_records(args.model, records, args.imgsz, args.device)
    report = {"model": args.model, "manifest": args.manifest, "split": args.split,
              "imgsz": args.imgsz, "device": "cpu" if Path(args.model).is_dir() else args.device,
              "end2end": False, "inference_confidence": INFERENCE_CONFIDENCE,
              "seconds": time.perf_counter() - started, "records": predictions,
              "summary": summarize(predictions, args.conf, args.iou)}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["summary"]["all"], indent=2), flush=True)


if __name__ == "__main__":
    main()
