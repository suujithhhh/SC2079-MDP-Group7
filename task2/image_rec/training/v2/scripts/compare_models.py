"""Compare frozen V1/V2 Task 2 NCNN models; never select or tune a threshold.

Run only after V2 checkpoint selection and export are complete. Writes every
artifact below task2/v2/reports and reads Task 2 V1 without modifying its files.
"""
import argparse
import hashlib
import importlib.util
import json
import os
import platform
import statistics
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V1 = ROOT.parent
REPORTS = ROOT / "reports"
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "config"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

from evaluate import box_iou, summarize
from package_model import environment_versions, read_operating_point, sha256, verify_export

RAW_CONFIDENCE = 0.01
ARENA_OBSERVATION_CONFIDENCE = 0.35


def load_runner():
    path = ROOT / "scripts/inference.py"
    spec = importlib.util.spec_from_file_location("task2_v2_inference", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, sha256(path)


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def model_hashes(folder):
    return {name: sha256(Path(folder) / name)
            for name in ("model.ncnn.bin", "model.ncnn.param", "metadata.yaml")}


def input_hashes(records):
    """Image bytes matter even when a frozen manifest points at symlinked replay."""
    return {record["path"]: sha256(record["path"]) for record in records}


def cached_predictions(label, detector, records, split, cache_inputs, refresh=False):
    path = REPORTS / f"{label}_{split}_ncnn.json"
    identity = {**cache_inputs, "split": split, "records_sha256": canonical_hash(records),
                "input_images_sha256": input_hashes(records), "imgsz": 640,
                "raw_confidence": RAW_CONFIDENCE, "confirmation_confidence": detector.conf,
                "nms_iou": 0.5, "max_det": 20, "class_agnostic_nms": False,
                "end2end": False, "threads": 4}
    key = canonical_hash(identity)
    if path.exists() and not refresh:
        saved = json.loads(path.read_text())
        if saved.get("cache_key") == key and len(saved.get("records", [])) == len(records):
            print("CACHE_VALID", label, split, len(records), flush=True)
            return saved["records"]
    import cv2
    rows = []
    started = time.perf_counter()
    for index, record in enumerate(records):
        frame = cv2.imread(record["path"])
        if frame is None or frame.shape[:2] != (640, 640):
            raise ValueError(f"Expected a readable 640x640 dataset frame: {record['path']}")
        detections = detector.predict(frame, observation_conf=RAW_CONFIDENCE)
        predictions = [{"class": d["class_id"], "confidence": d["confidence"], "box": d["bbox"],
                        "source_min_side": d["source_min_side"], "input_min_side": d["input_min_side"]}
                       for d in detections]
        rows.append({**record, "image_width": 640, "image_height": 640, "predictions": predictions})
        if index % 128 == 0 or index + 1 == len(records):
            print("NCNN_COMPARE_PROGRESS", label, split, index + 1, len(records), flush=True)
    payload = {"cache_key": key, "cache_inputs": identity, "model": str(detector.model_dir),
               "split": split, "runtime": "NCNN", "imgsz": 640, "threads": detector.loaded_net_threads,
               "raw_prediction_confidence": RAW_CONFIDENCE,
               "frozen_confirmation_confidence": detector.conf,
               "seconds_including_file_decode": time.perf_counter() - started, "records": rows}
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return rows


def mixed_bullseye_count(row):
    """Identify requested mixed scenes from annotations, independent of kind names."""
    objects = row["objects"]
    arrows = sum(int(obj["class"]) in (1, 2) for obj in objects)
    bullseyes = sum(int(obj["class"]) == 0 for obj in objects)
    return bullseyes if arrows == 1 and bullseyes in (1, 2) else 0


def scene_metrics(records, confidence, matching_iou):
    counts = {key: 0 for key in ("images", "positive_scenes", "arrow_scenes",
                                 "arrow_correct_scenes", "first_returned_arrow_correct_scenes",
                                 "all_objects_correct_scenes",
                                 "all_objects_correct_no_extra_scenes", "negative_scenes",
                                 "negative_scenes_without_detections")}
    for row in records:
        truth = row["objects"]
        predictions = [p for p in row["predictions"] if p["confidence"] >= confidence]
        candidates = sorted(((box_iou(g["box"], p["box"]), gi, pi)
                             for gi, g in enumerate(truth) for pi, p in enumerate(predictions)),
                            key=lambda item: (-item[0], item[1], item[2]))
        matched, used = {}, set()
        for overlap, gi, pi in candidates:
            if overlap < matching_iou:
                break
            if gi not in matched and pi not in used:
                matched[gi] = pi
                used.add(pi)
        correct = {gi for gi, pi in matched.items() if truth[gi]["class"] == predictions[pi]["class"]}
        arrows = {gi for gi, obj in enumerate(truth) if obj["class"] in (1, 2)}
        counts["images"] += 1
        counts["positive_scenes"] += bool(truth)
        counts["arrow_scenes"] += bool(arrows)
        counts["arrow_correct_scenes"] += bool(arrows) and arrows.issubset(correct)
        counts["first_returned_arrow_correct_scenes"] += any(
            gi in correct and matched.get(gi) == 0 for gi in arrows)
        all_correct = bool(truth) and len(correct) == len(truth)
        counts["all_objects_correct_scenes"] += all_correct
        counts["all_objects_correct_no_extra_scenes"] += all_correct and len(used) == len(predictions)
        counts["negative_scenes"] += not truth
        counts["negative_scenes_without_detections"] += not truth and not predictions
    counts["arrow_scene_recall"] = (counts["arrow_correct_scenes"] / counts["arrow_scenes"]
                                    if counts["arrow_scenes"] else None)
    counts["first_returned_arrow_correct_rate"] = (
        counts["first_returned_arrow_correct_scenes"] / counts["arrow_scenes"]
        if counts["arrow_scenes"] else None)
    counts["all_object_scene_recall"] = (counts["all_objects_correct_scenes"] / counts["positive_scenes"]
                                         if counts["positive_scenes"] else None)
    counts["strict_all_object_scene_recall"] = (
        counts["all_objects_correct_no_extra_scenes"] / counts["positive_scenes"]
        if counts["positive_scenes"] else None)
    counts["note"] = "All annotated arrows/objects must match their correct class at the stated IoU; strict additionally excludes unmatched predictions. First-returned counts use the shared V2 arrow-priority runner's order after frozen-confidence filtering. No temporal confirmation is simulated."
    return counts


def summarize_subsets(records, confidence, matching_iou=0.5):
    selectors = {
        "all": lambda row: True,
        "v1_regression": lambda row: row["kind"].startswith("regression_"),
        "v1_regression_real": lambda row: row["kind"] == "regression_real",
        "v1_regression_synthetic": lambda row: row["kind"] == "regression_synthetic",
        "new_black_on_white": lambda row: row["kind"].startswith("white_card_") or bool(mixed_bullseye_count(row)),
        "multi_1bull": lambda row: mixed_bullseye_count(row) == 1,
        "multi_2bull": lambda row: mixed_bullseye_count(row) == 2,
    }
    reports = {}
    for name, predicate in selectors.items():
        selected = [row for row in records if predicate(row)]
        reports[name] = summarize(selected, confidence, matching_iou)
        reports[name]["scene_metrics"] = scene_metrics(selected, confidence, matching_iou)
    return reports


def cpu_name():
    source = Path("/proc/cpuinfo")
    if source.exists():
        for line in source.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor() or platform.machine()


def benchmark(detector, frames, paths):
    """Time actual Detector.predict on the same preloaded frames for each model."""
    import numpy as np
    if len(frames) != 8:
        raise ValueError("Benchmark requires exactly eight preloaded frames")
    for index in range(5):
        detector.predict(frames[index % len(frames)])
    timings = []
    for index in range(32):
        start = time.perf_counter()
        detector.predict(frames[index % len(frames)])
        timings.append((time.perf_counter() - start) * 1000)
    return {"median_ms": statistics.median(timings), "p90_ms": float(np.percentile(timings, 90)),
            "mean_ms": statistics.mean(timings), "min_ms": min(timings), "max_ms": max(timings),
            "samples_ms": timings, "iterations": 32, "warmup_iterations": 5, "loaded_frames": 8,
            "frame_paths": paths, "threads": detector.loaded_net_threads,
            "confidence": detector.conf, "imgsz": detector.imgsz, "runtime": "NCNN",
            "cpu": cpu_name(), "platform": platform.platform(), "is_pi_measurement": False,
            "includes": "Detector.predict: input validation, preprocessing, NCNN, NMS, result conversion",
            "excludes": "model loading, image file decode, camera capture, transfer, navigation"}


def arena_reference(label, detector, runner, arena):
    frame = runner.load_bgr(arena)
    candidates = detector.predict(frame, observation_conf=ARENA_OBSERVATION_CONFIDENCE)
    observations = []
    height, width = frame.shape[:2]
    for candidate in candidates:
        x0, y0, x1, y1 = candidate["bbox"]
        roi = [max(0, x0 - 3), max(0, y0 - 3), min(width, x1 + 3), min(height, y1 + 3)]
        # Retain every candidate in the ROI so conflicting outputs cause abstention.
        observation = runner.select_arrow(candidates, roi, conf=detector.conf,
                                           min_arrow_input_side=detector.min_arrow_input_side)
        observations.append({"candidate": candidate, "roi": roi, "single_frame_observation": observation})
    decision = {"direction": None, "confirmed": False, "reason": "single_arena_reference_photo_only"}
    context = {"model": str(detector.model_dir), "label": label,
               "observation_confidence": ARENA_OBSERVATION_CONFIDENCE,
               "confirmation_confidence": detector.conf,
               "min_arrow_input_side": detector.min_arrow_input_side,
               "used_for_selection_or_tuning": False}
    evidence = runner.save_evidence(arena, candidates, decision, REPORTS / f"arena_{label}", context)
    return {"source": str(arena), "source_sha256": sha256(arena), "frame_shape": list(frame.shape),
            "observation_confidence": ARENA_OBSERVATION_CONFIDENCE,
            "confirmation_confidence": detector.conf, "all_candidates": candidates,
            "single_frame_observations_by_roi": observations,
            "confirmed_direction": None, "evidence": evidence, "used_for_tuning": False,
            "note": "One overhead reference screenshot, not a live Pi frame sequence or independent accuracy test."}


def text_report(report):
    lines = ["Task 2 V1 versus frozen arrow V2: actual 640 NCNN comparison", "",
             "Frame-level detections at each model's own frozen confidence; IoU >= 0.50.",
             "These counts do not establish three-frame confirmed turns or real Pi driving performance.", ""]
    for split in ("val", "test"):
        lines.append(split.upper())
        for subset in ("all", "v1_regression_real", "v1_regression_synthetic", "new_black_on_white",
                       "multi_1bull", "multi_2bull"):
            descriptions = []
            for label in ("v1", "v2"):
                item = report["models"][label]
                counts = item["splits"][split]["iou_0.50"][subset]["all"]
                descriptions.append(f"{label} conf {item['confidence']}: {counts['correct']}/{counts['targets']} correct, "
                                    f"{counts['wrong_direction']} opposite arrows, {counts['wrong_class']} total wrong classes, "
                                    f"{counts['false_positives']} unmatched detections")
            lines.append(f"  {subset}: " + " | ".join(descriptions))
            if subset.startswith("multi_"):
                scenes = []
                for label in ("v1", "v2"):
                    item = report["models"][label]["splits"][split]["iou_0.50"][subset]["scene_metrics"]
                    scenes.append(f"{label}: arrow correct in {item['arrow_correct_scenes']}/{item['arrow_scenes']} scenes; "
                                  f"first returned arrow correct in {item['first_returned_arrow_correct_scenes']}/{item['arrow_scenes']}; "
                                  f"all objects correct in {item['all_objects_correct_scenes']}/{item['positive_scenes']}; "
                                  f"all correct with no extras in {item['all_objects_correct_no_extra_scenes']}/{item['positive_scenes']}")
                lines.append("    " + " | ".join(scenes))
        lines.append("")
    for label in ("v1", "v2"):
        item = report["models"][label]
        timing = item["host_benchmark"]
        lines.append(f"{label}: host median {timing['median_ms']:.2f} ms, p90 {timing['p90_ms']:.2f} ms; "
                     "eight identical input frames, four threads, not Pi timing.")
        candidates = item["arena_reference"]["all_candidates"]
        descriptions = [f"{row['name']} {row['confidence']:.3f}" for row in candidates]
        lines.append(f"{label} arena early candidates at 0.35: " + (", ".join(descriptions) or "none"))
    lines.extend(["", "LIMITATIONS", *["- " + item for item in report["limitations"]]])
    return "\n".join(lines) + "\n"


def compare(v2_model, v1_model, manifest_path, operating_point_path, include_iou03=False, refresh=False):
    """Evaluate provided frozen artifacts. Checkpoint/threshold selection is out of scope."""
    import cv2
    cv2.setNumThreads(1)
    REPORTS.mkdir(parents=True, exist_ok=True)
    point = read_operating_point(operating_point_path)
    if point["confidence"] < RAW_CONFIDENCE:
        raise ValueError("Frozen threshold is below cached raw-prediction confidence")
    provenance = json.loads((Path(v2_model) / "export_provenance.json").read_text())
    selected_checkpoint_hash = sha256(ROOT / "weights/best.pt")
    if provenance["checkpoint_sha256"] != selected_checkpoint_hash:
        raise ValueError("V2 NCNN export does not match frozen weights/best.pt")
    manifest = json.loads(Path(manifest_path).read_text())
    split_records = {split: [row for row in manifest["records"] if row["split"] == split]
                     for split in ("val", "test")}
    if any(not rows for rows in split_records.values()):
        raise ValueError("Both validation and test records are required")
    overlap = manifest.get("group_overlap", {})
    if isinstance(overlap, dict) and any(overlap.values()):
        raise ValueError("Manifest reports overlapping source groups")
    timing_rows = [row for row in split_records["test"] if row["kind"] == "regression_real"][:8]
    if len(timing_rows) != 8:
        raise ValueError("Need eight regression_real test frames for the fixed timing benchmark")
    timing_paths = [row["path"] for row in timing_rows]
    frames = [cv2.imread(path) for path in timing_paths]
    if any(frame is None for frame in frames):
        raise ValueError("Unreadable benchmark image")
    runner, runner_hash = load_runner()
    model_specs = {"v1": (Path(v1_model).resolve(), 0.85, 12.0),
                   "v2": (Path(v2_model).resolve(), point["confidence"], point.get("min_arrow_input_side", 12.0))}
    arena = next(path for path in (V1 / "description").iterdir() if "10.48.05" in path.name)
    report = {"manifest": str(Path(manifest_path).resolve()), "manifest_sha256": sha256(manifest_path),
              "v2_operating_point": point, "v2_operating_point_sha256": sha256(operating_point_path),
              "v2_selected_checkpoint_sha256": selected_checkpoint_hash,
              "v2_export_provenance": provenance,
              "runtime_environment": environment_versions(),
              "runner_sha256": runner_hash, "selection_or_tuning_performed": False,
              "shared_runner_behavior": "Both weight sets use the V2 runner: arrows first, all detections retained. This compares weights rather than historical application ordering.",
              "metric_scope": "Per-frame detections before temporal confirmation or arrow-size decision gating",
              "models": {}, "limitations": [
                  "No live Task 2 Pi camera frames or physical detection-distance measurements are available.",
                  "All new black-on-white cases are synthetic or restyled, not newly captured Task 2 frames.",
                  "The original V1 regression test has been evaluated previously; report it separately from the new subset.",
                  "Source groups are disjoint, but different groups may still share capture-session context.",
                  "Confidence points were frozen before this comparison. Test and arena results are not used to tune them.",
                  "Detection scores do not measure three-frame confirmed decisions, autonomous navigation, or parking.",
                  "Host timing excludes camera capture and transfer and is not Pi timing.",
                  "Arena candidate observations at 0.35 are qualitative and are not accepted-ID accuracy rates.",
              ]}
    for label, (folder, confidence, min_side) in model_specs.items():
        metadata = verify_export(folder)
        detector = runner.Detector(folder, conf=confidence, threads=4)
        detector.min_arrow_input_side = min_side  # read the already-frozen point supplied above
        if detector.loaded_net_threads != 4:
            raise RuntimeError("NCNN did not initialize with four threads")
        hashes = model_hashes(folder)
        item = {"model": str(folder), "confidence": confidence, "min_arrow_input_side": min_side,
                "model_sha256": hashes, "metadata": metadata, "splits": {}}
        for split, records in split_records.items():
            rows = cached_predictions(label, detector, records, split,
                                      {"model_sha256": hashes, "runner_sha256": runner_hash,
                                       "manifest_sha256": report["manifest_sha256"],
                                       "runtime_environment": report["runtime_environment"]}, refresh)
            item["splits"][split] = {"iou_0.50": summarize_subsets(rows, confidence, 0.5)}
            if include_iou03:
                item["splits"][split]["iou_0.30"] = summarize_subsets(rows, confidence, 0.3)
        item["host_benchmark"] = benchmark(detector, frames, timing_paths)
        item["arena_reference"] = arena_reference(label, detector, runner, arena)
        report["models"][label] = item
        (REPORTS / f"{label}_comparison.json").write_text(json.dumps(item, indent=2) + "\n")
        del detector
    (REPORTS / "REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    (REPORTS / "COMPARISON.txt").write_text(text_report(report))
    print("FROZEN_NCNN_COMPARISON_COMPLETE", REPORTS / "REPORT.json", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2-model", type=Path, default=ROOT / "weights/task2_arrow_v2_640_ncnn_model")
    parser.add_argument("--v1-model", type=Path, default=V1 / "weights/task2_640_ncnn_model")
    parser.add_argument("--manifest", type=Path, default=ROOT / "data/manifest.json")
    parser.add_argument("--operating-point", type=Path, default=ROOT / "weights/operating_point.json")
    parser.add_argument("--include-iou03", action="store_true")
    parser.add_argument("--refresh", action="store_true", help="ignore verified raw prediction caches")
    args = parser.parse_args()
    compare(args.v2_model, args.v1_model, args.manifest, args.operating_point, args.include_iou03, args.refresh)


if __name__ == "__main__":
    main()
