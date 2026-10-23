"""Run selected known-success examples through the final packaged NCNN API.

This is an API smoke check, NOT an accuracy estimate. Examples are deliberately
selected from already-correct frozen test predictions. Complete test-set results
belong in REPORT.json. Each evidence event is one frame; no temporal confirmation
or robot action is simulated. Run only after export, comparison, and packaging.
"""
from collections import Counter
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import os

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKAGE = ROOT / "handoff/task2_arrow_v2_640_ncnn_model"
DEFAULT_CACHE = ROOT / "reports/v2_test_ncnn.json"
OUTPUT = ROOT / "reports/multiobject_smoke"
MODEL_FILES = ("model.ncnn.bin", "model.ncnn.param", "metadata.yaml")
NAMES = {0: "bullseye", 1: "left", 2: "right"}
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "config"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

from evaluate import summarize


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prediction_sizes(prediction, row, imgsz):
    box = prediction["box"]
    source = min(box[2] - box[0], box[3] - box[1])
    width, height = row.get("image_width", 640), row.get("image_height", 640)
    return (prediction.get("source_min_side", source),
            prediction.get("input_min_side", source * imgsz / max(width, height)))


def choose_examples(records, confidence, min_side, imgsz):
    """Choose four mixed-scene examples plus one bullseye-only example."""
    pools = {(cls, count): [] for cls in (1, 2) for count in (1, 2)}
    bull_only = []
    for row in records:
        expected = Counter(int(obj["class"]) for obj in row["objects"])
        if not expected:
            continue
        mixed = (expected[1] + expected[2] == 1 and expected[0] in (1, 2)
                 and sum(expected.values()) == 1 + expected[0])
        only_bull = expected[0] > 0 and sum(expected.values()) == expected[0]
        if not mixed and not only_bull:
            continue
        score = summarize([row], confidence, 0.5)["all"]
        if score["correct"] != len(row["objects"]) or score["false_positives"]:
            continue
        predictions = [p for p in row["predictions"] if p["confidence"] >= confidence]
        if only_bull:
            bull_only.append((row, min(p["confidence"] for p in predictions)))
            continue
        arrow = next(p for p in predictions if p["class"] in (1, 2))
        source_side, input_side = prediction_sizes(arrow, row, imgsz)
        if min(source_side, input_side) < min_side:
            continue
        marker_confidence = max(p["confidence"] for p in predictions if p["class"] == 0)
        item = {"row": row, "bull_confidence_above_arrow": marker_confidence > arrow["confidence"],
                "cached_arrow_confidence": arrow["confidence"],
                "cached_highest_bull_confidence": marker_confidence,
                "minimum_target_confidence": min(p["confidence"] for p in predictions),
                "arrow_min_side": min(source_side, input_side)}
        pools[(int(arrow["class"]), expected[0])].append(item)
    chosen, missing = [], []
    for (cls, count), pool in pools.items():
        case = f"{NAMES[cls]}_{count}bull"
        if not pool:
            missing.append(case)
            continue
        pool.sort(key=lambda item: (not item["bull_confidence_above_arrow"],
                                    -item["minimum_target_confidence"],
                                    -item["arrow_min_side"], item["row"]["path"]))
        chosen.append({**pool[0], "case": case, "arrow_class": cls,
                       "bullseye_count": count, "eligible_cached_examples": len(pool),
                       "higher_confidence_bull_example_available": any(
                           item["bull_confidence_above_arrow"] for item in pool)})
    if bull_only:
        bull_only.sort(key=lambda pair: (-pair[1], pair[0]["path"]))
        row = bull_only[0][0]
        chosen.append({"case": "bullseye_only", "row": row, "arrow_class": None,
                       "bullseye_count": len(row["objects"]),
                       "eligible_cached_examples": len(bull_only),
                       "higher_confidence_bull_example_available": None})
    else:
        missing.append("bullseye_only")
    return chosen, missing


def run(package=DEFAULT_PACKAGE, cache_path=DEFAULT_CACHE):
    package, cache_path = Path(package).resolve(), Path(cache_path).resolve()
    point_path = package / "operating_point.json"
    frozen_point_path = ROOT / "weights/operating_point.json"
    runner_path = package / "inference.py"
    cache = json.loads(cache_path.read_text())
    point = json.loads(point_path.read_text())
    package_manifest = json.loads((package / "manifest.json").read_text())
    identity = cache["cache_inputs"]
    confidence = float(point["confidence"])
    if sha256(point_path) != sha256(frozen_point_path):
        raise RuntimeError("Packaged operating point differs from the frozen selection")
    if cache["split"] != "test" or cache["runtime"] != "NCNN":
        raise ValueError("Expected the completed NCNN test prediction cache")
    if cache["frozen_confirmation_confidence"] != confidence:
        raise RuntimeError("Cached and packaged confidence points differ")
    for name in MODEL_FILES:
        actual = sha256(package / name)
        if actual != identity["model_sha256"][name]:
            raise RuntimeError(f"Packaged model differs from evaluated cache: {name}")
        if actual != package_manifest["files"][name]:
            raise RuntimeError(f"Packaged model checksum differs from manifest: {name}")
    if sha256(runner_path) != identity["runner_sha256"]:
        raise RuntimeError("Packaged runner differs from the evaluated runner")
    for name, expected in package_manifest["files"].items():
        if sha256(package / name) != expected:
            raise RuntimeError(f"Packaged payload checksum mismatch: {name}")
    tracked = {"test_cache": cache_path, "operating_point": point_path,
               "frozen_operating_point": frozen_point_path, "runner": runner_path,
               "package_manifest": package / "manifest.json",
               **{name: package / name for name in MODEL_FILES}}
    before = {name: sha256(path) for name, path in tracked.items()}
    spec = importlib.util.spec_from_file_location("task2_packaged_smoke_runner", runner_path)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    min_side = max(12.0, float(point.get("min_arrow_input_side", 12.0)))
    examples, missing = choose_examples(cache["records"], confidence, min_side,
                                       int(point.get("imgsz", 640)))
    if not examples:
        raise RuntimeError("No suitable known-success cached examples for this API smoke check")
    detector = runner.Detector(package, threads=4)
    if detector.conf != confidence or detector.loaded_net_threads != 4:
        raise RuntimeError("Package did not load the expected confidence and four NCNN threads")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    report = {"status": "RUNNING", "purpose": "Selected-success NCNN package API smoke; not an accuracy estimate",
              "selection_note": "Examples were deliberately selected from already-correct frozen test predictions. Complete test-set accuracy is reported separately. No model or operating threshold is selected or tuned here.",
              "package": str(package), "cache": str(cache_path), "hashes": before,
              "confirmation_confidence": confidence, "min_arrow_side_for_examples": min_side,
              "ncnn_threads": detector.loaded_net_threads, "missing_example_categories": missing,
              "single_frame_only": True, "temporal_confirmation_tested": False,
              "robot_action_performed": False, "cases": []}
    try:
        for example in examples:
            row = example["row"]
            source = Path(row["path"])
            image_hash = sha256(source)
            if identity["input_images_sha256"][str(source)] != image_hash:
                raise RuntimeError(f"Image changed since cached test inference: {source}")
            detections = detector.predict(source)
            actual_row = {**row, "predictions": [
                {"class": d["class_id"], "confidence": d["confidence"], "box": d["bbox"]}
                for d in detections]}
            metric = summarize([actual_row], confidence, 0.5)["all"]
            checks = {
                "all_objects_matched_correctly": metric["correct"] == len(row["objects"]),
                "no_extra_detections": metric["false_positives"] == 0,
                "class_counts_match": Counter(d["class_id"] for d in detections) ==
                                      Counter(o["class"] for o in row["objects"]),
                "deterministic_arrow_first_order": detections == runner.order_detections(detections),
            }
            frame = runner.load_bgr(source)
            roi = [0, 0, frame.shape[1], frame.shape[0]]
            observation = runner.select_arrow(detections, roi, detector.conf,
                                              detector.min_arrow_input_side)
            confirmation = runner.ArrowConfirmation(conf=detector.conf,
                                                    min_arrow_input_side=detector.min_arrow_input_side)
            decision = confirmation.update(detections, roi, frame_id=example["case"])
            checks["single_frame_not_confirmed"] = decision["confirmed"] is False and decision["direction"] is None
            if example["arrow_class"] is not None:
                cls = example["arrow_class"]
                checks["first_detection_is_expected_arrow"] = bool(detections) and detections[0]["class_id"] == cls
                checks["returned_robot_id_correct"] = bool(detections) and detections[0]["image_id"] == {1: "39", 2: "38"}[cls]
                checks["single_arrow_observed_despite_bullseyes"] = observation["direction"] == NAMES[cls]
                checks["confirmation_streak_is_one"] = decision["streak"] == 1
                arrows = [d for d in detections if d["class_id"] in (1, 2)]
                bulls = [d for d in detections if d["class_id"] == 0]
                actual_bull_above = bool(arrows and bulls and
                                        max(d["confidence"] for d in bulls) > arrows[0]["confidence"])
            else:
                checks["bullseye_only_never_turns"] = observation["direction"] is None and decision["streak"] == 0
                actual_bull_above = None
            evidence = Path(runner.save_evidence(source, detections, decision, OUTPUT / "evidence",
                {"case": example["case"], "selection": "Known-success API smoke, not accuracy evaluation",
                 "source": str(source), "source_sha256": image_hash,
                 "expected_objects": row["objects"], "single_frame_only": True,
                 "confirmation_confidence": confidence}))
            metadata = json.loads((evidence / "metadata.json").read_text())
            checks["raw_evidence_bytes_preserved"] = sha256(evidence / metadata["raw_file"]) == image_hash
            case = {key: value for key, value in example.items() if key != "row"}
            case.update(source=str(source), source_sha256=image_hash, detections=detections,
                        observation=observation, single_frame_decision=decision,
                        actual_bull_confidence_above_arrow=actual_bull_above,
                        checks=checks, evidence=str(evidence),
                        evidence_sha256={p.name: sha256(p) for p in evidence.iterdir() if p.is_file()})
            report["cases"].append(case)
            if not all(checks.values()):
                raise AssertionError(f"Package API smoke failed for {example['case']}: {checks}")
        after = {name: sha256(path) for name, path in tracked.items()}
        if after != before:
            raise RuntimeError("Tracked model, operating point, runner, or cache changed during the smoke check")
        report["artifacts_unchanged"] = True
        mixed = [case for case in report["cases"] if case["arrow_class"] is not None]
        report["higher_confidence_bull_case_verified"] = any(case["actual_bull_confidence_above_arrow"] for case in mixed)
        report["higher_confidence_bull_note"] = (
            "At least one actual inference returned a higher-confidence bullseye after its arrow."
            if report["higher_confidence_bull_case_verified"] else
            "No qualifying cached higher-confidence-bull example was available or reproduced; no such example was fabricated.")
        report["status"] = "PASS" if not missing else "INCOMPLETE_MISSING_EXAMPLES"
    except Exception as error:
        report.update(status="FAIL", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        (OUTPUT / "smoke.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "cases": len(report["cases"]),
                      "report": str(OUTPUT / "smoke.json"),
                      "higher_confidence_bull_case_verified": report.get("higher_confidence_bull_case_verified"),
                      "missing_example_categories": missing}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=DEFAULT_PACKAGE)
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    report = run(args.package, args.cache)
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
