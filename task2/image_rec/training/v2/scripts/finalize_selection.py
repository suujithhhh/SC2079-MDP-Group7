"""Select retained checkpoints on full validation, then freeze before any test.

No test/arena images or results are read. Prediction caches require matching
model, validation records/image bytes, evaluation code, and runtime identities.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json, os, time

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT/"config"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
GRID = [round(i/100, 2) for i in range(45, 96, 5)]
NAMES = {0:"bullseye", 1:"left", 2:"right"}
CRITERION = ("Validation only, IoU0.50: minimize opposite directions, wrong classes, "
             "extra detections; then maximize correct arrows and total correct. "
             "Ties prefer lower confidence, earlier known checkpoint epoch, then "
             "checkpoint path. Unknown epochs sort after known epochs. "
             "Confidence grid0.45 through0.95 in0.05 steps.")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def atomic_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False)+"\n")
    temporary.replace(path)


def model_state_hash(model):
    """Equal floating values hash equally across FP16/FP32 storage formats."""
    h = hashlib.sha256()
    for name, tensor in sorted(model.state_dict().items()):
        tensor = tensor.detach().cpu().contiguous()
        if tensor.is_floating_point():
            tensor = tensor.float()
        h.update(json.dumps([name, list(tensor.shape), str(tensor.dtype)]).encode())
        h.update(tensor.numpy().tobytes())
    return h.hexdigest()


def epoch_metadata(checkpoint, path):
    raw = checkpoint.get("epoch")
    history = checkpoint.get("train_results") or {}
    values = history.get("epoch", []) if isinstance(history, dict) else []
    epochs = []
    if isinstance(values, (list, tuple)):
        for value in values:
            try:
                value = float(value)
                if value >= 1 and value.is_integer():
                    epochs.append(int(value))
            except (TypeError, ValueError):
                pass
    epoch, origin = None, "unknown"
    if isinstance(raw, int) and not isinstance(raw, bool) and raw >= 0:
        epoch, origin = raw+1, "checkpoint.epoch (zero-based) + 1"
    elif Path(path).name == "last.pt" and epochs:
        epoch, origin = max(epochs), "stripped last.pt train_results.epoch maximum (one-based)"
    return {"epoch":epoch, "epoch_origin":origin, "checkpoint_epoch_raw":raw,
            "training_history_last_epoch":max(epochs) if epochs else None,
            "epoch_note":"Stripped best.pt can contain final last.pt history; that history does not identify the epoch of its weights."}


def inspect_checkpoint(path):
    import torch
    before = sha256(path)
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    model = ckpt.get("ema")
    if model is None:
        model = ckpt.get("model")
    if model is None:
        raise ValueError(f"No EMA/model in {path}")
    names = {int(k):str(v) for k,v in model.names.items()}
    architecture = {"class_mapping":names, "model_class":type(model).__name__,
                    "head_class":type(model.model[-1]).__name__,
                    "scale":(getattr(model, "yaml", {}) or {}).get("scale"),
                    "end2end":bool(getattr(model, "end2end", True)),
                    "one2one_parameters_present":any("one2one" in k for k in model.state_dict()),
                    "parameters":sum(p.numel() for p in model.parameters())}
    if names != NAMES or architecture["end2end"] or architecture["one2one_parameters_present"]:
        raise ValueError(f"Incorrect class mapping or deployment head: {path}: {architecture}")
    if architecture["scale"] != "n" or architecture["model_class"] != "DetectionModel" or not 0 < architecture["parameters"] < 5_000_000:
        raise ValueError(f"Not the expected YOLO26 nano detector: {path}: {architecture}")
    info = {"path":str(path.resolve()), "checkpoint_sha256":before,
            "model_state_sha256":model_state_hash(model), "architecture":architecture,
            **epoch_metadata(ckpt, path)}
    if sha256(path) != before:
        raise RuntimeError(f"Checkpoint changed during inspection; finish training first: {path}")
    return info


def checkpoint_order(item):
    return (item["epoch"] if item["epoch"] is not None else 2**31, item["path"])


def discover_candidates(folder):
    paths = [folder/n for n in ("best_arrow.pt","best.pt","last.pt")]
    paths += list(folder.glob("epoch*.pt"))
    grouped = {}
    for path in sorted({p.resolve() for p in paths if p.is_file()}):
        item = inspect_checkpoint(path)
        grouped.setdefault(item["model_state_sha256"], []).append(item)
    if not grouped:
        raise FileNotFoundError(f"No retained checkpoints in {folder}")
    candidates = []
    for aliases in grouped.values():
        aliases.sort(key=checkpoint_order)
        candidates.append({**aliases[0], "aliases":aliases})
    return sorted(candidates, key=checkpoint_order)


def score(summary):
    total = summary["all"]
    arrows = sum(summary["by_class"][c]["correct"] for c in ("left","right"))
    return (total["wrong_direction"], total["wrong_class"],
            total["false_positives"], -arrows, -total["correct"])


def cached_predictions(candidate, records, common, folder, device="0", refresh=False):
    from evaluate import predict_records
    identity = {**common, "source_checkpoint":candidate["path"],
                "checkpoint_sha256":candidate["checkpoint_sha256"],
                "model_state_sha256":candidate["model_state_sha256"],
                "architecture":candidate["architecture"]}
    key = canonical_hash(identity)
    path = folder/f"checkpoint_{candidate['model_state_sha256'][:16]}_val.json"
    if path.exists() and not refresh:
        try:
            saved = json.loads(path.read_text()); rows = saved.get("records", [])
            matched = saved.get("cache_key") == key and len(rows) == len(records)
            matched = matched and all({k:row.get(k) for k in original} == original for row,original in zip(rows,records))
            if matched:
                print("FINAL_SELECTION_CACHE_VALID", Path(candidate["path"]).name, len(rows), flush=True)
                return rows, str(path)
            print("FINAL_SELECTION_CACHE_INVALID", path.name, "identity or records changed", flush=True)
        except (ValueError, TypeError, KeyError):
            print("FINAL_SELECTION_CACHE_INVALID", path.name, "unreadable cache", flush=True)
    if sha256(candidate["path"]) != candidate["checkpoint_sha256"]:
        raise RuntimeError("Checkpoint changed after discovery; stop training before selection")
    started = time.perf_counter()
    rows = predict_records(candidate["path"], records, 640, device)
    if sha256(candidate["path"]) != candidate["checkpoint_sha256"]:
        raise RuntimeError("Checkpoint changed during validation; no cache published")
    atomic_json(path, {"cache_key":key, "cache_identity":identity, "model":candidate["path"],
                       "split":"val", "imgsz":640, "end2end":False,
                       "seconds":time.perf_counter()-started, "records":rows})
    return rows, str(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="0")
    parser.add_argument("--refresh", action="store_true")
    args = parser.parse_args(argv)
    import torch, ultralytics
    from ultralytics.utils.torch_utils import strip_optimizer
    from evaluate import summarize, INFERENCE_CONFIDENCE
    torch.set_num_threads(4)
    dst = ROOT/"weights/best.pt"
    point_path = ROOT/"weights/operating_point.json"
    report_path = ROOT/"reports/checkpoint_selection.json"
    if any(p.exists() for p in (dst,point_path,report_path)):
        raise FileExistsError("Frozen or partial final artifacts exist; preserve and review before rerunning.")
    folder = ROOT/"reports/final_selection"; folder.mkdir(parents=True, exist_ok=True)
    manifest_path = ROOT/"data/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    records = [r for r in manifest["records"] if r["split"]=="val"]
    if len(records) != 1122:
        raise ValueError(f"Expected full expanded validation (1122 images), got {len(records)}")
    overlap = manifest.get("group_overlap", {})
    if isinstance(overlap, dict) and any(overlap.values()):
        raise ValueError("Manifest reports source-group overlap")
    images = {r["path"]:sha256(r["path"]) for r in records}
    common = {"validation_records_sha256":canonical_hash(records), "input_images_sha256":images,
              "evaluation_code_sha256":sha256(Path(__file__).with_name("evaluate.py")),
              "torch_version":str(torch.__version__), "ultralytics_version":ultralytics.__version__,
              "device":args.device, "imgsz":640, "raw_confidence":INFERENCE_CONFIDENCE,
              "nms_iou":.5, "max_det":50, "agnostic_nms":False, "end2end":False, "matching_iou":.5}
    candidates = discover_candidates(ROOT/"runs/yolo26n_task2_arrow_multi_v2/weights")
    atomic_json(folder/"candidate_inventory.json", {"deduplication":"normalized model-state SHA256",
                "confidence_grid":GRID, "criterion":CRITERION, "candidates":candidates,
                "validation_identity_sha256":canonical_hash(common), "validation_images":len(records),
                "test_used_for_selection":False, "arena_reference_used_for_selection":False})
    winner = None; results = []
    for candidate in candidates:
        rows, raw_cache = cached_predictions(candidate, records, common, folder, args.device, args.refresh)
        grid = []
        for confidence in GRID:
            summary = summarize(rows, confidence, .5); criterion_score = score(summary)
            key = (*criterion_score, confidence, *checkpoint_order(candidate))
            grid.append({"confidence":confidence, "score":list(criterion_score),
                         "summary":{k:v for k,v in summary.items() if k!="errors"}})
            if winner is None or key < winner["key"]:
                winner = {"key":key, "candidate":candidate, "confidence":confidence,
                          "score":criterion_score, "summary":summary, "raw_cache":raw_cache}
        result = {"source_checkpoint":candidate["path"], "checkpoint_sha256":candidate["checkpoint_sha256"],
                  "model_state_sha256":candidate["model_state_sha256"], "epoch":candidate["epoch"],
                  "epoch_origin":candidate["epoch_origin"], "aliases":candidate["aliases"],
                  "raw_cache":raw_cache, "grid":grid}
        results.append(result)
        atomic_json(folder/f"checkpoint_{candidate['model_state_sha256'][:16]}_scores.json", result)
        best_grid = min(grid, key=lambda item:(*item["score"],item["confidence"]))
        print("FINAL_SELECTION_CANDIDATE",Path(candidate["path"]).name,"epoch",candidate["epoch"],
              "confidence",best_grid["confidence"],"score",best_grid["score"],flush=True)
        torch.cuda.empty_cache()
    current_val = [r for r in json.loads(manifest_path.read_text())["records"] if r["split"]=="val"]
    if canonical_hash(current_val) != common["validation_records_sha256"] or any(sha256(p)!=h for p,h in images.items()):
        raise RuntimeError("Validation records or image bytes changed during selection")
    if any(sha256(a["path"])!=a["checkpoint_sha256"] for c in candidates for a in c["aliases"]):
        raise RuntimeError("Retained checkpoints changed during selection; finish training first")
    selected = winner["candidate"]; source = Path(selected["path"])
    temporary = dst.with_name("best.selection_pending.pt")
    if temporary.exists():
        raise FileExistsError(f"Preserve existing pending artifact: {temporary}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    strip_optimizer(source, s=str(temporary))
    final = inspect_checkpoint(temporary)
    if final["model_state_sha256"] != selected["model_state_sha256"]:
        raise RuntimeError("Stripping changed selected model tensors; pending artifact preserved")
    if dst.exists():
        raise FileExistsError("Frozen weights appeared during selection; preserving both artifacts")
    temporary.replace(dst)
    frozen_at = datetime.now(timezone.utc).isoformat()
    point = {"confidence":winner["confidence"], "imgsz":640, "selected_on":"validation only",
             "confirm_frames":3, "min_arrow_input_side":12.0, "frozen_at_utc":frozen_at,
             "checkpoint_sha256":sha256(dst), "confidence_grid":GRID, "matching_iou":.5}
    report = {"checkpoint":str(dst), "checkpoint_sha256":point["checkpoint_sha256"],
              "source_checkpoint":str(source), "source_checkpoint_sha256":selected["checkpoint_sha256"],
              "model_state_sha256":selected["model_state_sha256"], "selected_epoch":selected["epoch"],
              "selected_epoch_origin":selected["epoch_origin"], "selected_checkpoint_aliases":selected["aliases"],
              "warmstart_stage_epochs":7, "selected_total_finetuning_epochs":7+selected["epoch"] if selected["epoch"] is not None else None,
              "confidence":point["confidence"], "confidence_grid":GRID,
              "architecture":"YOLO26n detect one-to-many only, 3 classes",
              "architecture_verification":final["architecture"], "parameters":final["architecture"]["parameters"],
              "criterion":CRITERION, "score":list(winner["score"]), "validation":winner["summary"],
              "validation_images":len(records), "validation_identity_sha256":canonical_hash(common),
              "validation_records_sha256":common["validation_records_sha256"],
              "selected_raw_validation_cache":winner["raw_cache"], "candidate_count":len(candidates),
              "candidate_results":results, "frozen_at_utc":frozen_at,
              "test_used_for_selection":False, "arena_reference_used_for_selection":False,
              "retained_source_checkpoints_modified":False}
    atomic_json(point_path, point); atomic_json(report_path, report)
    atomic_json(folder/"selection_complete.json", {"checkpoint_sha256":point["checkpoint_sha256"],
                "selection_report_sha256":sha256(report_path), "operating_point_sha256":sha256(point_path),
                "selected_epoch":selected["epoch"], "confidence":point["confidence"], "frozen_at_utc":frozen_at})
    print(json.dumps({k:v for k,v in report.items() if k not in ("validation","candidate_results","selected_checkpoint_aliases")}, indent=2), flush=True)


if __name__ == "__main__":
    main()

