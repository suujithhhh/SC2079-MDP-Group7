"""Fine-tune lowres160 with circle/6 data; evaluate fixed-threshold regressions.

Default creates a candidate, never replaces either deployed handoff.
Run with PYTHONPATH=.circle_deps if using the isolated dependencies.
"""
import argparse
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault("PIN_MEMORY", "False")
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp")

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from circle_data import ROOT, NAMES, label_for, labels


def overlap(a, b):
    wh = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]))
    inter = float(np.prod(wh))
    return inter / max(float(np.prod(a[2:] - a[:2]) + np.prod(b[2:] - b[:2]) - inter), 1e-9)


def summarize(records):
    counts = defaultdict(Counter)
    for row in records:
        group = counts[row["expected"] or "empty"]
        group["images"] += 1
        group["correct_return"] += row["returned"] == row["expected"]
        group["correct_localized"] += row["matched"] == row["expected"] and row["expected"] is not None
        group["returned_16"] += row["returned"] == "16"
        group["returned_40"] += row["returned"] == "40"
        group["miss"] += row["returned"] is None
    return {cls: {**dict(c), "return_accuracy": c["correct_return"] / c["images"],
                  "localized_recall": c["correct_localized"] / c["images"]}
            for cls, c in counts.items()}


def evaluate(weights, paths, out, device="0", batch=32, end2end=None):
    """Mirror handoff's confidence/dedup/largest-box rules, with no C/L/R signal.

    Also match boxes to ground truth to distinguish localization from returned-ID errors.
    Validation is at a FIXED 0.35 confidence, not an automatically chosen F1 threshold.
    """
    model = YOLO(str(weights), task="segment")
    records = []
    for start in range(0, len(paths), batch):
        chunk = paths[start:start + batch]
        predictions = model.predict(source=[str(p) for p in chunk], imgsz=640, device=device,
                                    conf=0.01, iou=0.7, rect=False, verbose=False, stream=True,
                                    end2end=end2end)
        for path, result in zip(chunk, predictions):
            ann = labels(label_for(path))
            if len(ann) > 1:
                raise ValueError(f"Expected single-object/negative evaluation image: {path}")
            boxes = result.boxes.data.cpu().numpy()
            kept = []
            for b in sorted(boxes, key=lambda b: -b[4]):
                if b[4] >= 0.35 and all(overlap(b[:4], k[:4]) < 0.5 for k in kept):
                    kept.append(b)
            top = max(kept, key=lambda b: (b[2]-b[0])*(b[3]-b[1])) if kept else None
            match = None
            if ann:
                cls, poly = ann[0]
                h, w = result.orig_shape
                gt = np.r_[poly.min(0) * [w,h], poly.max(0) * [w,h]]
                matching = [b for b in kept if overlap(b[:4],gt) >= 0.5]
                match = max(matching,key=lambda b:b[4]) if matching else None
            records.append({"path": str(path), "expected": NAMES[ann[0][0]] if ann else None,
                            "returned": NAMES[int(top[5])] if top is not None else None,
                            "confidence": float(top[4]) if top is not None else 0,
                            "matched": NAMES[int(match[5])] if match is not None else None,
                            "circle_candidate_conf": max([float(b[4]) for b in boxes if int(b[5])==29],default=0),
                            "six_candidate_conf": max([float(b[4]) for b in boxes if int(b[5])==5],default=0)})
        if start % 256 == 0:
            print(f"evaluate {out.stem}: {min(start+batch,len(paths))}/{len(paths)}", flush=True)
    result = {"weights": str(weights), "confidence": 0.35, "network_imgsz": 640,
              "requested_end2end": end2end,
              "per_class": summarize(records), "records": records}
    out.write_text(json.dumps(result, indent=2) + "\n")
    del model
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    return result


def slice_metrics(report):
    real = [r for r in report["records"] if "/lowres_data_160/val/" in r["path"]]
    stress = [r for r in report["records"] if "/lowres_data_160/val/" not in r["path"]]
    return {"original_160": summarize(real), "synthetic_stress": summarize(stress)}


def gates(baseline, candidate):
    old, new = slice_metrics(baseline), slice_metrics(candidate)
    noncircle = [c for c in old["original_160"] if c not in ("40", "empty")]
    total = sum(old["original_160"][c]["images"] for c in noncircle)
    before = sum(old["original_160"][c]["correct_return"] for c in noncircle) / total
    after = sum(new["original_160"][c]["correct_return"] for c in noncircle) / total
    regressions = {c: old["original_160"][c]["correct_return"] - new["original_160"][c]["correct_return"]
                   for c in noncircle if old["original_160"][c]["correct_return"] > new["original_160"][c]["correct_return"]}
    old_circle, new_circle = old["synthetic_stress"]["40"], new["synthetic_stress"]["40"]
    old_six, new_six = old["synthetic_stress"]["16"], new["synthetic_stress"]["16"]
    old_empty, new_empty = old["synthetic_stress"]["empty"], new["synthetic_stress"]["empty"]
    other_stress = [c for c in old["synthetic_stress"] if c not in ("40", "16", "empty")]
    old_other = sum(old["synthetic_stress"][c]["correct_return"] for c in other_stress)
    new_other = sum(new["synthetic_stress"][c]["correct_return"] for c in other_stress)
    checks = {
        "circle_accuracy_at_least_95pct": new_circle["return_accuracy"] >= .95,
        "circle_accuracy_not_worse": new_circle["correct_return"] >= old_circle["correct_return"],
        "circle_to_six_not_worse": new_circle["returned_16"] <= old_circle["returned_16"],
        "six_accuracy_not_worse": new_six["correct_return"] >= old_six["correct_return"],
        "six_to_circle_not_worse": new_six["returned_40"] <= old_six["returned_40"],
        "negative_false_circles_not_worse": new_empty["returned_40"] <= old_empty["returned_40"],
        "other_symbols_on_patterns_not_worse": new_other >= old_other,
        "all_false_circles_not_worse": sum(r["returned"] == "40" and r["expected"] != "40" for r in candidate["records"])
            <= sum(r["returned"] == "40" and r["expected"] != "40" for r in baseline["records"]),
        "noncircle_overall_drop_at_most_half_point": after >= before - .005,
        "no_class_loses_more_than_one_correct": all(n <= 1 for n in regressions.values()),
    }
    return {"checks": checks, "passed": all(checks.values()), "noncircle_before": before,
            "noncircle_after": after, "noncircle_regressions": regressions,
            "slices": new, "requires_independent_pi_validation": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir",type=Path,default=ROOT/"circle_data_v1")
    parser.add_argument("--weights",type=Path,default=ROOT/"runs/yolo26s_seg_mdp_lowres160/weights/best.pt")
    parser.add_argument("--name",default="yolo26s_seg_mdp_circle_v1")
    parser.add_argument("--epochs",type=int,default=20)
    parser.add_argument("--batch",type=int,default=32)
    parser.add_argument("--device",default="0")
    parser.add_argument("--evaluate-only",type=Path)
    args=parser.parse_args()
    cv2.setNumThreads(1)
    torch.set_num_threads(4)
    manifest=json.loads((args.data_dir/"manifest.json").read_text())
    if not manifest["complete"]:
        raise ValueError("Dataset generation incomplete")
    out=ROOT/"runs"/(args.name+"_review")
    out.mkdir(exist_ok=True)
    paths=[Path(s) for s in (args.data_dir/"selection_val.txt").read_text().splitlines()]
    if args.evaluate_only:
        report=evaluate(args.evaluate_only,paths,out/"external_evaluation.json",args.device)
        print(json.dumps(slice_metrics(report),indent=2))
        return
    baseline_file=out/"baseline.json"
    if baseline_file.exists():
        baseline=json.loads(baseline_file.read_text())
        if baseline["weights"]!=str(args.weights) or [r["path"] for r in baseline["records"]]!=[str(p) for p in paths]:
            raise ValueError("Baseline belongs to another checkpoint or dataset; use a new run name")
    else:
        baseline=evaluate(args.weights,paths,baseline_file,args.device)
    best_pair=[-1.0]

    def keep_pair_checkpoint(trainer):
        m=trainer.validator.metrics.box
        idx={int(c):i for i,c in enumerate(m.ap_class_index)}
        score=float(np.mean([m.ap50[idx[c]] for c in (5,29)]))
        with (out/"epoch_pair_metrics.jsonl").open("a") as f:
            f.write(json.dumps({"epoch":trainer.epoch+1,"pair_ap50":score,
                                "circle_recall":float(m.r[idx[29]]),"six_recall":float(m.r[idx[5]])})+"\n")
        if score>best_pair[0]:
            shutil.copy2(trainer.last,trainer.save_dir/"weights/best_pair.pt")
            best_pair[0]=score

    model=YOLO(str(args.weights))
    model.add_callback("on_model_save",keep_pair_checkpoint)
    model.train(data=str(args.data_dir/"data.yaml"),epochs=args.epochs,patience=6,
                batch=args.batch,imgsz=640,workers=2,device=args.device,
                project=str(ROOT/"runs"),name=args.name,exist_ok=False,resume=False,
                optimizer="AdamW",lr0=1e-4,lrf=.1,warmup_epochs=1,warmup_bias_lr=0,
                seed=0,deterministic=True,freeze=None,amp=True,
                fliplr=0,flipud=0,copy_paste=0,degrees=5,translate=.1,scale=.2,
                hsv_h=.015,hsv_s=.3,hsv_v=.3,mosaic=.2,close_mosaic=5,
                mixup=0,plots=True,save=True)
    run=Path(model.trainer.save_dir)
    del model
    torch.cuda.empty_cache()
    candidates={}
    for name in ("best","best_pair","last"):
        checkpoint=run/"weights"/(name+".pt")
        report=evaluate(checkpoint,paths,out/(name+".json"),args.device)
        candidates[name]=gates(baseline,report)
    eligible=[name for name,r in candidates.items() if r["passed"]]
    selected=max(eligible,key=lambda n:candidates[n]["slices"]["synthetic_stress"]["40"]["return_accuracy"]) if eligible else None
    report={"baseline":slice_metrics(baseline),"candidates":candidates,"selected":selected,
            "status":"candidate_for_pi_validation" if selected else "regression_gate_failed",
            "run":str(run),"deployed_model_changed":False}
    (out/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    if selected:
        shutil.copy2(run/"weights"/(selected+".pt"),run/"weights/candidate.pt")
    print("CIRCLE_TRAINING_COMPLETE",json.dumps({k:v for k,v in report.items() if k not in ("baseline","candidates")}),flush=True)


if __name__=="__main__":
    main()
