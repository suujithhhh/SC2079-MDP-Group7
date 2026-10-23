"""Write Task 2 reports from completed validation/export results; never rerun models."""
from pathlib import Path
import argparse
import csv
import json

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "reports"
CLASSES = ["bullseye", "left", "right"]
BANDS = ["<20", "20-32", "32-48", "48-80", "80+"]


def percent(value):
    return "n/a" if value is None else f"{100 * value:.2f}%"


def recall_text(group):
    return f"{group.get('correct', 0)}/{group.get('objects', 0)} ({percent(group.get('recall'))})"


def table(headers, rows):
    rows = [[str(x) for x in row] for row in rows]
    widths = [max(len(str(h)), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    line = lambda row: "  ".join(str(x).ljust(w) for x, w in zip(row, widths))
    return "\n".join([line(headers), line(["-" * w for w in widths]), *map(line, rows)])


def get_groups(result):
    return [("Old v3 640*", result["baseline_test"], "baseline_v3_640"),
            ("Task 2 640", result["test"]["640"], "640"),
            ("Task 2 512", result["test"]["512"], "512")]


def training_details():
    run = ROOT / "runs/yolo26n_task2_v1"
    details = {}
    if (run / "args.yaml").exists():
        import yaml
        details.update(yaml.safe_load((run / "args.yaml").read_text()))
    if (run / "results.csv").exists():
        with (run / "results.csv").open() as source:
            rows = list(csv.DictReader(source))
        if rows:
            last = {k.strip(): v.strip() for k, v in rows[-1].items()}
            details["completed_epochs"] = int(float(last["epoch"]))
    return details


def charts(result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    groups = get_groups(result)
    colors = ["#8c8c8c", "#1776b6", "#ed8b23"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True)
    x = np.arange(len(CLASSES))
    for i, (name, scores, _) in enumerate(groups):
        values = [100 * (scores.get(f"synthetic/{c}", {}).get("recall") or 0) for c in CLASSES]
        axes[0].bar(x + (i - 1) * .25, values, .25, label=name, color=colors[i])
    axes[0].set_xticks(x, ["Bullseye", "Left", "Right"])
    axes[0].set_ylim(0, 105)
    axes[0].set_ylabel("Correct-class recall (%)")
    axes[0].set_title("Held-out synthetic targets by class")
    axes[0].legend(fontsize=8, loc="lower right")
    for i, (name, scores, _) in enumerate(groups):
        values = [scores.get(f"synthetic/size_{b}", {}).get("recall") for b in BANDS]
        axes[1].plot(BANDS, [np.nan if v is None else 100 * v for v in values],
                     marker="o", linewidth=2, label=name, color=colors[i])
    axes[1].set_ylim(0, 105)
    axes[1].set_ylabel("Correct-class recall (%)")
    axes[1].set_xlabel("Maximum glyph side in source 640 x 640 frame (px)")
    axes[1].set_title("Synthetic targets by source pixel size")
    axes[1].grid(axis="y", alpha=.2)
    medians = [result["host_benchmarks"][key]["median_ms"] for _, _, key in groups]
    bars = axes[2].bar([name for name, _, _ in groups], medians, color=colors)
    axes[2].bar_label(bars, labels=[f"{v:.1f} ms" for v in medians], padding=4, fontsize=9)
    axes[2].set_ylim(0, max(medians) * 1.2)
    axes[2].set_ylabel("Median elapsed milliseconds per frame")
    axes[2].set_title("Host CPU / 4 threads — NOT Raspberry Pi")
    axes[2].tick_params(axis="x", labelsize=8)
    fig.suptitle(f"Task 2 NCNN test results | new conf {result['confidence']:g}, old conf {result.get('baseline_confidence', .35):g}", fontsize=15)
    fig.supxlabel("*Old v3 previously trained on some regrouped holdout sources: reference only. "
                  "Synthetic size tests do not measure physical detection distance.", fontsize=9)
    fig.savefig(REPORTS / "performance.png", dpi=180, facecolor="white")
    plt.close(fig)


def detailed_report(result, manifest, training):
    chosen = int(result["recommended_imgsz"])
    threshold = result["confidence"]
    selection = result["selected_checkpoint"]
    models = get_groups(result)
    lines = ["Task 2 YOLO26n detection: completed training and deployment evaluation", "",
             f"Recommended NCNN input: {chosen} x {chosen}; confidence: {threshold:g}.",
             f"Selected training checkpoint: {selection['selected']} (epoch filenames use zero-based indexing).",
             "Deployable PyTorch checkpoint: task2/weights/best.pt.",
             f"Pi package: task2/handoff/task2_yolo26n_{chosen}_ncnn_model.zip.", "",
             "Class mapping: 0 = bullseye (legacy marker), 1 = left (ID 39),",
             "2 = right (ID 38). The bullseye is the concentric-square marker;",
             "the solid circle / ID 40 is not a Task 2 target.", "",
             "Model and training", "",
             "Base: pretrained YOLO26n detection. Segmentation masks are unnecessary",
             "for these three classes. Training uses 640-pixel input; fixed 640 and",
             "512 NCNN exports are compared for detail versus speed. Export is FP32,",
             "batch 1, end2end=False, with the one-to-many head and NMS.",
             f"Training completed epochs: {training.get('completed_epochs', 'see training CSV')} "
             f"(configured maximum {training.get('epochs', 50)}).",
             f"Optimizer: {training.get('optimizer', 'AdamW')}; initial LR: {training.get('lr0', .001)}; "
             f"batch: {training.get('batch', 64)}; seed: {training.get('seed', 2602)}.",
             "Horizontal flips are disabled so left and right labels remain valid.", "",
             "Dataset", ""]
    rows = []
    for split in ["train", "val", "test"]:
        count = manifest["counts"][split]
        rows.append([split, count["images"], count["real_targets"], count["real_negatives"],
                     count["synthetic_images"]])
    lines += [table(["Split", "Images", "Real targets", "Real negatives", "Synthetic images"], rows), "",
              "Legacy arrow polygons often covered only the arrowhead. The source audit",
              "corrected 625 to full-glyph outlines and retained 26 already-full arrows.",
              "All 121 empty-label images and 6 related variants were excluded. Other",
              "annotated symbols provide negatives. Source images and labels were preserved.",
              "Synthetic arrows are black on white cards. Bullseye synthesis preserves",
              "the source marker rings. Synthetic samples include small targets, mild",
              "perspective, blur, brightness changes, and some multiple-target frames.",
              "Scene variants, same-timestamp frames, adjacent timestamp blocks, and exact",
              "image duplicates stay together. No source group crosses train/val/test.", "",
              "Validation-only model selection", "", selection["criterion"] + ".",
              f"Selected confidence: {threshold:g}. Test used for selection: "
              f"{selection.get('test_used_for_selection', False)}.",
              "Fixed NCNN input resolution was also chosen using validation results.", ""]
    rows = []
    for size in ["640", "512"]:
        scores = result["validation"][size]["validation"]["all"]
        rows.append([size, recall_text(scores), scores["wrong_direction"],
                     scores["wrong_class"], scores["false_positives"]])
    lines += [table(["Input", "Val correct/objects", "Opposite arrows", "Wrong classes", "Extra detections"], rows), "",
              "Held-out test: actual NCNN exports", "",
              "Correct-class recall requires IoU >= 0.30 with a ground-truth object.",
              "Matching is class-independent, so opposite arrows count explicitly as",
              "errors. IoU 0.30 allows comparison with old arrowhead-only boxes; it is",
              "not standard AP. AP for the new model is reported separately below.",
              f"New model confidence {threshold:g}; old v3 uses its original confidence {result.get('baseline_confidence', .35):g}.",
              "Unmatched predictions are extra detections (false positives). Opposite",
              "arrows are a subset of wrong classes, not additional errors to sum.", ""]
    for kind in ["all", "real", "synthetic"]:
        rows = []
        for name, scores, _ in models:
            s = scores[kind]
            rows.append([name, recall_text(s), s["missed"], s["wrong_class"],
                         s["wrong_direction"], s["false_positives"],
                         f"{s['negative_images_with_fp']}/{s['negative_images']}"])
        lines += [kind.upper(), table(["Model", "Correct/objects", "Missed", "Wrong class", "Opposite arrow",
                                      "Extra detections", "Negative frames with FP"], rows), ""]
    for kind in ["real", "synthetic"]:
        rows = [[c, *(recall_text(scores.get(f"{kind}/{c}", {})) for _, scores, _ in models)] for c in CLASSES]
        lines += [f"{kind.upper()} PER CLASS", table(["Class", *(name for name, _, _ in models)], rows), ""]
        rows = [[b, *(recall_text(scores.get(f"{kind}/size_{b}", {})) for _, scores, _ in models)] for b in BANDS]
        lines += [f"{kind.upper()} BY SOURCE SIZE", table(["Maximum side (px)", *(name for name, _, _ in models)], rows), ""]
    lines += ["Size bins use the full glyph's MAXIMUM side in the 640 x 640 source",
              "frame. At 512 network input, multiply these source sizes by 0.8.",
              "They are not measured physical distances. The turn helper's minimum",
              "side check instead uses the SHORTER side in actual network-input pixels.", "",
              "Standard box AP: PyTorch deployment head (end2end=False)", ""]
    rows = []
    for key, values in result["standard_ap"].items():
        rows.append([key, percent(values["box_map50"]), percent(values["box_map50_95"])])
    lines += [table(["Split_input", "AP50", "AP50-95"], rows), "",
              "Host CPU benchmark: these are NOT Raspberry Pi measurements", ""]
    rows = []
    for name, _, key in models:
        s = result["host_benchmarks"][key]
        rows.append([name, f"{s['median_ms']:.2f}", f"{s['p90_ms']:.2f}", s["threads"], s["frames"]])
    lines += [table(["Model", "Median ms", "P90 ms", "Threads", "Timed frames"], rows), ""]
    speed = result["host_benchmarks"][str(chosen)]
    lines += [f"Platform: {speed.get('platform', 'not recorded')}",
              f"Processor: {speed.get('processor') or 'not recorded'}",
              f"Includes: {speed['includes']}.", f"Excludes: {speed['excludes']}.", "",
              "Inference helper and remaining work", "",
              "Detector returns observations for all three classes. ArrowConfirmation",
              "requires an unambiguous arrow in a caller-selected obstacle ROI, a",
              "minimum 12-pixel shorter side in both raw frame and network input, and three consecutive",
              "associated observations. Too-small targets remain available as detector",
              "observations but cannot confirm a turn. Reset between obstacles and pass",
              "distinct live frame IDs. Bullseye observations never request a turn.",
              "The helper does not drive, navigate, return to the carpark, or stop the",
              "robot. These checks do not guarantee correct decisions or competition",
              "safety. Repeated-frame behavior still needs live integration and testing.",
              "Evidence capture preserves raw images separately from annotated images",
              "and creates contact_sheet.jpg. Save a representative frame per goal.", "",
              "Limits on interpretation", ""]
    lines += [f"- {limit}" for limit in result["limitations"]]
    lines += ["- Supplied overhead arena reference photo: zero accepted detections at confidence0.85.",
              "  Both arrows appear correctly as early candidates at about0.48 confidence, but",
              "  remain unconfirmed. This photo was not used for training or model selection.",
              "- No actual Task 2 Pi photographs or live Pi latency measurements were available.",
              "- Nearby source blocks can share rooms and capture sessions. Split separation",
              "  does not establish independence across camera deployments or environments.",
              "- Pasted synthetic cards are simplified scenes; white-background arrow",
              "  performance here does not replace real competition-card tests.",
              "- Old v3 is evaluated after filtering to marker/39/38, not using its old",
              "  application's largest-box selection policy. Its prior exposure to some",
              "  regrouped sources prevents a clean independent baseline comparison.",
              "- Raw model scores above precede the temporal confirmation and size gate.", "",
              "Reproduction and source guidance: task2/README.txt and",
              "task2/SETUP_AND_SOURCES.txt. Machine-readable results: reports/results.json."]
    return "\n".join(lines) + "\n"


def readme(result):
    chosen = int(result["recommended_imgsz"])
    conf = result["confidence"]
    s = result["test"][str(chosen)]
    latency = result["host_benchmarks"][str(chosen)]["median_ms"]
    return f"""Task 2: YOLO26n bullseye / left / right detector

Ready artifacts
  PyTorch checkpoint: weights/best.pt
  Recommended Pi ZIP: handoff/task2_yolo26n_{chosen}_ncnn_model.zip
  Fixed network input: {chosen} x {chosen}; confidence: {conf:g}
  Class IDs: 0=bullseye (legacy marker), 1=left (ID39), 2=right (ID38).
  Do not send the local class index as the robot image ID.
  For PyTorch predictions matching the NCNN head use:
    YOLO("weights/best.pt").predict(frame, imgsz=640, conf={conf:g},
                                   end2end=False, rect=False)

Measured held-out NCNN results at the validation-selected operating point
  Real targets: {recall_text(s['real'])}
  Synthetic targets: {recall_text(s['synthetic'])}
  Opposite-arrow errors: {s['all']['wrong_direction']}; wrong-class errors: {s['all']['wrong_class']}.
  Extra detections: {s['all']['false_positives']}; negative frames with detections:
  {s['all']['negative_images_with_fp']}/{s['all']['negative_images']}.
  Host CPU median: {latency:.2f} ms/frame, 4 threads. This is NOT Pi timing.
  These are raw model results, before temporal confirmation or the size gate.

The supplied overhead arena reference photo has two arrows. Neither reaches the
0.85 confirmation threshold; both appear correctly as early candidates at about
0.48 confidence. This remains a live-validation limitation, not a solved field test.
The photo was not used for training or model/threshold selection.

No actual Task 2 Pi photographs or live Pi benchmark were available. Synthetic
small-target results measure pixel-scale robustness, not detection distance in
meters. Nearby held-out blocks still share rooms/capture sessions. Old v3 had
seen some regrouped sources during earlier training, so its comparison is a
reference only. Full results: reports/REPORT.txt and reports/performance.png.

Try a frame on this training machine
  cd {ROOT}
  export PYTHONPATH="$(pwd)/../.circle_deps${{PYTHONPATH:+:$PYTHONPATH}}"
  export YOLO_CONFIG_DIR="$(pwd)/config"
  export YOLO_AUTOINSTALL=false
  export PYTHONDONTWRITEBYTECODE=1
  python scripts/inference.py /path/to/frame.jpg \\
    --model handoff/task2_yolo26n_{chosen}_ncnn_model --evidence-dir evidence

On the Pi
  Copy the recommended ZIP and extract it. Use the package requirements.txt
  with platform-compatible Torch, torchvision and OpenCV. From the extracted
  model folder run:
    python inference.py /path/to/frame.jpg --model . --evidence-dir evidence
  For early observations add --observation-conf 0.35. This returns candidates
  while confirmation remains at0.85. Candidate counts are not the accepted-ID
  rates above. Do not act on an unconfirmed candidate.
  The package reads its confidence and fixed input size from its metadata.
  Do not change imgsz without exporting a model for that size. A single image
  is an observation; it never confirms a turn.

Live integration
  Create Detector once. detector.predict(frame, observation_conf=0.35) can
  expose early candidates while detector.conf remains the confirmation threshold.
  Pass upright BGR frames and a live-selected obstacle
  ROI to ArrowConfirmation(conf=detector.conf,
  min_arrow_input_side=detector.min_arrow_input_side). It requires three consecutive
  associated arrow observations and at least 12 pixels on the shorter glyph
  side in both raw frame and network input. Too-small detections remain available as observations.
  Use original camera frames at least 640 pixels wide; do not pre-upscale low-resolution
  frames, which cannot recover missing directional detail. Pass distinct frame IDs
  and reset for each new goal. Bullseye never turns.
  This helper does not drive or stop the robot, and its gates do not guarantee
  correct competition behavior. Measure latency and validate on live Pi frames.
  save_evidence preserves raw inputs, separate bounding-box overlays, JSON,
  and a tiled contact_sheet.jpg for representative goal images.

Reproduce the work
  Use the environment exports above. Source data and shared dependencies are
  read from the parent mdp directory; new artifacts stay inside task2.
    python audit/build_source_manifest.py
    python scripts/build_dataset.py
    python scripts/train.py
    python scripts/export_validate.py
    python scripts/report_results.py
  Dataset construction and training intentionally refuse existing outputs.
  Preserve this run and use new explicit dataset/run paths for another run;
  update the matching paths in the scripts before rebuilding or retraining.
  export_validate.py selects checkpoints/confidence on validation, exports
  both fixed sizes, evaluates test data, benchmarks the host and builds ZIPs.
  report_results.py only reads completed results and writes reports.

Training uses YOLO26n detection at 640, with horizontal flips disabled to
preserve arrow direction. The audit fixes arrowhead-only source labels and
excludes empty-label images containing real symbols. See audit/SOURCE_AUDIT.txt
and SETUP_AND_SOURCES.txt for data provenance and official YOLO26/NCNN guidance.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=REPORTS / "results.json")
    args = parser.parse_args()
    if not args.results.exists():
        raise SystemExit(f"Results are not ready: {args.results}. Finish export_validate.py first.")
    result = json.loads(args.results.read_text())
    manifest = json.loads((ROOT / "data/manifest.json").read_text())
    training = training_details()
    REPORTS.mkdir(exist_ok=True)
    (ROOT / "README.txt").write_text(readme(result))
    (REPORTS / "REPORT.txt").write_text(detailed_report(result, manifest, training))
    charts(result)
    print("Wrote task2/README.txt, reports/REPORT.txt and reports/performance.png")


if __name__ == "__main__":
    main()
