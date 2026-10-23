"""Export and package the selected Task 2 arrow V2 detector; never calculate metrics.

Run with the existing .circle_deps environment. Export first, evaluate that artifact
independently, then package with --package-only --validation-report YOUR_REPORT.
All generated files stay below task2/v2; Task 2 V1 is read only. Packages use the
V2 inference runner, including its arrow-first handling of mixed-target scenes.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import shutil
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V1 = ROOT.parent
PACKAGE_NAME = "task2_arrow_v2_640_ncnn_model"
DEFAULT_CHECKPOINT = ROOT / "weights/best.pt"
DEFAULT_OPERATING_POINT = ROOT / "weights/operating_point.json"
DEFAULT_EXPORT = ROOT / "weights" / PACKAGE_NAME
MODEL_FILES = ("model.ncnn.param", "model.ncnn.bin", "metadata.yaml")
EXPECTED_NAMES = {0: "bullseye", 1: "left", 2: "right"}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_operating_point(path):
    point = json.loads(Path(path).read_text())
    confidence = point["confidence"]
    if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not 0 < confidence <= 1):
        raise ValueError("Operating confidence must be a finite number in (0, 1]")
    if point.get("imgsz", 640) != 640:
        raise ValueError("This package supports only a 640x640 export")
    if point.get("confirm_frames", 3) != 3:
        raise ValueError("The copied inference runner uses three-frame confirmation")
    size = point.get("min_arrow_input_side", 12.0)
    if (isinstance(size, bool) or not isinstance(size, (int, float))
            or not math.isfinite(size) or size < 0):
        raise ValueError("Minimum arrow side must be a finite nonnegative number")
    return point


def verify_export(folder):
    import yaml
    folder = Path(folder)
    for name in MODEL_FILES:
        if not (folder / name).is_file() or (folder / name).stat().st_size == 0:
            raise FileNotFoundError(folder / name)
    metadata = yaml.safe_load((folder / "metadata.yaml").read_text())
    names = metadata.get("names", {})
    names = dict(enumerate(names)) if isinstance(names, list) else names
    names = {int(key): value for key, value in names.items()}
    if names != EXPECTED_NAMES:
        raise ValueError(f"Unexpected model classes: {names}")
    shape = metadata.get("imgsz")
    shape = [shape, shape] if isinstance(shape, int) else shape
    if shape != [640, 640] or metadata.get("batch") != 1:
        raise ValueError("Expected static 640x640, batch 1 export")
    if metadata.get("task") != "detect" or metadata.get("end2end") is not False:
        raise ValueError("Expected detection export with end2end=False")
    return metadata


def export_model(checkpoint=DEFAULT_CHECKPOINT):
    """Export FP32, batch-one, 640 NCNN under V2; return its validated folder."""
    checkpoint = Path(checkpoint).resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "config"))
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    from ultralytics import YOLO, __version__
    if __version__ != "8.4.128":
        raise RuntimeError("Use ultralytics==8.4.128 to match the tested inference runner")
    staging = ROOT / "weights/task2_arrow_v2_640.pt"
    staging.parent.mkdir(parents=True, exist_ok=True)
    if checkpoint != staging:
        shutil.copy2(checkpoint, staging)
    exported = Path(YOLO(str(staging)).export(format="ncnn", imgsz=640, batch=1,
                                             end2end=False, quantize=32, device="cpu")).resolve()
    if not exported.is_relative_to(ROOT):
        raise RuntimeError(f"Exporter unexpectedly wrote outside V2: {exported}")
    verify_export(exported)
    (exported / "export_provenance.json").write_text(json.dumps({
        "checkpoint": str(checkpoint), "checkpoint_sha256": sha256(checkpoint),
        "imgsz": 640, "batch": 1, "end2end": False, "precision": "FP32 weights",
        "ultralytics": __version__,
    }, indent=2, sort_keys=True) + "\n")
    return exported


def environment_versions():
    packages = {}
    for name in ("ultralytics", "ncnn", "pnnx", "torch", "torchvision", "numpy", "Pillow", "PyYAML",
                 "opencv-python", "opencv-python-headless"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = "not installed in packaging interpreter"
    return {"python": platform.python_version(), "platform": platform.platform(),
            "packages": packages, "note": "Packaging host versions, not measured Pi compatibility or speed."}


def readme(point, has_validation):
    confidence = point["confidence"]
    size = point.get("min_arrow_input_side", 12.0)
    validation_note = ("The supplied evaluation report is copied unchanged to validation.json.\n"
                       if has_validation else "No evaluation report is included; accuracy has not been certified by this packaging step.\n")
    return f"""Task 2 arrow V2: YOLO26n detection, fixed 640x640 NCNN

Three local classes: 0 bullseye -> marker, 1 left -> 39, 2 right -> 38.
Legacy IDs are strings. Do not send local class indices as robot image IDs.
Use this detection runner; a former segmentation decoder is not interchangeable.
The V2 runner retains all target detections and prioritizes arrows over bullseyes
in mixed scenes. A bullseye must not replace a visible arrow's requested turn.

From this package folder, in the existing Pi Python environment:
  python inference.py IMAGE.jpg --model . --evidence-dir evidence
  python inference.py IMAGE.jpg --model . --observation-conf 0.35 --evidence-dir evidence
  python -m unittest test_inference.py

The operating_point.json default confirmation threshold is {confidence}.
Early observation confidence may expose weaker candidates; it never changes the
confirmation threshold. Candidate counts are not accepted-ID accuracy rates.

Construct Detector once per serial worker; predict accepts original upright BGR
camera frames. For each selected goal obstacle, pass its current image-pixel ROI
and distinct consecutive live frame IDs to:
  ArrowConfirmation(conf=detector.conf,
                    min_arrow_input_side=detector.min_arrow_input_side)
Reset on a new goal. Confirmation needs three associated observations and at
least {size:g} pixels across the smaller arrow dimension in BOTH the supplied raw
frame and network input. Use original camera frames at least 640 pixels wide;
external enlargement of low-resolution frames cannot restore missing detail.
Bullseye never produces a turn. A single-image result never confirms a turn.

This package observes images only. It does not drive, navigate, park, or stop the
robot. Save one representative frame for each recognized goal obstacle. Evidence
contains an unmodified raw input, a separate annotated image, JSON, and a tiled
contact sheet. The team still handles the competition evidence submission.

{validation_note}Model metrics, even perfect synthetic results, do not establish a physical
detection range or live driving reliability. Camera distance, angle, exposure,
motion blur, Pi latency, and temporal confirmation require a separate live test.
Host timing is not Pi timing. No evaluation, tuning, or robot action runs during
packaging. This remains a candidate for live Pi validation.

requirements.txt pins the model runtime used during development. Use the Pi's
platform-compatible torch/torchvision and OpenCV installation. versions.json
describes the packaging host; it is not an instruction to install its CUDA build
on the Pi. manifest.json contains SHA256 checksums for every packaged payload.
"""


def reproducible_zip(folder, archive):
    """Stable sorted paths, timestamps and permissions for identical file content."""
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as output:
        for path in sorted(folder.iterdir(), key=lambda item: item.name):
            if not path.is_file():
                continue
            entry = zipfile.ZipInfo(f"{folder.name}/{path.name}", date_time=(1980, 1, 1, 0, 0, 0))
            entry.create_system = 3
            entry.external_attr = 0o100644 << 16
            output.writestr(entry, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)
    with zipfile.ZipFile(archive) as check:
        if check.testzip() is not None:
            raise RuntimeError("ZIP integrity check failed")


def package_model(export_dir=DEFAULT_EXPORT, checkpoint=DEFAULT_CHECKPOINT,
                  operating_point=DEFAULT_OPERATING_POINT, validation_report=None, overwrite=False):
    """Copy an evaluated export and existing report; never run inference or metrics."""
    checkpoint, export_dir = Path(checkpoint).resolve(), Path(export_dir).resolve()
    operating_point = Path(operating_point).resolve()
    metadata, point = verify_export(export_dir), read_operating_point(operating_point)
    checkpoint_hash = sha256(checkpoint)
    provenance = export_dir / "export_provenance.json"
    if not provenance.is_file():
        raise FileNotFoundError("Missing export_provenance.json; export with this utility first")
    if json.loads(provenance.read_text())["checkpoint_sha256"] != checkpoint_hash:
        raise ValueError("Export provenance does not match the selected checkpoint")
    if validation_report is not None:
        validation_report = Path(validation_report).resolve()
        json.loads(validation_report.read_text())  # reject a truncated/non-JSON report
    handoff = ROOT / "handoff"
    handoff.mkdir(parents=True, exist_ok=True)
    package, archive = handoff / PACKAGE_NAME, handoff / f"{PACKAGE_NAME}.zip"
    if (package.exists() or archive.exists()) and not overwrite:
        raise FileExistsError("V2 handoff already exists; use --overwrite to rebuild this V2 package")
    with tempfile.TemporaryDirectory(prefix=".package_build_", dir=handoff) as temporary:
        staging = Path(temporary) / PACKAGE_NAME
        staging.mkdir()
        for name in MODEL_FILES:
            shutil.copy2(export_dir / name, staging / name)
        shutil.copy2(provenance, staging / provenance.name)
        for name in ("inference.py", "test_inference.py"):
            shutil.copy2(ROOT / "scripts" / name, staging / name)
        shutil.copy2(operating_point, staging / "operating_point.json")
        if validation_report:
            shutil.copy2(validation_report, staging / "validation.json")
        (staging / "README.txt").write_text(readme(point, validation_report is not None))
        (staging / "requirements.txt").write_text(
            "ultralytics==8.4.128\nncnn==1.0.20260526\nPillow\nPyYAML\n"
            "# Reuse platform-compatible torch, torchvision, numpy, and OpenCV.\n")
        (staging / "versions.json").write_text(json.dumps(environment_versions(), indent=2, sort_keys=True) + "\n")
        files = {path.name: sha256(path) for path in sorted(staging.iterdir()) if path.is_file()}
        manifest = {"source_checkpoint_sha256": checkpoint_hash, "imgsz": 640,
                    "confidence": point["confidence"], "classes": EXPECTED_NAMES,
                    "legacy_ids": {"bullseye": "marker", "left": "39", "right": "38"},
                    "candidate_only": True, "requires_live_pi_validation": True,
                    "metrics_recomputed_during_packaging": False,
                    "export_ultralytics_version": metadata.get("version"), "files": files}
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        for name, expected in files.items():
            if sha256(staging / name) != expected:
                raise RuntimeError(f"Payload checksum changed: {name}")
        staged_archive = Path(temporary) / archive.name
        reproducible_zip(staging, staged_archive)
        if package.exists():
            if package.is_symlink() or not package.is_dir():
                raise ValueError("Refusing to replace a non-directory or symlink handoff")
            shutil.rmtree(package)
        staging.replace(package)
        staged_archive.replace(archive)
    return {"package": str(package), "archive": str(archive), "archive_sha256": sha256(archive),
            "checkpoint_sha256": checkpoint_hash, "validation_included": validation_report is not None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--operating-point", type=Path, default=DEFAULT_OPERATING_POINT)
    parser.add_argument("--export-dir", type=Path, default=DEFAULT_EXPORT)
    parser.add_argument("--validation-report", type=Path)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--export-only", action="store_true")
    action.add_argument("--package-only", action="store_true")
    parser.add_argument("--overwrite", action="store_true", help="replace only this generated V2 handoff")
    args = parser.parse_args()
    folder = args.export_dir if args.package_only else export_model(args.checkpoint)
    if args.export_only:
        print(json.dumps({"export_dir": str(folder), "metrics_computed": False}, indent=2))
        return
    print(json.dumps(package_model(folder, args.checkpoint, args.operating_point,
                                   args.validation_report, args.overwrite), indent=2))


if __name__ == "__main__":
    main()
