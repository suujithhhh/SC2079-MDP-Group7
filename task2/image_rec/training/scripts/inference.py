"""Task 2 NCNN detection and evidence, with no robot-control side effects.

Arrays and ROI coordinates use the original upright BGR camera frame. Create one
Detector per worker and call it serially. The caller selects the current obstacle
ROI, supplies consecutive live frames, and resets ArrowConfirmation on a new
obstacle. Use ArrowConfirmation(conf=detector.conf) to match the packaged operating
point, and min_arrow_input_side=detector.min_arrow_input_side for its size gate.
Detector returns all observations, including tiny targets; only turn confirmation
requires sufficient arrow pixels in both the supplied raw frame and network input.
Use original camera frames at least 640 pixels wide; do not pre-upscale low-resolution
frames, since their lost detail cannot be measured after external upscaling.
A single-image result never confirms a turn.

Optional observation_conf exposes earlier low-confidence candidates without
lowering the packaged confirmation threshold. Candidate counts are not validated
accepted-ID rates; callers must use ArrowConfirmation with detector.conf.
"""
import argparse
import json
import math
import shutil
import uuid
from datetime import datetime, timezone
from pathlib import Path

NAMES = {0: "bullseye", 1: "left", 2: "right"}
LEGACY_IDS = {0: "marker", 1: "39", 2: "38"}


def operating_confidence(model_folder, override=None):
    """Read a validation-selected package threshold; caller override takes priority."""
    path = Path(model_folder) / "operating_point.json"
    value = override
    if value is None:
        value = json.loads(path.read_text())["confidence"] if path.exists() else 0.45
    return validate_confidence(value)


def validate_confidence(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not 0 < value <= 1:
        raise ValueError("confidence must be a number in (0, 1]")
    return float(value)


def operating_min_arrow_side(model_folder):
    """Minimum arrow width/height in both supplied raw frame and network input."""
    path = Path(model_folder) / "operating_point.json"
    config = json.loads(path.read_text()) if path.exists() else {}
    value = config.get("min_arrow_input_side", 12.0)
    validate_min_arrow_side(value)
    return float(value)


def validate_min_arrow_side(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError("min_arrow_input_side must be a finite nonnegative number")


def iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]))
    area = lambda x: max(0, x[2] - x[0]) * max(0, x[3] - x[1])
    union = area(a) + area(b) - intersection
    return intersection / union if union > 0 else 0.0


def validate_roi(roi):
    if len(roi) != 4 or not all(math.isfinite(v) for v in roi):
        raise ValueError("ROI must contain four finite pixel coordinates")
    if roi[2] <= roi[0] or roi[3] <= roi[1]:
        raise ValueError("ROI must be x0 y0 x1 y1 with positive width and height")


def select_arrow(detections, roi, conf=0.45, min_arrow_input_side=12.0):
    """Select only an unambiguous target centered inside the caller's pixel ROI.

    Returns an observation, not permission to turn. Multiple targets in the ROI
    abstain, including two arrows with the same direction. Bullseye never turns.
    Tiny arrows remain detections but abstain from a turn decision. A missing
    input_min_side also abstains unless the caller explicitly sets the gate to 0.
    When source_min_side is provided, both source and input must clear the gate.
    """
    validate_roi(roi)
    validate_min_arrow_side(min_arrow_input_side)
    candidates = []
    for detection in detections:
        if detection["confidence"] < conf:
            continue
        x0, y0, x1, y1 = detection["bbox"]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if roi[0] <= cx <= roi[2] and roi[1] <= cy <= roi[3]:
            candidates.append(detection)
    answer = {"direction": None, "reason": "missing", "detection": None}
    if len(candidates) > 1:
        answer["reason"] = "ambiguous_targets"
    elif candidates:
        target = candidates[0]
        answer["detection"] = target
        name = NAMES.get(target["class_id"])
        if name in ("left", "right"):
            size = target.get("input_min_side")
            known = (not isinstance(size, bool) and isinstance(size, (int, float))
                     and math.isfinite(size) and size >= 0)
            if "source_min_side" in target:
                source_size = target["source_min_side"]
                source_known = (not isinstance(source_size, bool)
                                and isinstance(source_size, (int, float))
                                and math.isfinite(source_size) and source_size > 0)
                known = known and source_known
                if known:
                    size = min(size, source_size)
            if min_arrow_input_side > 0 and not known:
                answer["reason"] = "target_size_unknown"
            elif min_arrow_input_side > 0 and size < min_arrow_input_side:
                answer["reason"] = "arrow_too_small"
            else:
                answer.update(direction=name, reason="arrow_observed")
        else:
            answer["reason"] = "bullseye" if name == "bullseye" else "unknown_class"
    return answer


def same_target(a, b):
    """Associate overlapping boxes or modest center/scale motion between frames."""
    if iou(a, b) >= 0.2:
        return True
    aw, ah, bw, bh = a[2] - a[0], a[3] - a[1], b[2] - b[0], b[3] - b[1]
    if min(aw, ah, bw, bh) <= 0:
        return False
    ratio = max(aw / bw, bw / aw, ah / bh, bh / ah)
    distance = math.hypot((a[0] + a[2] - b[0] - b[2]) / 2,
                          (a[1] + a[3] - b[1] - b[3]) / 2)
    return ratio <= 2 and distance <= 0.5 * min(max(aw, ah), max(bw, bh))


class ArrowConfirmation:
    """Require three consecutive, associated arrow observations before confirming.

    Reset explicitly for each new goal obstacle. Missing, ambiguous, bullseye and
    conflicting observations clear the streak. Optional frame_id rejects repeated
    frames. Confirmation is evidence only: navigation belongs to the caller.
    """
    def __init__(self, frames=3, conf=0.45, min_arrow_input_side=12.0):
        if frames < 3:
            raise ValueError("At least three frames are required")
        self.frames, self.conf = frames, conf
        validate_min_arrow_side(min_arrow_input_side)
        self.min_arrow_input_side = min_arrow_input_side
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0
        self.last_frame_id = None

    def update(self, detections, roi, frame_id=None):
        observation = select_arrow(detections, roi, self.conf, self.min_arrow_input_side)
        observation.update(confirmed=False, streak=0)
        if frame_id is not None and frame_id == self.last_frame_id:
            observation.update(direction=None, reason="duplicate_frame", streak=self.count)
            return observation
        self.last_frame_id = frame_id
        target = observation["detection"]
        if observation["direction"] is None:
            self.previous, self.count = None, 0
            return observation
        if self.previous and self.previous["class_id"] != target["class_id"]:
            self.previous, self.count = None, 0
            observation.update(direction=None, reason="conflicting_direction")
            return observation
        linked = self.previous and same_target(self.previous["bbox"], target["bbox"])
        self.count = min(self.frames, self.count + 1) if linked else 1
        self.previous = target
        confirmed = self.count >= self.frames
        observation.update(confirmed=confirmed, streak=self.count,
                           reason="confirmed" if confirmed else "pending_confirmation")
        if not confirmed:
            observation["direction"] = None
        return observation


def load_bgr(image):
    """Paths honor EXIF orientation; ndarray callers must supply upright BGR."""
    import cv2
    import numpy as np
    from PIL import Image, ImageOps
    if isinstance(image, np.ndarray):
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("Camera frame must be H x W x 3 uint8 BGR")
        return image
    with Image.open(image) as source:
        return cv2.cvtColor(np.array(ImageOps.exif_transpose(source).convert("RGB")),
                            cv2.COLOR_RGB2BGR)


class Detector:
    """Portable Ultralytics 8.4.128 NCNN adapter. No torch import until construction."""
    def __init__(self, model, imgsz=None, conf=None, threads=4):
        import yaml
        folder = Path(model).resolve()
        conf = operating_confidence(folder, conf)
        self.min_arrow_input_side = operating_min_arrow_side(folder)
        metadata = yaml.safe_load((folder / "metadata.yaml").read_text())
        shape = metadata["imgsz"]
        shape = [shape, shape] if isinstance(shape, int) else shape
        if len(shape) != 2 or shape[0] != shape[1]:
            raise ValueError("This runner expects a square NCNN export")
        self.imgsz = int(shape[0])
        if imgsz is not None and imgsz != self.imgsz:
            raise ValueError(f"Export requires imgsz={self.imgsz}; re-export to change it")
        names = metadata["names"]
        names = dict(enumerate(names)) if isinstance(names, list) else names
        names = {int(k): v for k, v in names.items()}
        if names != NAMES or metadata.get("task") != "detect" or metadata.get("end2end"):
            raise ValueError("Expected Task2 detect classes bullseye/left/right and end2end=False")
        if not 0 < conf <= 1 or threads < 1:
            raise ValueError("conf must be in (0, 1] and threads must be positive")
        import ncnn
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(threads)
        self.model = YOLO(str(folder), task="detect")
        self.conf, self.threads, self.model_dir = conf, threads, folder
        # Initialize the framework backend, then reload its network with the thread
        # option set BEFORE weights: NCNN kernels retain their load-time value.
        # This avoids a process-global ncnn.Net monkeypatch during construction.
        import numpy as np
        self.model.predict(np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
                           imgsz=self.imgsz, rect=False, end2end=False, verbose=False,
                           device="cpu", max_det=20)
        backend = self.model.predictor.model.backend
        if isinstance(getattr(backend, "net", None), ncnn.Net):
            network = ncnn.Net()
            network.opt.num_threads = threads
            network.opt.use_vulkan_compute = False
            if network.load_param(str(folder / "model.ncnn.param")) != 0:
                raise RuntimeError("Could not load NCNN network parameters")
            if network.load_model(str(folder / "model.ncnn.bin")) != 0:
                raise RuntimeError("Could not load NCNN model weights")
            backend.net.clear()
            backend.net = network
        self.loaded_net_threads = int(backend.net.opt.num_threads)

    def observation_threshold(self, override=None):
        """Observation sensitivity is separate from self.conf for turn confirmation."""
        return self.conf if override is None else validate_confidence(override)

    def predict(self, image, observation_conf=None):
        """Return all targets, including tiny arrows; bboxes use original-frame pixels.

        input_min_side records the glyph box's smaller dimension after square
        letterboxing to this export's fixed network input size.
        source_min_side records it in the supplied unscaled camera frame, so an
        internally upscaled low-resolution arrow cannot pass the decision gate.
        observation_conf may expose early candidates, but never changes self.conf
        or a caller's required ArrowConfirmation(conf=self.conf) threshold.
        """
        threshold = self.observation_threshold(observation_conf)
        frame = load_bgr(image)
        input_scale = self.imgsz / max(frame.shape[:2])
        result = self.model.predict(frame, imgsz=self.imgsz, rect=False,
                                    end2end=False, conf=threshold, iou=0.5,
                                    agnostic_nms=False, max_det=20,
                                    device="cpu", verbose=False)[0]
        targets = []
        for x0, y0, x1, y1, confidence, class_id in result.boxes.data.cpu().tolist():
            class_id = int(class_id)
            targets.append({"class_id": class_id, "name": NAMES[class_id],
                            "image_id": LEGACY_IDS[class_id], "confidence": confidence,
                            "meets_confirmation_confidence": confidence >= self.conf,
                            "bbox": [x0, y0, x1, y1],
                            "source_min_side": min(x1 - x0, y1 - y0),
                            "input_min_side": min(x1 - x0, y1 - y0) * input_scale})
        return sorted(targets, key=lambda d: -d["confidence"])


def save_evidence(image, detections, decision, directory, context=None):
    """Preserve raw input plus separate overlay/JSON and rebuild a tiled contact sheet.

    Call once per chosen representative frame/goal. File inputs are copied byte
    for byte; camera arrays are saved losslessly as PNG. Nothing is drawn on raw.
    """
    import cv2
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    event = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%f") + "_" + uuid.uuid4().hex[:8]
    output = directory / event
    output.mkdir()
    frame = load_bgr(image)
    if isinstance(image, (str, Path)):
        raw = output / ("raw" + Path(image).suffix.lower())
        shutil.copyfile(image, raw)
    else:
        raw = output / "raw.png"
        if not cv2.imwrite(str(raw), frame):
            raise OSError(f"Cannot write {raw}")
    annotated = frame.copy()
    for detection in detections:
        x0, y0, x1, y1 = [round(v) for v in detection["bbox"]]
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 200, 255), 2)
        label = f'{detection["name"]} {detection["confidence"]:.2f}'
        cv2.putText(annotated, label, (x0, max(15, y0 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(annotated, label, (x0, max(15, y0 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
    overlay = output / "annotated.jpg"
    if not cv2.imwrite(str(overlay), annotated):
        raise OSError(f"Cannot write {overlay}")
    record = {"event_id": event, "raw_file": raw.name, "annotated_file": overlay.name,
              "detections": detections, "decision": decision, "context": context or {}}
    (output / "metadata.json").write_text(json.dumps(record, indent=2) + "\n")
    make_contact_sheet(directory)
    return str(output)


def make_contact_sheet(directory):
    import cv2
    import numpy as np
    records = sorted(Path(directory).glob("*/metadata.json"))
    columns, width, height = 3, 480, 390
    sheet = np.full((math.ceil(len(records) / columns) * height, columns * width, 3), 255, np.uint8)
    for index, path in enumerate(records):
        record = json.loads(path.read_text())
        frame = cv2.imread(str(path.parent / record["annotated_file"]))
        scale = min(width / frame.shape[1], (height - 45) / frame.shape[0])
        thumb = cv2.resize(frame, (max(1, round(frame.shape[1] * scale)),
                                   max(1, round(frame.shape[0] * scale))))
        x, y = index % columns * width, index // columns * height
        sheet[y:y + thumb.shape[0], x:x + thumb.shape[1]] = thumb
        decision = record["decision"]
        label = f'{index + 1}: {decision.get("reason", "observation")}'
        cv2.putText(sheet, label, (x + 8, y + height - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 1)
    if records and not cv2.imwrite(str(Path(directory) / "contact_sheet.jpg"), sheet):
        raise OSError("Cannot write contact sheet")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, help="must match model metadata; default reads metadata")
    parser.add_argument("--conf", type=float, default=None,
                        help="override packaged operating_point.json threshold (fallback 0.45)")
    parser.add_argument("--observation-conf", type=float, default=None,
                        help="show earlier candidates, e.g. 0.35, without lowering confirmation confidence")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--roi", type=float, nargs=4, metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("--evidence-dir", type=Path)
    args = parser.parse_args()
    detector = Detector(args.model, args.imgsz, args.conf, args.threads)
    observation_confidence = detector.observation_threshold(args.observation_conf)
    detections = detector.predict(args.image, observation_conf=args.observation_conf)
    decision = {"direction": None, "confirmed": False, "reason": "target_roi_required"}
    if args.roi:
        decision = ArrowConfirmation(conf=detector.conf,
                                     min_arrow_input_side=detector.min_arrow_input_side).update(
                                         detections, args.roi)
    output = {"detections": detections, "decision": decision, "imgsz": detector.imgsz,
              "confidence": detector.conf, "confirmation_confidence": detector.conf,
              "observation_confidence": observation_confidence,
              "min_arrow_input_side": detector.min_arrow_input_side}
    if args.evidence_dir:
        output["evidence"] = save_evidence(args.image, detections, decision, args.evidence_dir,
                                           {"model": str(args.model), "conf": detector.conf,
                                            "confirmation_confidence": detector.conf,
                                            "observation_confidence": observation_confidence,
                                            "imgsz": detector.imgsz, "roi": args.roi,
                                            "min_arrow_input_side": detector.min_arrow_input_side})
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
