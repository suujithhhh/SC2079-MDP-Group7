"""Arrow voting, camera ranging and the camera worker for Task 2.

Uses the Task 2 model (class names "left", "right" and "bullseye";
config.ARROW_IDS, config.BULLSEYE_LABEL).

Ranging is the pinhole model: depth along the camera axis = K / box height,
lateral offset = depth * x_norm * tan(HFOV / 2). K must be calibrated per
symbol (the calibration tool went with the old task2.py).

Heavy imports (cv2, picamera2, ultralytics) are deferred so the logic and the
tests run without them.
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

import config


@dataclass(frozen=True)
class Detection:
    label: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return (self.x2 - self.x1) * (self.y2 - self.y1)

    def norm_x(self, frame_width: int = config.FRAME_WIDTH) -> float:
        """Box centre, -1 (left edge) .. +1 (right edge)."""
        return (self.x1 + self.x2) / frame_width - 1


@dataclass
class Look:
    """One stationary capture: detections for each frame, plus where the
    raw and annotated images were saved."""

    tag: str
    frames: list[list[Detection]]
    raw_path: Path | None = None
    annotated_path: Path | None = None
    seconds: float = 0.0
    extra: dict = field(default_factory=dict)


MIRRORED_LABEL = {"left": "right", "right": "left"}


def unmirror(detections: list[Detection], frame_width: int = config.FRAME_WIDTH) -> list[Detection]:
    """Detections made on a left-right mirrored frame, mapped back to the real frame: x flipped, and an arrow's
    direction swapped (a right arrow mirrored points left)."""
    return [Detection(MIRRORED_LABEL.get(d.label, d.label), d.conf, frame_width - d.x2, d.y1, frame_width - d.x1, d.y2)
            for d in detections]


def has_arrow(detections: list[Detection]) -> bool:
    return any(kind_of(d.label) == "arrow" and d.conf >= config.MIN_CONFIDENCE for d in detections)


def kind_of(label: str) -> str | None:
    if label in config.ARROW_IDS:
        return "arrow"
    if label == config.BULLSEYE_LABEL:
        return "marker"
    return None


def largest(detections: list[Detection], kind: str) -> Detection | None:
    candidates = [d for d in detections if kind_of(d.label) == kind and d.conf >= config.MIN_CONFIDENCE
                  and d.height > 0]
    return max(candidates, key=lambda d: (d.area, d.conf), default=None)


def decide_arrow(frames: list[list[Detection]]) -> tuple[str | None, dict[str, int]]:
    """Each frame votes with its largest arrow. Accepted only when one
    direction has at least VOTE_MIN votes and the other has none. Task 2
    passes a single frame, so its largest confident arrow decides."""
    votes = {"LEFT": 0, "RIGHT": 0}
    for detections in frames:
        best = largest(detections, "arrow")
        if best is not None:
            votes[config.ARROW_IDS[best.label]] += 1
    for side, other in (("LEFT", "RIGHT"), ("RIGHT", "LEFT")):
        if votes[side] >= config.VOTE_MIN and votes[other] == 0:
            return side, votes
    return None, votes


def depth_cm(box_height_px: float, kind: str) -> float:
    return config.RANGE_K[kind] / box_height_px


def lateral_cm(norm_x: float, depth: float) -> float:
    """cm to the right of the camera axis at `depth`."""
    return depth * norm_x * math.tan(math.radians(config.HFOV_DEG) / 2)


def range_target(frames: list[list[Detection]], kind: str) -> tuple[float, float] | None:
    """(camera depth, lateral offset) of the largest `kind` box, median over
    the frames that saw one; None if no frame did."""
    boxes = [b for b in (largest(f, kind) for f in frames) if b is not None]
    if not boxes:
        return None
    depth = statistics.median(depth_cm(b.height, kind) for b in boxes)
    norm = statistics.median(b.norm_x() for b in boxes)
    return depth, lateral_cm(norm, depth)


def calibration_k(frames: list[list[Detection]], camera_depth_cm: float) -> dict[str, float]:
    """K per kind from boxes seen at a known camera-to-face depth."""
    result = {}
    for kind in ("arrow", "marker"):
        heights = [b.height for b in (largest(f, kind) for f in frames) if b is not None]
        if heights:
            result[kind] = statistics.median(heights) * camera_depth_cm
    return result


# --- camera + model ----------------------------------------------------------------
def _worker(model_dir: str, imgsz: int, conf: float, jobs, results) -> None:
    """Model in its own process (NCNN holds the GIL). Job:
    (tag, frames, raw_path, annotated_path) -> (tag, [[Detection...] per frame])."""
    import signal

    import cv2
    import numpy as np
    from ultralytics import YOLO

    signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C is the main process's emergency STOP, not ours
    model = YOLO(model_dir, task="detect")   # the Task 2 model is a detector (metadata.yaml task: detect)
    # The same arguments as the real calls below: the robot run 2026-10-15 printed "Loading ... for NCNN inference" a
    # second time at the first real look (after START), which suggests the model was set up again there.
    model.predict(source=np.zeros((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8),
                  imgsz=imgsz, conf=conf, device="cpu", verbose=False)
    results.put(("ready", None))
    while True:
        job = jobs.get()
        if job is None:
            return
        tag, frames, raw_path, annotated_path = job
        def detect(image):
            result = model.predict(source=image, imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]
            found = []
            if result.boxes is not None:
                for cls, c, box in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist(),
                                       result.boxes.xyxy.tolist()):
                    found.append(Detection(result.names[int(cls)], float(c), *map(float, box)))
            return result, found

        per_frame, best = [], None   # best = (score, frame, annotated image)
        for frame in frames:
            result, detections = detect(frame)
            annotated = result.plot()
            if not has_arrow(detections):
                # Robot runs 2026-10-14: right arrows the model missed outright (even at 0.05) on the plain frame were
                # read at 0.91-0.95 on the mirrored frame. Same frame, mirrored, its result mapped back.
                _, mirrored = detect(cv2.flip(frame, 1))
                mirrored = unmirror(mirrored, frame.shape[1])
                if has_arrow(mirrored):
                    detections = mirrored
                    annotated = frame.copy()
                    for d in detections:
                        p1, p2 = (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2))
                        cv2.rectangle(annotated, p1, p2, (0, 255, 0), 3)
                        cv2.putText(annotated, f"{d.label} {d.conf:.2f} (mirror)",
                                    (max(0, min(p1[0], annotated.shape[1] - 270)), max(p1[1] - 8, 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            per_frame.append(detections)
            score = max((d.conf for d in detections if kind_of(d.label) == "arrow"), default=0.0)
            if best is None or score > best[0]:
                best = (score, frame, annotated)
        _, frame, annotated = best
        cv2.imwrite(raw_path, frame)
        cv2.imwrite(annotated_path, annotated)
        results.put((tag, per_frame))


class Camera:
    """Pi camera in this process, the NCNN model in a worker process."""

    def __init__(self, model_dir: Path, imgsz: int = 640, conf: float = 0.25,
                 settle_s: float = 0.15, ready_timeout_s: float = 120.0) -> None:
        import multiprocessing
        import queue

        from picamera2 import Picamera2

        if not (model_dir / "model.ncnn.param").is_file():
            raise FileNotFoundError(f"NCNN model not found in {model_dir}")
        self.settle_s = settle_s
        self.out_dir: Path | None = None
        self.looks: list[Look] = []
        context = multiprocessing.get_context("spawn")
        self._jobs, self._done = context.Queue(), context.Queue()
        self._worker = context.Process(target=_worker, name="task2-vision", daemon=True,
                                       args=(str(model_dir), imgsz, conf, self._jobs, self._done))
        self._worker.start()
        self._camera = Picamera2()
        camera_config = self._camera.create_preview_configuration(
            main={"format": "RGB888", "size": (config.FRAME_WIDTH, config.FRAME_HEIGHT)},
            buffer_count=2, queue=False)
        self._camera.align_configuration(camera_config)
        self._camera.configure(camera_config)
        self._camera.start()
        try:
            reply = self._done.get(timeout=ready_timeout_s)
        except queue.Empty:
            reply = None
        if reply != ("ready", None):
            self.close()
            raise RuntimeError("vision worker did not start (see its error above)")

    def close(self) -> None:
        if self._worker.is_alive():
            self._jobs.put(None)
            self._worker.join(timeout=5)
        self._camera.stop()
        self._camera.close()

    def look(self, tag: str, frames: int = 1) -> Look:
        """Settle, capture `frames` frames, run the model on all of them (waits)."""
        started = time.monotonic()
        time.sleep(self.settle_s)
        images = [self._camera.capture_array() for _ in range(frames)]
        raw = self.out_dir / f"{tag}_raw.jpg"
        annotated = self.out_dir / f"{tag}_boxes.jpg"
        self._jobs.put((tag, images, str(raw), str(annotated)))
        while True:
            item = self._done.get(timeout=60)
            if item[0] == tag:
                break
        look = Look(tag, item[1], raw, annotated, time.monotonic() - started)
        self.looks.append(look)
        return look

    def stitch(self, looks: list[Look]) -> Path | None:
        """Tile the annotated images of the given looks into one picture."""
        import cv2
        import numpy as np

        tiles = [cv2.imread(str(look.annotated_path)) for look in looks
                 if look.annotated_path and look.annotated_path.is_file()]
        if not tiles:
            return None
        height, width = tiles[0].shape[:2]
        canvas = np.zeros((height, width * len(tiles), 3), dtype=np.uint8)
        for i, tile in enumerate(tiles):
            canvas[:, i * width:(i + 1) * width] = cv2.resize(tile, (width, height))
        path = self.out_dir / "stitched.jpg"
        cv2.imwrite(str(path), canvas)
        return path
