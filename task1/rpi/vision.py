"""Camera capture, image recognition and the stitched result image (Task 1).

Runs the team's Ultralytics NCNN model on the Pi camera. The model's class
names are the image IDs ("11".."40") plus "marker" (the bullseye), which is
never reported as a target. Heavy imports (cv2, picamera2, ultralytics) are
deferred so the rest of the program and its tests run without them.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

BULLSEYE_LABEL = "marker"


@dataclass
class Recognition:
    obstacle_id: int
    image_id: str | None       # None = nothing but bullseyes / nothing at all
    confidence: float
    raw_path: Path
    annotated_path: Path
    upload: str | None = None  # result of sending the photo to the laptop (None = not sent)


def image_upload_url(images_url: str, mission_id: str, obstacle_id: int, image_id: str | None) -> str:
    """`POST /images` URL (PROTOCOL.md §7.1). `target_id` only for a real image
    ID (11-40); anything else is sent without one ("nothing recognised")."""
    from urllib.parse import urlencode

    query = {"mission_id": mission_id, "obstacle_id": obstacle_id}
    if image_id is not None and image_id.isdigit() and 11 <= int(image_id) <= 40:
        query["target_id"] = int(image_id)
    return f"{images_url}?{urlencode(query)}"


def upload_image(url: str, jpeg: bytes, timeout_s: float = 5.0) -> str:
    """Send one JPEG to the laptop; never raises. Returns a short status line."""
    import json
    import urllib.error
    import urllib.request

    request = urllib.request.Request(url, data=jpeg, method="POST", headers={"Content-Type": "image/jpeg"})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            reply = json.loads(response.read() or b"{}")
        return f"uploaded {reply.get('image_id', '?')} ({len(jpeg) // 1024} KB)"
    except urllib.error.HTTPError as exc:
        return f"upload rejected: HTTP {exc.code} {exc.read()[:120]!r}"
    except (OSError, ValueError) as exc:
        return f"upload failed: {exc}"


def pick_target(names: dict[int, str], classes: list[int], confs: list[float],
                boxes_xyxy: list[list[float]]) -> tuple[str, float] | None:
    """Choose the image to report: the largest non-bullseye box, ties broken by
    confidence (same rule as realtime_ncnn.py, minus the bullseye)."""
    best_key, best = None, None
    for cls, conf, (x1, y1, x2, y2) in zip(classes, confs, boxes_xyxy):
        label = names[int(cls)]
        if label == BULLSEYE_LABEL or x2 <= x1 or y2 <= y1:
            continue
        key = ((x2 - x1) * (y2 - y1), conf)
        if best_key is None or key > best_key:
            best_key, best = key, (label, float(conf))
    return best


def target_centre_x(confs: list[float], boxes_xyxy: list[list[float]], frame_width: int) -> float | None:
    """Horizontal centre of the largest box, bullseye included, as -1 (left
    edge) .. +1 (right edge); None if nothing was detected. Used for alignment."""
    best_key, best = None, None
    for conf, (x1, y1, x2, y2) in zip(confs, boxes_xyxy):
        if x2 <= x1 or y2 <= y1:
            continue
        key = ((x2 - x1) * (y2 - y1), conf)
        if best_key is None or key > best_key:
            best_key, best = key, (x1 + x2) / 2
    return None if best is None else best / (frame_width / 2) - 1


def _annotate(cv2, result, obstacle_id: int, label: str | None, confidence: float):
    annotated = result.plot()
    caption = f"Obstacle {obstacle_id}: " + (f"ID {label} ({confidence:.2f})" if label else "not recognised")
    cv2.putText(annotated, caption, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 0) if label else (0, 0, 255), 2, cv2.LINE_AA)
    return annotated


def _inference_worker(model_dir: str, imgsz: int, conf: float, jobs, results,
                      images_url: str | None = None) -> None:
    """Runs in its own process, so the NCNN model (which holds the GIL while it
    computes) never stalls the main process's STM/Android loop.

    Jobs: ("snap", obstacle_id, frames, raw_path, annotated_path) -> ("snap", Recognition);
          with `images_url` set, the annotated photo is also sent to the laptop
          (`POST /images`, mission id = the run folder's name).
          ("centre", tag, frame, image_path) -> ("centre", tag, norm_x or None)
          None -> exit.
    """
    import cv2
    import numpy as np
    from ultralytics import YOLO

    model = YOLO(model_dir, task="segment")
    model.predict(source=np.zeros((480, 640, 3), dtype=np.uint8), imgsz=imgsz, verbose=False)  # warm-up
    results.put(("ready",))

    def predict(frame):
        return model.predict(source=frame, imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]

    while True:
        job = jobs.get()
        if job is None:
            return
        if job[0] == "centre":
            _, tag, frame, image_path = job
            result = predict(frame)
            cv2.imwrite(image_path, result.plot())
            boxes = result.boxes
            norm_x = (None if boxes is None or len(boxes) == 0
                      else target_centre_x(boxes.conf.tolist(), boxes.xyxy.tolist(), frame.shape[1]))
            results.put(("centre", tag, norm_x))
            continue
        _, obstacle_id, frames, raw_path, annotated_path = job
        best, fallback = None, None   # (confidence, label, frame, result)
        for frame in frames:
            result = predict(frame)
            fallback = fallback or (frame, result)
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue
            choice = pick_target(result.names, boxes.cls.tolist(), boxes.conf.tolist(), boxes.xyxy.tolist())
            if choice and (best is None or choice[1] > best[0]):
                best = (choice[1], choice[0], frame, result)
        if best is not None:
            confidence, label, frame, result = best
        else:
            (confidence, label), (frame, result) = (0.0, None), fallback
        cv2.imwrite(raw_path, frame)
        ok, jpeg = cv2.imencode(".jpg", _annotate(cv2, result, obstacle_id, label, confidence),
                                [cv2.IMWRITE_JPEG_QUALITY, 85])
        jpeg = jpeg.tobytes()
        Path(annotated_path).write_bytes(jpeg)
        upload = None
        if images_url:
            mission_id = Path(annotated_path).parent.name
            upload = upload_image(image_upload_url(images_url, mission_id, obstacle_id, label), jpeg)
        results.put(("snap", Recognition(obstacle_id, label, confidence, Path(raw_path), Path(annotated_path),
                                         upload)))


class Vision:
    """Camera in this process, model in a background process: `submit` takes
    the photo and returns at once (the robot can drive on), `poll` collects
    finished recognitions, `finish` waits for the rest."""

    def __init__(self, model_dir: Path, out_dir: Path, imgsz: int = 640, conf: float = 0.5,
                 width: int = 640, height: int = 480, burst: int = 1, settle_s: float = 0.3,
                 ready_timeout_s: float = 120.0, images_url: str | None = None) -> None:
        import multiprocessing

        import cv2
        from picamera2 import Picamera2

        self._cv2 = cv2
        self.out_dir = out_dir
        self.burst, self.settle_s = burst, settle_s
        if not (model_dir / "model.ncnn.param").is_file():
            raise FileNotFoundError(f"NCNN model not found in {model_dir}")
        # spawn, not fork: the child must not inherit the camera / serial handles
        context = multiprocessing.get_context("spawn")
        self._jobs, self._done = context.Queue(), context.Queue()
        self._worker = context.Process(target=_inference_worker, name="vision-model", daemon=True,
                                       args=(str(model_dir), imgsz, conf, self._jobs, self._done, images_url))
        self._worker.start()
        self._camera = Picamera2()
        config = self._camera.create_preview_configuration(
            main={"format": "RGB888", "size": (width, height)}, buffer_count=2, queue=False
        )
        self._camera.align_configuration(config)
        self._camera.configure(config)
        self._camera.start()
        import queue

        deadline = time.monotonic() + ready_timeout_s   # wait for the model to load and warm up
        while True:
            try:
                reply = self._done.get(timeout=1.0)
                break
            except queue.Empty:
                if not self._worker.is_alive() or time.monotonic() > deadline:
                    self.close()
                    raise RuntimeError("vision worker did not start (see its error above)") from None
        if reply != ("ready",):
            raise RuntimeError(f"vision worker did not start: {reply!r}")
        self.results: list[Recognition] = []
        self._pending: list[int] = []
        self._centres: dict[int, float | None] = {}
        self._tag = 0

    def close(self) -> None:
        if self._worker.is_alive():
            self._jobs.put(None)
            self._worker.join(timeout=5)
        self._camera.stop()
        self._camera.close()

    def new_mission(self, mission_id: str) -> None:
        self.mission_dir = self.out_dir / mission_id
        self.mission_dir.mkdir(parents=True, exist_ok=True)
        self.results = []
        self._pending = []

    def _collect(self, block: bool, timeout: float | None = None) -> list[Recognition]:
        """Move finished jobs off the result queue; snaps are returned, centres kept."""
        import queue

        finished = []
        while True:
            try:
                item = self._done.get(block=block, timeout=timeout)
            except queue.Empty:
                return finished
            if item[0] == "centre":
                self._centres[item[1]] = item[2]
            elif item[0] == "snap":
                recognition = item[1]
                if recognition.obstacle_id in self._pending:
                    self._pending.remove(recognition.obstacle_id)
                self.results.append(recognition)
                finished.append(recognition)
            block = False   # after the first item, only take what is already there

    def submit(self, obstacle_id: int) -> None:
        """Settle, take the photo(s) and queue them for the model; returns at once."""
        time.sleep(self.settle_s)   # let motion blur die down after ACK
        frames = [self._camera.capture_array() for _ in range(self.burst)]
        self._pending.append(obstacle_id)
        self._jobs.put(("snap", obstacle_id, frames,
                        str(self.mission_dir / f"obstacle_{obstacle_id}_raw.jpg"),
                        str(self.mission_dir / f"obstacle_{obstacle_id}_boxes.jpg")))

    def poll(self) -> list[Recognition]:
        """Recognitions finished since the last call (never waits)."""
        return self._collect(block=False)

    def finish(self, timeout_s: float = 30.0) -> list[Recognition]:
        """Wait for every submitted photo; the ones still missing after `timeout_s`
        (or if the worker died) come back as not recognised."""
        deadline = time.monotonic() + timeout_s
        finished = []
        while self._pending and time.monotonic() < deadline and self._worker.is_alive():
            finished += self._collect(block=True, timeout=min(1.0, max(0.0, deadline - time.monotonic())))
        for obstacle_id in self._pending:
            print(f"Obstacle {obstacle_id}: no result from the vision worker")
            finished.append(Recognition(obstacle_id, None, 0.0,
                                        self.mission_dir / f"obstacle_{obstacle_id}_raw.jpg",
                                        self.mission_dir / f"obstacle_{obstacle_id}_boxes.jpg"))
        self._pending = []
        return finished

    def recognise(self, obstacle_id: int) -> Recognition:
        """Synchronous: photo + wait for this obstacle's result."""
        self.submit(obstacle_id)
        while obstacle_id in self._pending and self._worker.is_alive():
            self._collect(block=True, timeout=1.0)
        return next((r for r in reversed(self.results) if r.obstacle_id == obstacle_id), None) \
            or self.finish(0)[-1]

    def centre_x(self, obstacle_id: int, count: int) -> float | None:
        """One settled frame -> the obstacle's horizontal position (-1..+1), or
        None. Waits for the worker (used only by --align)."""
        time.sleep(self.settle_s)
        frame = self._camera.capture_array()
        self._tag += 1
        tag = self._tag
        self._jobs.put(("centre", tag, frame, str(self.mission_dir / f"obstacle_{obstacle_id}_align_{count}.jpg")))
        while tag not in self._centres and self._worker.is_alive():
            self._collect(block=True, timeout=1.0)
        return self._centres.pop(tag, None)

    def stitch(self) -> Path | None:
        """Tile every annotated capture into one image for verification."""
        tiles = [r for r in self.results if r.annotated_path.is_file()]
        if not tiles:
            return None
        cv2 = self._cv2
        import numpy as np

        tiles = [cv2.imread(str(r.annotated_path)) for r in tiles]
        tile_h, tile_w = tiles[0].shape[:2]
        columns = min(4, len(tiles))
        rows = math.ceil(len(tiles) / columns)
        canvas = np.zeros((rows * tile_h, columns * tile_w, 3), dtype=np.uint8)
        for index, tile in enumerate(tiles):
            row, column = divmod(index, columns)
            tile = cv2.resize(tile, (tile_w, tile_h))
            canvas[row * tile_h:(row + 1) * tile_h, column * tile_w:(column + 1) * tile_w] = tile
        path = self.mission_dir / "stitched.jpg"
        cv2.imwrite(str(path), canvas)
        return path
