"""Task 2 arrow pictures, RPi to PC: the RAW camera image with its bounding boxes for each obstacle's arrow.

Run it on the PC/laptop, open http://localhost:5001/ in a browser, and start final_run.py on the Pi with
`--pc <laptop address>`. The page shows the two pictures side by side as they arrive. The rules (Task 2, rules 8
and 10, FAQ 16) need this screen at the end of the run: the supervisor photographs it, and the team emails a
screenshot.

Same shape as the Task 1 planner's image endpoints (algo/server.py, algo/image_store.py), cut down to two
pictures:

    POST /images?run_id=<run>&obstacle_id=<1|2>&arrow=<LEFT|RIGHT>   body: one JPEG
    GET  /images            the current pictures (JSON)
    GET  /images/<1|2>      one picture (JPEG)
    GET  /                  the page
    GET  /health

In memory only (a restart clears it). A picture from a new run_id clears the old run's pair. Only the JPEG
markers and the size are checked; the picture is not decoded. Standard library only.
"""

from __future__ import annotations

import argparse
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001                 # the Task 1 planner uses 5000
MAX_IMAGE_BYTES = 2 * 1024 * 1024
MAX_RUN_ID_CHARS = 128
ARROWS = ("LEFT", "RIGHT")
_OBSTACLE_RE = re.compile(r"[12]")
_JPEG_START, _JPEG_END = b"\xff\xd8\xff", b"\xff\xd9"


class ImageRejected(Exception):
    """A client error: carries the HTTP status and a short public message."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


def validate_jpeg(data: bytes) -> None:
    if not data:
        raise ImageRejected(400, "request body must not be empty")
    if len(data) > MAX_IMAGE_BYTES:
        raise ImageRejected(413, f"image is larger than {MAX_IMAGE_BYTES} bytes")
    if not (len(data) >= 5 and data.startswith(_JPEG_START) and data.endswith(_JPEG_END)):
        raise ImageRejected(400, "body is not a complete JPEG")


class ArrowImages:
    """Thread-safe: one picture per obstacle (1, 2) for the current run."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._run_id: str | None = None
        self._images: dict[int, dict[str, Any]] = {}
        self._revision = 0

    def store(self, run_id: str, obstacle_id: str, arrow: str, data: bytes) -> dict[str, Any]:
        if not isinstance(run_id, str) or not run_id or len(run_id) > MAX_RUN_ID_CHARS \
                or any(ord(ch) < 32 or ord(ch) == 127 for ch in run_id):
            raise ImageRejected(400, f"run_id must be 1-{MAX_RUN_ID_CHARS} printable characters")
        if not isinstance(obstacle_id, str) or not _OBSTACLE_RE.fullmatch(obstacle_id):
            raise ImageRejected(400, "obstacle_id must be 1 or 2")
        if arrow not in ARROWS:
            raise ImageRejected(400, "arrow must be LEFT or RIGHT")
        validate_jpeg(data)
        with self._lock:
            if run_id != self._run_id:
                self._run_id, self._images = run_id, {}
            self._revision += 1
            self._images[int(obstacle_id)] = {"arrow": arrow, "data": data, "revision": self._revision,
                                              "received_at_s": time.time()}
            return {"run_id": run_id, "obstacle_id": int(obstacle_id), "arrow": arrow, "revision": self._revision}

    def listing(self) -> dict[str, Any]:
        with self._lock:
            return {"type": "ARROW_IMAGES", "run_id": self._run_id, "revision": self._revision,
                    "images": [{"obstacle_id": n, "arrow": image["arrow"], "revision": image["revision"],
                                "url": f"/images/{n}?rev={image['revision']}"}
                               for n, image in sorted(self._images.items())]}

    def get(self, obstacle_id: str) -> bytes | None:
        if not _OBSTACLE_RE.fullmatch(obstacle_id):
            return None
        with self._lock:
            image = self._images.get(int(obstacle_id))
            return None if image is None else image["data"]


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Task 2 Arrows</title>
<style>
  :root { --bg: #f4f4f2; --card: #ffffff; --ink: #1d1d1b; --muted: #6b6b66; --line: #d9d9d4; }
  @media (prefers-color-scheme: dark) {
    :root { --bg: #161615; --card: #20201f; --ink: #ececea; --muted: #9a9a94; --line: #34342f; }
  }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 16px; background: var(--bg); color: var(--ink);
         font: 16px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  header { display: flex; justify-content: space-between; align-items: baseline; flex-wrap: wrap; gap: 8px;
           margin-bottom: 16px; }
  h1 { font-size: 22px; margin: 0; }
  #run { color: var(--muted); font-size: 14px; }
  .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }
  @media (max-width: 700px) { .grid { grid-template-columns: 1fr; } }
  .card { background: var(--card); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }
  .card h2 { font-size: 18px; margin: 0; padding: 10px 14px; border-bottom: 1px solid var(--line); }
  .card h2 span { font-weight: 700; }
  .card img, .empty { display: block; width: 100%; aspect-ratio: 4 / 3; }
  .card img { object-fit: contain; background: #000; }
  .empty { display: grid; place-items: center; color: var(--muted); }
</style>
</head>
<body>
<header><h1>Task 2: arrows recognised</h1><div id="run">{run}</div></header>
<div class="grid">
{cards}
</div>
<script>
let shown = {revision};
async function refresh() {
  try {
    const reply = await fetch("/images", {cache: "no-store"});
    const data = await reply.json();
    if (data.revision === shown) return;
    shown = data.revision;
    document.getElementById("run").textContent = data.run_id ? "run " + data.run_id : "waiting for the robot";
    for (const n of [1, 2]) {  // the same markup render_page() writes
      const card = document.getElementById("o" + n);
      const image = data.images.find(i => i.obstacle_id === n);
      card.querySelector("h2 span").textContent = image ? image.arrow : "-";
      const body = card.lastElementChild;
      if (image) {
        const img = document.createElement("img");
        img.src = image.url;
        img.alt = "Obstacle " + n + " arrow " + image.arrow;
        body.replaceWith(img);
      } else if (body.tagName === "IMG") {
        const empty = document.createElement("div");
        empty.className = "empty";
        empty.textContent = "no picture yet";
        body.replaceWith(empty);
      }
    }
  } catch (e) { /* the server restarted or is unreachable: try again */ }
}
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


def render_page(listing: dict[str, Any]) -> str:
    """The page with the current pictures already in it (they show on first paint); the script then polls
    /images once a second for new ones."""
    from html import escape

    cards = []
    for n in (1, 2):
        image = next((i for i in listing["images"] if i["obstacle_id"] == n), None)
        arrow = escape(image["arrow"]) if image else "-"
        body = (f'<img src="{escape(image["url"])}" alt="Obstacle {n} arrow {arrow}">' if image
                else '<div class="empty">no picture yet</div>')
        cards.append(f'  <div class="card" id="o{n}"><h2>Obstacle {n}: <span>{arrow}</span></h2>{body}</div>')
    run = f"run {escape(listing['run_id'])}" if listing["run_id"] else "waiting for the robot"
    return (PAGE.replace("{run}", run).replace("{cards}", "\n".join(cards))
            .replace("{revision}", str(listing["revision"] if listing["run_id"] else -1)))


class ArrowHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], verbose: bool = True):
        self.images = ArrowImages()
        self.verbose = verbose
        super().__init__(address, ArrowRequestHandler)


class ArrowRequestHandler(BaseHTTPRequestHandler):
    server: ArrowHTTPServer

    def log_message(self, format: str, *args: Any) -> None:
        if self.server.verbose:
            super().log_message(format, *args)

    def _write(self, status: int, body: bytes, content_type: str, close: bool = False) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if close:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _write_json(self, status: int, payload: dict[str, Any], close: bool = False) -> None:
        self._write(status, json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                    "application/json; charset=utf-8", close)

    def _image_error(self, status: int, message: str, close: bool = False) -> None:
        self._write_json(status, {"type": "IMAGE_ERROR", "error": message}, close=close)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/health":
            self._write_json(200, {"status": "ok", "service": "task2-arrow-images"})
        elif path in ("/", "/index.html"):
            self._write(200, render_page(self.server.images.listing()).encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/images":
            self._write_json(200, self.server.images.listing())
        elif path.startswith("/images/"):
            data = self.server.images.get(path[len("/images/"):])
            if data is None:
                self._image_error(404, "no such image")
            else:
                self._write(200, data, "image/jpeg")
        else:
            self._image_error(404, "not found")

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/images":
            self._image_error(404, "not found", close=True)
            return
        # The body is read before any other check, so an error reply never leaves unread bytes on the
        # connection; only a body that is too large is refused from Content-Length alone (as algo/server.py).
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._image_error(400, "Content-Length must be an integer", close=True)
            return
        if length <= 0:
            self._image_error(400, "request body must not be empty", close=True)
            return
        if length > MAX_IMAGE_BYTES:
            self._image_error(413, f"image is larger than {MAX_IMAGE_BYTES} bytes", close=True)
            return
        try:
            self.connection.settimeout(30)
            data = self.rfile.read(length)
        except OSError:
            self.close_connection = True
            return
        if len(data) != length:
            self.close_connection = True
            return
        if self.headers.get_content_type() != "image/jpeg":
            self._image_error(415, "Content-Type must be image/jpeg")
            return
        query = parse_qs(urlparse(self.path).query, keep_blank_values=True)
        try:
            values = {}
            for key in ("run_id", "obstacle_id", "arrow"):
                if len(query.get(key, [])) != 1:
                    raise ImageRejected(400, f"exactly one {key} query parameter is required")
                values[key] = query[key][0]
            stored = self.server.images.store(values["run_id"], values["obstacle_id"], values["arrow"], data)
        except ImageRejected as rejected:
            self._image_error(rejected.status, rejected.message)
            return
        if self.server.verbose:
            print(f"arrow picture: run {stored['run_id']}, obstacle {stored['obstacle_id']}: {stored['arrow']}")
        self._write_json(200, {"type": "IMAGE_STORED", **stored})


def create_server(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, verbose: bool = True) -> ArrowHTTPServer:
    return ArrowHTTPServer((host, port), verbose)


def main() -> None:
    parser = argparse.ArgumentParser(description="Show the Task 2 arrow pictures sent by the Pi")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    server = create_server(args.host, args.port, verbose=not args.quiet)
    host, port = server.server_address
    print(f"Task 2 arrow pictures listening on http://{host}:{port}")
    print(f"Page: http://localhost:{port}/   (Pi: final_run.py --pc <this laptop's address>:{port})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
