import argparse
import json
import mimetypes
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse
import time

from planning_service import plan_request
from planning_service_v2 import plan_request_v2
import image_store


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5000
MAX_REQUEST_BYTES = 1_000_000
Planner = Callable[[Any], dict[str, Any]]

WEB_DIR = Path(__file__).resolve().parent / "web"

# Sentinel returned by `_read_json_body()` on a transport-level failure (an
# error response has already been written). Distinct from `None`, which is
# a legitimate parsed JSON value (the literal `null`) that must still reach
# the endpoint's own validator instead of being treated as a read failure.
_BODY_READ_FAILED = object()


class PlanningHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        planner: Planner,
        verbose: bool = True,
        planner_v2: Planner = plan_request_v2,
    ):
        self.planner = planner
        self.planner_v2 = planner_v2
        self.verbose = verbose
        self.latest_request = None
        self.latest_response = None
        # Which protocol version the most recent plan request used, and who
        # sent it. Display only (GET /state/versions): nothing plans from it.
        self._last_request_lock = threading.Lock()
        self._last_plan_request: dict[str, Any] | None = None
        # RAW camera images (PROTOCOL.md section 7.1): in memory only, display only.
        self.image_store = image_store.ImageStore()
        # Original Roadmap Step 5 (v2 backend integration): one atomic,
        # lock-protected snapshot backing GET /state/v2. Kept fully separate
        # from latest_request/latest_response above so a v2 call can never
        # be observed via v1's /state, and vice versa.
        self.v2_state_lock = threading.Lock()
        self.v2_state: dict[str, Any] = {
            "mission_id": None,
            "request": None,
            "status": "idle",  # "idle" | "planning" | "complete" | "error"
            "result": None,
            "error": None,
        }
        # Non-blocking single-flight guard: only one /plan/v2 request plans
        # at a time (executable-pipeline searches are CPU-heavy and can run
        # for up to the bounded request window -- see bounded_prefix_planning.py).
        self._v2_planning_lock = threading.Lock()
        super().__init__(address, PlanningRequestHandler)

    def set_v2_state(self, **fields: Any) -> None:
        with self.v2_state_lock:
            self.v2_state.update(fields)

    def get_v2_state(self) -> dict[str, Any]:
        with self.v2_state_lock:
            return dict(self.v2_state)

    def record_plan_request(
        self, protocol_version: int, mission_id: str | None, user_agent: str,
        mode: str | None = None,
    ) -> None:
        """Remember the newest plan request: its endpoint's version, its
        mission id (v2 only, capped), the mode it asked for (capped; None when
        it is not a string) and its sender. A browser (the manual page) sends a
        `Mozilla` user agent; the RPi's script or curl does not."""
        with self._last_request_lock:
            self._last_plan_request = {
                "protocol_version": protocol_version,
                "mission_id": mission_id[:128] if mission_id else None,
                "mode": mode[:16] if mode else None,
                "source": "browser" if "Mozilla" in user_agent else "rpi",
                "received_at": time.monotonic(),
            }

    def last_plan_request(self) -> dict[str, Any] | None:
        with self._last_request_lock:
            if self._last_plan_request is None:
                return None
            snapshot = dict(self._last_plan_request)
        received_at = snapshot.pop("received_at")
        snapshot["age_s"] = round(max(0.0, time.monotonic() - received_at), 1)
        return snapshot


class PlanningRequestHandler(BaseHTTPRequestHandler):
    server: PlanningHTTPServer

    def _write_json(self, status: int, payload: dict[str, Any], close: bool = False) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if close:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _write_file(self, path: Path) -> None:
        try:
            body = path.read_bytes()
        except OSError:
            self._error(404, "web asset not found")
            return

        content_type = mimetypes.guess_type(
            path.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status: int, message: str, protocol_version: int = 1) -> None:
        self._write_json(
            status,
            {
                "type": "PLAN_ERROR",
                "protocol_version": protocol_version,
                "valid": False,
                "error": message,
            },
        )

    def do_GET(self) -> None:
        path = urlparse(self.path).path

        if path == "/health":
            self._write_json(
                200,
                {"status": "ok", "service": "mdp-path-planner"},
            )
            return

        if path == "/state":
            self._write_json(
                200,
                {
                    "request": self.server.latest_request,
                    "result": self.server.latest_response,
                },
            )
            return

        if path == "/state/v2":
            self._write_json(200, self.server.get_v2_state())
            return

        if path == "/state/versions":
            self._write_json(
                200,
                {"type": "PLAN_VERSIONS", "last_request": self.server.last_plan_request()},
            )
            return

        if path == "/images":
            self._write_json(200, self.server.image_store.list_images())
            return

        if path.startswith("/images/"):
            self._handle_get_image(path[len("/images/"):])
            return

        # `/` is the automatic planner page; `/manual.html` keeps the manual
        # v1/v2 debugging page (deliberately unlinked from `/`).
        assets = {
            "/": WEB_DIR / "index.html",
            "/index.html": WEB_DIR / "index.html",
            "/auto.js": WEB_DIR / "auto.js",
            "/autoplan.js": WEB_DIR / "autoplan.js",
            "/arena.js": WEB_DIR / "arena.js",
            "/gallery.js": WEB_DIR / "gallery.js",
            "/versions.js": WEB_DIR / "versions.js",
            "/manual.html": WEB_DIR / "manual.html",
            "/app.js": WEB_DIR / "app.js",
            "/planv2.js": WEB_DIR / "planv2.js",
            "/style.css": WEB_DIR / "style.css",
        }

        asset = assets.get(path)
        if asset is not None:
            self._write_file(asset)
            return

        self._error(404, "endpoint not found")

    def _read_json_body(self, protocol_version: int) -> Any:
        """Shared transport-level validation for a POST body. Writes an
        error response (tagged with `protocol_version`) and returns the
        `_BODY_READ_FAILED` sentinel on any failure; returns the parsed JSON
        value -- which may legitimately be `None` for a body of literal
        `null` -- on success. Never touches any endpoint's own state --
        callers own that.

        Content-Type is checked, and rejected, before any attempt to read
        the body: a client that declared a body it never sends (or sends
        slowly) must not be able to make this call block indefinitely.
        """
        content_type = self.headers.get_content_type()
        if content_type != "application/json":
            self._error(415, "Content-Type must be application/json", protocol_version)
            return _BODY_READ_FAILED

        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._error(400, "Content-Length must be an integer", protocol_version)
            return _BODY_READ_FAILED

        if length <= 0:
            self._error(400, "request body must not be empty", protocol_version)
            return _BODY_READ_FAILED

        if length > MAX_REQUEST_BYTES:
            self._error(413, "request body is too large", protocol_version)
            return _BODY_READ_FAILED

        try:
            raw_body = self.rfile.read(length)
            return json.loads(raw_body)
        except UnicodeDecodeError:
            self._error(400, "request body must be valid UTF-8", protocol_version)
        except json.JSONDecodeError:
            self._error(400, "request body must be valid JSON", protocol_version)
        return _BODY_READ_FAILED

    def do_POST(self) -> None:
        path = urlparse(self.path).path

        if path == "/plan":
            self._handle_plan_v1()
            return

        if path == "/plan/v2":
            self._handle_plan_v2()
            return

        if path == "/images":
            self._handle_post_image()
            return

        self._error(404, "endpoint not found")

    def _handle_plan_v1(self) -> None:
        payload = self._read_json_body(protocol_version=1)
        if payload is _BODY_READ_FAILED:
            return
        sent_mode = payload.get("mode") if isinstance(payload, dict) else None
        self.server.record_plan_request(
            1, None, self.headers.get("User-Agent", ""),
            mode=sent_mode if isinstance(sent_mode, str) else None,
        )

        self.server.latest_request = payload

        try:
            print("PLAN START:", payload)
            start_time = time.perf_counter()

            response = self.server.planner(payload)

            elapsed = time.perf_counter() - start_time
            print(f"PLAN FINISHED: {elapsed:.3f}s")
        except Exception:
            self._error(500, "planner failed unexpectedly")
            return

        self.server.latest_response = response

        status = 400 if response.get("type") == "PLAN_ERROR" else 200
        self._write_json(status, response)

    def _handle_plan_v2(self) -> None:
        """Bounded protocol-v2 backend integration.

        Transport-level failures (bad Content-Type/Length/body/JSON) are
        rejected by `_read_json_body()` before the single-flight guard is
        ever touched, so a malformed request never disturbs an in-flight
        mission's `v2_state`. Once a body is decoded, only one `/plan/v2`
        request plans at a time; a concurrent one gets HTTP 409 without
        touching `v2_state` at all. The guard is always released in
        `finally`, on success, on an ordinary validation failure, and on an
        internal exception alike.
        """
        payload = self._read_json_body(protocol_version=2)
        if payload is _BODY_READ_FAILED:
            return
        sent_mission = payload.get("mission_id") if isinstance(payload, dict) else None
        sent_mode = payload.get("mode") if isinstance(payload, dict) else None
        self.server.record_plan_request(
            2, sent_mission if isinstance(sent_mission, str) else None,
            self.headers.get("User-Agent", ""),
            mode=sent_mode if isinstance(sent_mode, str) else None,
        )

        acquired = self.server._v2_planning_lock.acquire(blocking=False)
        if not acquired:
            self._error(
                409, "planner is busy processing another v2 request", protocol_version=2
            )
            return

        try:
            mission_id = payload.get("mission_id") if isinstance(payload, dict) else None
            if not isinstance(mission_id, str) or not mission_id:
                mission_id = None
            self.server.set_v2_state(
                mission_id=mission_id, request=payload, status="planning",
                result=None, error=None,
            )

            try:
                print("PLAN V2 START:", payload)
                start_time = time.perf_counter()
                response = self.server.planner_v2(payload)
                elapsed = time.perf_counter() - start_time
                print(f"PLAN V2 FINISHED: {elapsed:.3f}s")
                if not isinstance(response, dict) or response.get("type") not in (
                    "PLAN_RESULT", "PLAN_ERROR",
                ):
                    raise TypeError(
                        f"planner_v2 returned a malformed response: {response!r}"
                    )
            except Exception as exc:
                # Diagnostic detail stays server-side only; the public state
                # and HTTP response both get the same generic message. This
                # also covers a malformed (non-dict, or unrecognised `type`)
                # planner return -- a contract failure, not an ordinary plan
                # outcome -- caught here rather than inspected separately.
                print(f"PLAN V2 INTERNAL ERROR: {exc!r}")
                self.server.set_v2_state(status="error", error="planner failed unexpectedly")
                self._error(500, "planner failed unexpectedly", protocol_version=2)
                return

            if response.get("type") == "PLAN_ERROR":
                self.server.set_v2_state(status="error", error=response.get("error"))
                self._write_json(400, response)
                return

            self.server.set_v2_state(status="complete", result=response, error=None)
            self._write_json(200, response)
        finally:
            self.server._v2_planning_lock.release()


    def _image_error(self, status: int, message: str, close: bool = False) -> None:
        self._write_json(status, {"type": "IMAGE_ERROR", "error": message}, close=close)

    def _handle_get_image(self, image_id: str) -> None:
        data = self.server.image_store.get(image_id)
        if data is None:
            self._image_error(404, "no such image")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _handle_post_image(self) -> None:
        """`POST /images?mission_id=..&obstacle_id=..[&target_id=..]` with a JPEG body
        (`PROTOCOL.md` section 7.1). Touches no plan state.

        The body is read before any other check, so an error reply never
        leaves unread bytes on the connection. The only exception is a body
        that is too large: it is rejected from `Content-Length` alone and the
        connection is closed."""
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._image_error(400, "Content-Length must be an integer", close=True)
            return
        if length <= 0:
            self._image_error(400, "request body must not be empty", close=True)
            return
        if length > image_store.MAX_IMAGE_BYTES:
            self._image_error(413, f"image is larger than {image_store.MAX_IMAGE_BYTES} bytes", close=True)
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
            for key in ("mission_id", "obstacle_id"):
                if len(query.get(key, [])) != 1:
                    raise image_store.ImageRejected(400, f"exactly one {key} query parameter is required")
            obstacle_id = image_store.parse_obstacle_id(query["obstacle_id"][0])
            target_id = None
            if "target_id" in query:
                if len(query["target_id"]) != 1:
                    raise image_store.ImageRejected(400, "at most one target_id query parameter is allowed")
                target_id = image_store.parse_target_id(query["target_id"][0])
            stored = self.server.image_store.store(query["mission_id"][0], obstacle_id, data, target_id)
        except image_store.ImageRejected as rejected:
            self._image_error(rejected.status, rejected.message)
            return
        self._write_json(200, {"type": "IMAGE_STORED", **stored})


def create_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    planner: Planner = plan_request,
    verbose: bool = True,
    planner_v2: Planner = plan_request_v2,
) -> PlanningHTTPServer:
    return PlanningHTTPServer((host, port), planner, verbose, planner_v2)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve the MDP planner over HTTP")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    server = create_server(args.host, args.port, verbose=not args.quiet)
    host, port = server.server_address
    print(f"MDP planner listening on http://{host}:{port}")
    print("Web UI: GET /")
    print("RPi endpoints: GET /health and POST /plan")
    print("Bounded v2 (1-8, planning only): POST /plan/v2, GET /state/v2")
    print("RAW camera images (display only): POST /images, GET /images")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping MDP planner")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
