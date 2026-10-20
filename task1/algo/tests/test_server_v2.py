"""Original Roadmap Step 5 -- server-level tests for the v2 HTTP integration.

First tests in this repo to spin up `PlanningHTTPServer` on a real (ephemeral)
port in a background thread. Concurrency is driven entirely by
`threading.Event`s a fake planner sets/waits on -- never by arbitrary
sleeps -- so these tests are deterministic and fast.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import http.client
import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import server as server_module  # noqa: E402
from planning_service import plan_request  # noqa: E402
from planning_service_v2 import plan_request_v2  # noqa: E402

HOST = "127.0.0.1"


def _valid_v1_payload():
    return {
        "type": "PLAN_REQUEST",
        "protocol_version": 1,
        "mode": "B2",
        "coordinate_units": "cm",
        "start": {"x": 0, "y": 0, "heading": "E"},
        "obstacles": [{"id": 1, "x": 50, "y": 50, "face": "N"}],
    }


def _valid_v2_payload(mission_id="mission-1"):
    return {
        "type": "PLAN_REQUEST",
        "protocol_version": 2,
        "mission_id": mission_id,
        "mode": "B2",
        "coordinate_units": "cm",
        "allow_alternative_viewing": False,
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "obstacles": [{"id": 1, "x": 50, "y": 50, "face": "N"}],
    }


def _fixed_result(mission_id="mission-1"):
    return {
        "type": "PLAN_RESULT",
        "protocol_version": 2,
        "mission_id": mission_id,
        "valid": True,
        "complete": True,
        "obstacle_order": [1],
    }


class BlockingPlanner:
    """Injectable v2 planner that blocks until the test releases it.

    `started` fires the instant the handler thread enters this callable
    (proving the request is genuinely mid-flight); `release` is waited on
    before returning/raising, so the test controls exactly when the
    "slow" plan finishes -- no sleeps anywhere.
    """

    def __init__(self):
        self.started = threading.Event()
        self.release = threading.Event()
        self.result: dict | None = None  # None => echo the payload's own mission_id
        self.exception: Exception | None = None

    def __call__(self, payload):
        self.started.set()
        self.release.wait(timeout=10)
        if self.exception is not None:
            raise self.exception
        if self.result is not None:
            return self.result
        return _fixed_result(payload.get("mission_id"))


def _raising_planner(payload):
    raise RuntimeError("simulated internal planner bug")


def _fixed_planner():
    """Echoes whatever mission_id the incoming payload actually carries --
    a real planner's job, and exactly what these tests need to verify."""
    def planner(payload):
        return _fixed_result(payload.get("mission_id"))
    return planner


class _ServerTestCase(unittest.TestCase):
    """Starts a real PlanningHTTPServer on an ephemeral port for each test."""

    planner = staticmethod(plan_request)
    planner_v2 = staticmethod(_fixed_planner())

    def setUp(self):
        self.server = server_module.create_server(
            host=HOST, port=0, planner=self.planner, verbose=False,
            planner_v2=self.planner_v2,
        )
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)

    def _request(self, method, path, body=None, headers=None, content_type="application/json"):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            data = json.dumps(body).encode("utf-8") if body is not None else None
            hdrs = {"Content-Type": content_type}
            if data is not None:
                hdrs["Content-Length"] = str(len(data))
            if headers:
                hdrs.update(headers)
            conn.request(method, path, body=data, headers=hdrs)
            response = conn.getresponse()
            raw = response.read()
            parsed = json.loads(raw) if raw else None
            return response.status, parsed
        finally:
            conn.close()

    def _post_v2(self, payload):
        return self._request("POST", "/plan/v2", body=payload)

    def _post_v1(self, payload):
        return self._request("POST", "/plan", body=payload)

    def _get(self, path):
        return self._request("GET", path)

    def _raw_post(self, path, header_lines, body: bytes = b"", timeout: float = 5.0):
        """Send a hand-crafted POST over a raw socket, for cases
        `http.client` would normalise away: a missing/corrupt Content-Length,
        or a body that is never actually sent despite being declared."""
        sock = socket.create_connection((HOST, self.port), timeout=timeout)
        try:
            head = "\r\n".join(
                [f"POST {path} HTTP/1.1", f"Host: {HOST}"]
                + list(header_lines)
                + ["Connection: close", "", ""]
            ).encode("ascii")
            sock.sendall(head + body)
            chunks = []
            while True:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
            raw = b"".join(chunks)
        finally:
            sock.close()
        header_part, _, body_part = raw.partition(b"\r\n\r\n")
        status = int(header_part.split(b"\r\n", 1)[0].split(b" ")[1])
        parsed = json.loads(body_part) if body_part else None
        return status, parsed


class AtomicStateTransitions(_ServerTestCase):
    def setUp(self):
        self.blocking = BlockingPlanner()
        self.planner_v2 = self.blocking
        super().setUp()

    def test_status_transitions_planning_then_complete(self):
        self.blocking.result = _fixed_result("mission-atomic")
        results = {}

        def worker():
            results["response"] = self._post_v2(_valid_v2_payload("mission-atomic"))

        thread = threading.Thread(target=worker)
        thread.start()

        self.assertTrue(self.blocking.started.wait(timeout=5), "planner never started")

        status, state = self._get("/state/v2")
        self.assertEqual(status, 200)
        self.assertEqual(state["status"], "planning")
        self.assertEqual(state["mission_id"], "mission-atomic")
        self.assertIsNone(state["result"])
        self.assertIsNone(state["error"])

        self.blocking.release.set()
        thread.join(timeout=5)

        post_status, post_body = results["response"]
        self.assertEqual(post_status, 200)
        self.assertEqual(post_body["mission_id"], "mission-atomic")

        status, state = self._get("/state/v2")
        self.assertEqual(state["status"], "complete")
        self.assertEqual(state["mission_id"], "mission-atomic")
        self.assertEqual(state["result"], post_body)
        self.assertIsNone(state["error"])

    def test_get_and_static_endpoints_remain_usable_while_v2_is_planning(self):
        thread = threading.Thread(target=lambda: self._post_v2(_valid_v2_payload()))
        thread.start()
        self.assertTrue(self.blocking.started.wait(timeout=5))

        health_status, health_body = self._get("/health")
        self.assertEqual(health_status, 200)
        self.assertEqual(health_body["status"], "ok")

        state_status, _ = self._get("/state")
        self.assertEqual(state_status, 200)

        v1_status, v1_body = self._post_v1(_valid_v1_payload())
        self.assertIn(v1_status, (200, 400))
        self.assertEqual(v1_body["protocol_version"], 1)

        self.blocking.release.set()
        thread.join(timeout=5)


class BusyGuard(_ServerTestCase):
    def setUp(self):
        self.blocking = BlockingPlanner()
        self.planner_v2 = self.blocking
        super().setUp()

    def test_concurrent_request_receives_409_and_leaves_state_untouched(self):
        thread = threading.Thread(target=lambda: self._post_v2(_valid_v2_payload("mission-first")))
        thread.start()
        self.assertTrue(self.blocking.started.wait(timeout=5))

        status, body = self._post_v2(_valid_v2_payload("mission-second"))
        self.assertEqual(status, 409)
        self.assertEqual(body["type"], "PLAN_ERROR")
        self.assertEqual(body["protocol_version"], 2)

        _, state = self._get("/state/v2")
        self.assertEqual(state["mission_id"], "mission-first")
        self.assertEqual(state["status"], "planning")

        self.blocking.release.set()
        thread.join(timeout=5)

    def test_guard_is_released_after_success_and_available_for_next_request(self):
        thread = threading.Thread(target=lambda: self._post_v2(_valid_v2_payload("mission-first")))
        thread.start()
        self.assertTrue(self.blocking.started.wait(timeout=5))
        self.blocking.release.set()
        thread.join(timeout=5)

        status, body = self._post_v2(_valid_v2_payload("mission-second"))
        self.assertEqual(status, 200)
        self.assertEqual(body["mission_id"], "mission-second")


class GuardReleaseAfterException(_ServerTestCase):
    planner_v2 = staticmethod(_raising_planner)

    def test_internal_exception_returns_500_without_exposing_detail(self):
        status, body = self._post_v2(_valid_v2_payload("mission-boom"))
        self.assertEqual(status, 500)
        self.assertEqual(body["protocol_version"], 2)
        self.assertEqual(body["error"], "planner failed unexpectedly")
        self.assertNotIn("simulated internal planner bug", json.dumps(body))

        _, state = self._get("/state/v2")
        self.assertEqual(state["status"], "error")
        self.assertEqual(state["error"], "planner failed unexpectedly")
        self.assertNotIn("simulated internal planner bug", json.dumps(state))

    def test_guard_released_after_exception_allows_next_request(self):
        self._post_v2(_valid_v2_payload("mission-boom"))
        # A fresh server would still use the same raising planner; swap it
        # out to prove the *guard* (not the planner) is what mattered.
        self.server.planner_v2 = _fixed_planner()
        status, body = self._post_v2(_valid_v2_payload("mission-after-boom"))
        self.assertEqual(status, 200)
        self.assertEqual(body["mission_id"], "mission-after-boom")


class TransportErrorProtocolVersions(_ServerTestCase):
    def test_bad_content_type_v2(self):
        status, body = self._request(
            "POST", "/plan/v2", body={"a": 1}, content_type="text/plain"
        )
        self.assertEqual(status, 415)
        self.assertEqual(body["protocol_version"], 2)

    def test_bad_content_type_v1_unchanged(self):
        status, body = self._request(
            "POST", "/plan", body={"a": 1}, content_type="text/plain"
        )
        self.assertEqual(status, 415)
        self.assertEqual(body["protocol_version"], 1)

    def test_content_type_rejected_without_reading_declared_body(self):
        """A wrong Content-Type must be rejected immediately, without ever
        reading the (never-sent) declared body -- reading it would block
        until the client's own timeout instead of the server's."""
        start = time.perf_counter()
        status, body = self._raw_post(
            "/plan/v2",
            ["Content-Type: text/plain", "Content-Length: 5000000"],
            body=b"",  # declared 5MB, actually sends nothing
        )
        elapsed = time.perf_counter() - start
        self.assertEqual(status, 415)
        self.assertEqual(body["protocol_version"], 2)
        self.assertLess(elapsed, 2.0, "response should not wait for the declared body")

    def test_missing_content_length_v2(self):
        status, body = self._raw_post(
            "/plan/v2", ["Content-Type: application/json"], body=b"",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 2)

    def test_missing_content_length_v1(self):
        status, body = self._raw_post(
            "/plan", ["Content-Type: application/json"], body=b"",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 1)

    def test_non_integer_content_length_v2(self):
        status, body = self._raw_post(
            "/plan/v2",
            ["Content-Type: application/json", "Content-Length: abc"],
            body=b"",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 2)

    def test_non_integer_content_length_v1(self):
        status, body = self._raw_post(
            "/plan",
            ["Content-Type: application/json", "Content-Length: abc"],
            body=b"",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 1)

    def test_empty_body_v2(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request("POST", "/plan/v2", body=b"", headers={
                "Content-Type": "application/json", "Content-Length": "0",
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 400)
        self.assertEqual(body["protocol_version"], 2)

    def test_empty_body_v1(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request("POST", "/plan", body=b"", headers={
                "Content-Type": "application/json", "Content-Length": "0",
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 400)
        self.assertEqual(body["protocol_version"], 1)

    def test_invalid_json_v2(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            raw = b"{not json"
            conn.request("POST", "/plan/v2", body=raw, headers={
                "Content-Type": "application/json", "Content-Length": str(len(raw)),
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 400)
        self.assertEqual(body["protocol_version"], 2)

    def test_invalid_json_v1(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            raw = b"{not json"
            conn.request("POST", "/plan", body=raw, headers={
                "Content-Type": "application/json", "Content-Length": str(len(raw)),
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 400)
        self.assertEqual(body["protocol_version"], 1)

    def test_invalid_utf8_v2(self):
        raw = b"\x80\x81\x82"  # invalid UTF-8 continuation bytes, no BOM prefix
        status, body = self._raw_post(
            "/plan/v2",
            ["Content-Type: application/json", f"Content-Length: {len(raw)}"],
            body=raw,
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 2)

    def test_invalid_utf8_v1(self):
        raw = b"\x80\x81\x82"  # invalid UTF-8 continuation bytes, no BOM prefix
        status, body = self._raw_post(
            "/plan",
            ["Content-Type: application/json", f"Content-Length: {len(raw)}"],
            body=raw,
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["protocol_version"], 1)

    def test_oversized_body_v2(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request("POST", "/plan/v2", body=b"", headers={
                "Content-Type": "application/json",
                "Content-Length": str(server_module.MAX_REQUEST_BYTES + 1),
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 413)
        self.assertEqual(body["protocol_version"], 2)

    def test_oversized_body_v1(self):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request("POST", "/plan", body=b"", headers={
                "Content-Type": "application/json",
                "Content-Length": str(server_module.MAX_REQUEST_BYTES + 1),
            })
            response = conn.getresponse()
            body = json.loads(response.read())
        finally:
            conn.close()
        self.assertEqual(response.status, 413)
        self.assertEqual(body["protocol_version"], 1)

    def test_unknown_path_still_defaults_to_v1(self):
        status, body = self._get("/no/such/path")
        self.assertEqual(status, 404)
        self.assertEqual(body["protocol_version"], 1)


class InternalErrorV1(_ServerTestCase):
    planner = staticmethod(_raising_planner)

    def test_internal_exception_v1_returns_500(self):
        status, body = self._post_v1(_valid_v1_payload())
        self.assertEqual(status, 500)
        self.assertEqual(body["protocol_version"], 1)
        self.assertEqual(body["error"], "planner failed unexpectedly")
        self.assertNotIn("simulated internal planner bug", json.dumps(body))


class NullBodyRegression(_ServerTestCase):
    """A request body of the JSON literal `null` is a successfully parsed
    value, not a transport failure -- it must reach each protocol's own
    validator (and get an ordinary 400 PLAN_ERROR there), never be silently
    dropped as if the read had failed. Uses the real `plan_request_v2`
    validator (not the fixed fake planner) since a fake that unconditionally
    calls `payload.get(...)` would itself crash on a `None` payload."""

    planner_v2 = staticmethod(plan_request_v2)

    def test_valid_json_null_v1_reaches_planner(self):
        status, body = self._raw_post(
            "/plan", ["Content-Type: application/json", "Content-Length: 4"],
            body=b"null",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["type"], "PLAN_ERROR")
        self.assertEqual(body["protocol_version"], 1)
        self.assertEqual(body["error"], "request must be a JSON object")

    def test_valid_json_null_v2_reaches_validator(self):
        status, body = self._raw_post(
            "/plan/v2", ["Content-Type: application/json", "Content-Length: 4"],
            body=b"null",
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["type"], "PLAN_ERROR")
        self.assertEqual(body["protocol_version"], 2)
        self.assertIn("object", body["error"])


class MalformedPlannerReturn(_ServerTestCase):
    def test_non_dict_response_is_internal_failure(self):
        self.server.planner_v2 = lambda payload: "not a dict"
        status, body = self._post_v2(_valid_v2_payload())
        self.assertEqual(status, 500)
        self.assertEqual(body["protocol_version"], 2)
        self.assertEqual(body["error"], "planner failed unexpectedly")
        _, state = self._get("/state/v2")
        self.assertEqual(state["status"], "error")
        self.assertEqual(state["error"], "planner failed unexpectedly")

    def test_unknown_type_response_is_internal_failure(self):
        self.server.planner_v2 = lambda payload: {"type": "SOMETHING_ELSE"}
        status, body = self._post_v2(_valid_v2_payload())
        self.assertEqual(status, 500)
        self.assertEqual(body["protocol_version"], 2)
        self.assertEqual(body["error"], "planner failed unexpectedly")

    def test_guard_released_after_malformed_response(self):
        self.server.planner_v2 = lambda payload: "not a dict"
        self._post_v2(_valid_v2_payload("mission-malformed"))
        self.server.planner_v2 = _fixed_planner()
        status, body = self._post_v2(_valid_v2_payload("mission-after-malformed"))
        self.assertEqual(status, 200)
        self.assertEqual(body["mission_id"], "mission-after-malformed")


class StateAndGuardCoverage(_ServerTestCase):
    def setUp(self):
        self.blocking = BlockingPlanner()
        self.planner_v2 = self.blocking
        super().setUp()

    def test_new_accepted_mission_clears_previous_completed_result(self):
        self.blocking.result = _fixed_result("mission-a")
        self.blocking.release.set()
        first_status, first_body = self._post_v2(_valid_v2_payload("mission-a"))
        self.assertEqual(first_status, 200)
        _, state_after_first = self._get("/state/v2")
        self.assertEqual(state_after_first["status"], "complete")
        self.assertEqual(state_after_first["result"], first_body)

        self.blocking.release.clear()
        self.blocking.started.clear()
        thread = threading.Thread(
            target=lambda: self._post_v2(_valid_v2_payload("mission-b"))
        )
        thread.start()
        self.assertTrue(self.blocking.started.wait(timeout=5))

        _, state_while_planning = self._get("/state/v2")
        self.assertEqual(state_while_planning["mission_id"], "mission-b")
        self.assertEqual(state_while_planning["status"], "planning")
        self.assertIsNone(state_while_planning["result"])
        self.assertIsNone(state_while_planning["error"])

        self.blocking.release.set()
        thread.join(timeout=5)


class ValidationFailureStateAndGuard(_ServerTestCase):
    """Uses the real `plan_request_v2` validator (not a fake) so an actual
    `RequestErrorV2` is what drives the state transition and guard release
    being tested here."""

    planner_v2 = staticmethod(plan_request_v2)

    def test_validation_failure_sets_coherent_error_snapshot_no_stale_result(self):
        good_status, good_body = self._post_v2(_valid_v2_payload("mission-good"))
        self.assertEqual(good_status, 200)

        bad_payload = _valid_v2_payload("mission-bad")
        del bad_payload["mode"]
        bad_status, bad_body = self._post_v2(bad_payload)
        self.assertEqual(bad_status, 400)
        self.assertEqual(bad_body["type"], "PLAN_ERROR")

        _, state = self._get("/state/v2")
        self.assertEqual(state["status"], "error")
        self.assertIsNone(state["result"])
        self.assertIsNotNone(state["error"])

    def test_guard_released_after_validation_failure_next_request_succeeds(self):
        bad_payload = _valid_v2_payload("mission-bad")
        del bad_payload["coordinate_units"]
        bad_status, _ = self._post_v2(bad_payload)
        self.assertEqual(bad_status, 400)

        good_status, good_body = self._post_v2(_valid_v2_payload("mission-after-bad"))
        self.assertEqual(good_status, 200)
        self.assertEqual(good_body["mission_id"], "mission-after-bad")


class StrictV1Isolation(_ServerTestCase):
    def test_v2_call_never_mutates_v1_state(self):
        _, before = self._get("/state")
        self._post_v2(_valid_v2_payload())
        _, after = self._get("/state")
        self.assertEqual(before, after)

    def test_v1_call_never_mutates_v2_state(self):
        _, before = self._get("/state/v2")
        self._post_v1(_valid_v1_payload())
        _, after = self._get("/state/v2")
        self.assertEqual(before, after)

    def test_mission_id_echoed_end_to_end_over_http(self):
        status, body = self._post_v2(_valid_v2_payload("mission-http-echo"))
        self.assertEqual(status, 200)
        self.assertEqual(body["mission_id"], "mission-http-echo")
        _, state = self._get("/state/v2")
        self.assertEqual(state["mission_id"], "mission-http-echo")
        self.assertEqual(state["result"]["mission_id"], "mission-http-echo")


class NewStaticAsset(_ServerTestCase):
    """Original Roadmap Step 5 (web): `planv2.js` is the pure-function ES
    module the v2 preview UI and its Node tests both import. It must be
    servable exactly like the existing `app.js` -- same static-asset code
    path, same content type -- so the browser's `<script type="module">`
    import resolves correctly."""

    def _fetch_raw(self, path):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request("GET", path)
            response = conn.getresponse()
            body = response.read()
            return response.status, response.getheader("Content-Type") or "", body
        finally:
            conn.close()

    def test_planv2_js_is_served_with_a_javascript_content_type(self):
        status, content_type, body = self._fetch_raw("/planv2.js")
        self.assertEqual(status, 200)
        self.assertIn("javascript", content_type.lower())
        self.assertIn(b"buildPreviewSteps", body)

    def test_planv2_js_shares_app_js_content_type(self):
        _, app_js_type, _ = self._fetch_raw("/app.js")
        _, planv2_js_type, _ = self._fetch_raw("/planv2.js")
        self.assertEqual(app_js_type, planv2_js_type)


class WebEntryPoints(_ServerTestCase):
    """`/` is the automatic planner page; the manual v1/v2 debugging page
    stays reachable at `/manual.html`. Only the static-asset map changed."""

    _fetch_raw = NewStaticAsset._fetch_raw

    def test_root_and_index_serve_the_automatic_page(self):
        for path in ("/", "/index.html"):
            status, content_type, body = self._fetch_raw(path)
            self.assertEqual(status, 200, path)
            self.assertIn("text/html", content_type)
            self.assertIn(b'src="/auto.js"', body)
            self.assertNotIn(b"runB3", body)
            self.assertNotIn(b"manual.html", body)

    def test_manual_page_keeps_the_manual_interface(self):
        status, content_type, body = self._fetch_raw("/manual.html")
        self.assertEqual(status, 200)
        self.assertIn("text/html", content_type)
        self.assertIn(b'src="/app.js"', body)
        for control in (b"runB2", b"runB3", b"timeline", b"allowAlternativeViewing", b"copyActions"):
            self.assertIn(control, body)

    def test_new_modules_are_served_as_javascript(self):
        _, app_js_type, _ = self._fetch_raw("/app.js")
        for path, marker in (
            ("/auto.js", b"AutoPlanController"),
            ("/autoplan.js", b"AUTO_PLAN_IDLE_MS"),
            ("/arena.js", b"drawArenaGrid"),
        ):
            status, content_type, body = self._fetch_raw(path)
            self.assertEqual(status, 200, path)
            self.assertEqual(content_type, app_js_type, path)
            self.assertIn(marker, body)

    def test_unlisted_paths_are_still_404(self):
        for path in ("/support.js", "/tests/auto.test.js", "/manual", "/../server.py"):
            status, _, _ = self._fetch_raw(path)
            self.assertEqual(status, 404, path)


if __name__ == "__main__":
    unittest.main()
