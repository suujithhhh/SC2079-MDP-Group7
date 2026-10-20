"""Planner-version indicator support: `GET /state/versions`.

A real `PlanningHTTPServer` runs on an ephemeral port with fake planners (no
real planning happens here). The endpoint is display only: it reports which
protocol version the newest plan request used (`POST /plan` is v1, `POST
/plan/v2` is v2 -- the URL decides, not the `protocol_version` field), its
mission id and its sender. These tests also check that a request the server
rejects at the transport level is not recorded, and that the endpoint never
touches the planner state.
"""

import http.client
import json
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import server as server_module  # noqa: E402

HOST = "127.0.0.1"
PLAN_RESULT = {"type": "PLAN_RESULT", "valid": False}


def fake_planner(_payload):
    return dict(PLAN_RESULT)


class _ServerTestCase(unittest.TestCase):
    def setUp(self):
        self.server = server_module.create_server(
            host=HOST, port=0, planner=fake_planner, verbose=False, planner_v2=fake_planner
        )
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request(method, path, body=body, headers=headers or {})
            response = conn.getresponse()
            return response.status, response.read()
        finally:
            conn.close()

    def post_plan(self, path, payload, user_agent=None, content_type="application/json"):
        body = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": content_type, "Content-Length": str(len(body))}
        if user_agent is not None:
            headers["User-Agent"] = user_agent
        return self.request("POST", path, body=body, headers=headers)

    def versions(self):
        status, raw = self.request("GET", "/state/versions")
        self.assertEqual(status, 200)
        body = json.loads(raw)
        self.assertEqual(body["type"], "PLAN_VERSIONS")
        return body["last_request"]


class StateVersions(_ServerTestCase):
    def test_no_request_yet_is_null(self):
        self.assertIsNone(self.versions())

    def test_a_v2_request_is_recorded_with_its_mission_id(self):
        self.post_plan("/plan/v2", {"protocol_version": 2, "mission_id": "task1-run-001"},
                       user_agent="python-requests/2.31")
        last = self.versions()
        self.assertEqual(last["protocol_version"], 2)
        self.assertEqual(last["mission_id"], "task1-run-001")
        self.assertEqual(last["source"], "rpi")
        self.assertGreaterEqual(last["age_s"], 0)
        self.assertLess(last["age_s"], 5)
        self.assertEqual(set(last), {"protocol_version", "mission_id", "mode", "source", "age_s"})

    def test_the_requested_mode_is_recorded_for_v1_and_v2(self):
        self.post_plan("/plan/v2", {"protocol_version": 2, "mission_id": "m", "mode": "B3"})
        self.assertEqual(self.versions()["mode"], "B3")
        self.post_plan("/plan/v2", {"protocol_version": 2, "mission_id": "m", "mode": "B2"})
        self.assertEqual(self.versions()["mode"], "B2")
        self.post_plan("/plan", {"protocol_version": 1, "mode": "B3"})
        last = self.versions()
        self.assertEqual((last["protocol_version"], last["mode"]), (1, "B3"))

    def test_a_missing_or_non_string_mode_is_null_and_a_long_one_is_capped(self):
        for odd in (None, 3, ["B3"], {"a": 1}, ""):
            with self.subTest(mode=odd):
                self.post_plan("/plan/v2", {"mission_id": "m", "mode": odd})
                self.assertIsNone(self.versions()["mode"])
        self.post_plan("/plan/v2", {"mission_id": "m"})
        self.assertIsNone(self.versions()["mode"])
        self.post_plan("/plan/v2", {"mission_id": "m", "mode": "B" * 500})
        self.assertEqual(self.versions()["mode"], "B" * 16)

    def test_a_v1_request_is_recorded_as_v1_with_no_mission_id(self):
        self.post_plan("/plan", {"protocol_version": 1, "mission_id": "ignored"}, user_agent="curl/8.5")
        last = self.versions()
        self.assertEqual(last["protocol_version"], 1)
        self.assertIsNone(last["mission_id"])
        self.assertEqual(last["source"], "rpi")

    def test_the_url_decides_the_version_not_the_protocol_version_field(self):
        # A v2-shaped body sent to /plan is a v1 request (the planner rejects it);
        # a v1-shaped body sent to /plan/v2 is a v2 request.
        self.post_plan("/plan", {"protocol_version": 2, "mission_id": "m"})
        self.assertEqual(self.versions()["protocol_version"], 1)
        self.post_plan("/plan/v2", {"protocol_version": 1})
        last = self.versions()
        self.assertEqual(last["protocol_version"], 2)
        self.assertIsNone(last["mission_id"])

    def test_the_newest_request_wins(self):
        self.post_plan("/plan/v2", {"mission_id": "first"})
        self.post_plan("/plan", {})
        self.assertEqual(self.versions()["protocol_version"], 1)
        self.post_plan("/plan/v2", {"mission_id": "third"})
        last = self.versions()
        self.assertEqual((last["protocol_version"], last["mission_id"]), (2, "third"))

    def test_a_browser_user_agent_is_labelled_browser(self):
        self.post_plan("/plan/v2", {"mission_id": "web-preview-1"},
                       user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) Chrome/130.0")
        self.assertEqual(self.versions()["source"], "browser")

    def test_a_non_string_or_empty_mission_id_is_null_and_a_long_one_is_capped(self):
        for odd in (None, 5, ["m"], {"a": 1}, ""):
            with self.subTest(mission_id=odd):
                self.post_plan("/plan/v2", {"mission_id": odd})
                self.assertIsNone(self.versions()["mission_id"])
        self.post_plan("/plan/v2", {"mission_id": "x" * 5000})
        self.assertEqual(self.versions()["mission_id"], "x" * 128)

    def test_a_body_that_is_not_json_is_not_recorded(self):
        status, _ = self.request("POST", "/plan/v2", body=b"not json",
                                 headers={"Content-Type": "application/json", "Content-Length": "8"})
        self.assertEqual(status, 400)
        status, _ = self.post_plan("/plan", {}, content_type="text/plain")
        self.assertGreaterEqual(status, 400)
        self.assertIsNone(self.versions())

    def test_reading_it_never_changes_the_planner_state(self):
        before = self.server.get_v2_state()
        for _ in range(3):
            self.versions()
        self.assertEqual(self.server.get_v2_state(), before)
        self.assertIsNone(self.server.latest_request)

    def test_the_new_script_is_served(self):
        status, raw = self.request("GET", "/versions.js")
        self.assertEqual(status, 200)
        self.assertIn(b"versionIndicator", raw)


if __name__ == "__main__":
    unittest.main()
