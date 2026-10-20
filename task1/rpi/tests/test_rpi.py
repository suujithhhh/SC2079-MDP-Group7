"""Hardware-free tests for the RPi Task 1 program. Run from rpi/:
    python -m unittest discover -s tests -v
"""

import argparse
import json
import math
import queue
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from align import burst, heading_fix, offset_cm, simulate  # noqa: E402
from android_link import LineFramer  # noqa: E402
from planner_client import (PlannerError, StartPose, android_robot_cell, build_plan_request,  # noqa: E402
                            heading_to_direction, parse_android_request, planner_url, request_plan)
import io  # noqa: E402

from stm_link import StmLink, corrected_turn, mov_timeout_s, summarise_motion, validate_mov  # noqa: E402
from task1 import Mission, plan_from_result  # noqa: E402
from vision import image_upload_url, pick_target, target_centre_x, upload_image  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        self.now += 0.5
        return self.now


class FakeSerial:
    """Replies to each written command with a scripted list of lines."""

    def __init__(self, script):
        self.script = dict(script)
        self.written = []
        self.pending = []

    def write(self, data):
        command = data.decode().strip()
        self.written.append(command)
        self.pending.extend(self.script.get(command, []))
        return len(data)

    def readline(self):
        return (self.pending.pop(0) + "\n").encode() if self.pending else b""

    def reset_input_buffer(self):
        pass


SETUP = {
    "set_command": "PROFILE,PRECISION", "expected_set_reply": "PROFILE,OK",
    "verify_command": "PROFILE,GET", "expected_verify_reply": "PROFILE,PRECISION",
}
PROFILE_OK = {"PROFILE,PRECISION": ["PROFILE,OK"], "PROFILE,GET": ["PROFILE,PRECISION"]}


class ImuSerial(FakeSerial):
    """FakeSerial whose IMU,GET replies come from a list of headings (deg), or
    (heading, yaw rate deg/s) pairs, in order; ERR,IMU once the list runs out."""

    def __init__(self, script, headings):
        super().__init__(script)
        self.headings = list(headings)

    def write(self, data):
        if data.decode().strip() == "IMU,GET":
            self.written.append("IMU,GET")
            heading = self.headings.pop(0) if self.headings else None
            heading, rate = heading if isinstance(heading, tuple) else (heading, 0.0)
            self.pending.append("ERR,IMU" if heading is None else
                                f"IMU,Y:{round(heading * 10)},GZ:{round(rate * 10)},SIGN:+1")
            return len(data)
        return super().write(data)


def link(script, headings=None):
    port = FakeSerial(script) if headings is None else ImuSerial(script, headings)
    return StmLink(port, log=lambda _: None, clock=FakeClock(), sleep=lambda _: None), port


class StmLinkTests(unittest.TestCase):
    def test_ack_after_telemetry(self):
        stm, _ = link({"MOV,F,30": ["MM,EV:START", "MM,MD,1", "TB,x", "ACK"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertTrue(reply.motion_started)
        self.assertEqual(reply.telemetry, ["MM,EV:START", "MM,MD,1", "TB,x"])

    def test_err_is_terminal(self):
        stm, _ = link({"MOV,FL,90": ["MM,EV:START", "ERR,MOTION_TIMEOUT", "ACK"]})
        reply = stm.move("MOV,FL,90")
        self.assertEqual((reply.status, reply.line), ("error", "ERR,MOTION_TIMEOUT"))

    def test_silence_times_out_without_resend(self):
        stm, port = link({})
        reply = stm.move("MOV,B,10")
        self.assertEqual(reply.status, "timeout")
        self.assertEqual(port.written, ["MOV,B,10"])

    def test_done_or_ok_words_are_not_terminal(self):
        stm, _ = link({"MOV,F,10": ["MM,EV:END,D:F,T:1,RESULT:OK", "DONE"]})
        self.assertEqual(stm.move("MOV,F,10").status, "timeout")

    def test_rejects_commands_the_stm_would_reject(self):
        for bad in ("MOV,L,90", "MOV,FL,361", "MOV,F,1000", "FW10", "MOV,F,10.5", "MOV,F"):
            with self.assertRaises(ValueError, msg=bad):
                validate_mov(bad)

    def test_timeout_covers_stm_deadline(self):
        self.assertGreater(mov_timeout_s("MOV,F,100"), 5 + 15)
        self.assertGreater(mov_timeout_s("MOV,FR,90"), 5 + 9)

    def test_profile_setup(self):
        stm, port = link(PROFILE_OK)
        self.assertIsNone(stm.apply_profile(SETUP))
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET"])

    def test_profile_mismatch_aborts(self):
        stm, port = link({"PROFILE,PRECISION": ["PROFILE,OK"], "PROFILE,GET": ["PROFILE,CUSTOM"]})
        self.assertIn("PROFILE,CUSTOM", stm.apply_profile(SETUP))


MM_TURN = [
    "MM,EV:START,D:FL,T:0,V:10",
    "MM,D:FL,T:50,ST:PRE,TH:1000,H:900,HE:100,GZ:0,SP:1200,LT:500,LP:0,LE:500,LO:0,RT:500,RP:0,RE:500,RO:0",
    "MM,D:FL,T:900,ST:TURN,TH:1000,H:998,HE:2,GZ:5,SP:1200,LT:500,LP:480,LE:20,LO:9,RT:900,RP:870,RE:30,RO:12",
    "MM,EV:END,D:FL,T:950,RESULT:OK",
    "ACK",
]


class TelemetryTests(unittest.TestCase):
    def test_summary(self):
        text = summarise_motion(MM_TURN[:-1])
        self.assertIn("heading 90.0 -> 99.8 (target 100.0)", text)
        self.assertIn("wheels L 480 R 870 counts", text)
        self.assertIn("950 ms OK", text)
        self.assertIsNone(summarise_motion(["AK,1"]))
        self.assertIsNone(summarise_motion(["MM,D:FL,T:5,H:x"]))

    def test_raw_log_keeps_every_line(self):
        raw, logged = io.StringIO(), []
        stm = StmLink(FakeSerial({"MOV,FL,10": MM_TURN}), log=logged.append, clock=FakeClock(), raw_log=raw)
        stm.note("mission m1")
        self.assertTrue(stm.move("MOV,FL,10").ok)
        lines = raw.getvalue().splitlines()
        self.assertEqual(lines[0], "## mission m1")
        self.assertTrue(lines[1].endswith("<- MOV,FL,10"))
        self.assertEqual([line.split(" ", 2)[2] for line in lines[2:]], MM_TURN)
        self.assertTrue(any("heading 90.0 -> 99.8" in line for line in logged))


class PlannerClientTests(unittest.TestCase):
    ANDROID = json.dumps({
        "type": "PLAN_REQUEST", "protocol_version": 1, "mode": "B2", "coordinate_units": "grid",
        "start": {"x": 0, "y": 0, "heading": "N"},
        "obstacles": [{"id": 1, "x": 6, "y": 10, "face": "N"}, {"id": 7, "x": 13, "y": 5, "face": "w"}],
    })

    def test_android_request(self):
        mode, start, obstacles = parse_android_request(self.ANDROID)
        self.assertEqual(mode, "B2")
        self.assertEqual(start, StartPose(0.0, 0.0, 90.0))   # rear axle, grid units -> cm
        self.assertEqual(obstacles[1], {"id": 7, "x": 130, "y": 50, "face": "W"})

    def test_start_is_rear_axle(self):
        for units, x, y in (("cm", 20, 10), ("grid", 2, 1)):
            line = json.dumps({"type": "PLAN_REQUEST", "coordinate_units": units,
                               "start": {"x": x, "y": y, "heading": "E"},
                               "obstacles": [{"id": 1, "x": 5, "y": 10, "face": "S"}]})
            self.assertEqual(parse_android_request(line)[1], StartPose(20.0, 10.0, 0.0), units)

    def test_bad_requests(self):
        for line in ("not json", '{"type":"X"}', '{"type":"PLAN_REQUEST","obstacles":[]}',
                     '{"type":"PLAN_REQUEST","coordinate_units":"mm"}',
                     '{"type":"PLAN_REQUEST","start":{"x":0,"y":0,"heading":"N"},"obstacles":[{"id":1,"x":1,"y":1}]}'):
            with self.assertRaises(PlannerError, msg=line):
                parse_android_request(line)

    def test_v2_request(self):
        request = build_plan_request("m1", "B2", [], StartPose(10.5, 1.0, 90.0))
        self.assertEqual((request["protocol_version"], request["coordinate_units"], request["mission_id"]),
                         (2, "cm", "m1"))
        self.assertEqual(request["start"], {"x": 10.5, "y": 1.0, "heading_deg": 90.0})

    def test_heading_to_direction(self):
        self.assertEqual([heading_to_direction(h) for h in (0, 90, 180, 270, 359, 44)],
                         ["E", "N", "W", "S", "E", "E"])

    def test_robot_cell(self):
        # Rear axle (20, 13) facing N -> body centre (20, 25) -> 3x3 box bottom-left (1, 1).
        self.assertEqual(android_robot_cell({"x_cm": 20, "y_cm": 13, "heading_deg": 90}), (1, 1, "N"))


    def test_planner_url(self):
        self.assertEqual(planner_url("10.91.1.2"), "http://10.91.1.2:5000/plan/v2")
        self.assertEqual(planner_url("10.91.1.2:5077"), "http://10.91.1.2:5077/plan/v2")
        self.assertEqual(planner_url("http://lap:5000/plan/v2"), "http://lap:5000/plan/v2")
        self.assertEqual(planner_url("http://lap:5000/plan/v2/"), "http://lap:5000/plan/v2")

    def test_planner_url_rejects_any_other_path(self):
        for bad in ("http://lap:5000/plan", "10.91.1.2:5000/plan", "http://lap/plan/v1", "http://lap:5000/health"):
            with self.assertRaises(ValueError, msg=bad):
                planner_url(bad)


ALGO_DIR = Path(__file__).resolve().parents[2] / "algo"


@unittest.skipIf(not (ALGO_DIR / "server.py").is_file(), "planner server (algo/) not available")
class PlannerServerTests(unittest.TestCase):
    """The Pi's HTTP path against the real algo/server.py on a throwaway port."""

    def test_plans_over_http(self):
        sys.path.insert(0, str(ALGO_DIR))
        import server

        httpd = server.create_server("127.0.0.1", 0, verbose=False)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        _, start, obstacles = parse_android_request(json.dumps({
            "type": "PLAN_REQUEST", "coordinate_units": "cm", "start": {"x": 20, "y": 10, "heading": "N"},
            "obstacles": [{"id": 3, "x": 50, "y": 100, "face": "S"}]}))
        url = planner_url(f"127.0.0.1:{httpd.server_address[1]}")
        plan = request_plan(url, build_plan_request("http-1", "B2", obstacles, start))
        self.assertEqual(plan["mission_id"], "http-1")
        for action in plan["actions"]:
            if action["type"] == "MOV":
                validate_mov(action["stm_command"])


class LineFramerTests(unittest.TestCase):
    def test_split_and_joined_reads(self):
        framer = LineFramer()
        self.assertEqual(framer.feed(b'{"type":"PLAN_'), [])
        self.assertEqual(framer.feed(b'REQUEST"}\r\nSTART\n\nPI'), ['{"type":"PLAN_REQUEST"}', "START"])
        self.assertEqual(framer.feed(b"NG\n"), ["PING"])


class ImageUploadTests(unittest.TestCase):
    def test_upload_url(self):
        from planner_client import images_url
        base = images_url(planner_url("10.91.1.2"))
        self.assertEqual(base, "http://10.91.1.2:5000/images")
        self.assertEqual(image_upload_url(base, "m 1", 3, "17"),
                         "http://10.91.1.2:5000/images?mission_id=m+1&obstacle_id=3&target_id=17")
        for nothing in (None, "marker", "10", "41"):     # not an NTU image ID: sent without target_id
            self.assertNotIn("target_id", image_upload_url(base, "m1", 3, nothing))

    def test_upload_posts_jpeg_and_reports_failures(self):
        import http.server
        received = {}

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                received["type"] = self.headers.get("Content-Type")
                received["body"] = self.rfile.read(int(self.headers["Content-Length"]))
                received["path"] = self.path
                status = 200 if "obstacle_id=1" in self.path else 400
                body = b'{"type":"IMAGE_STORED","image_id":"1-1"}' if status == 200 else b'{"error":"bad"}'
                self.send_response(status)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        httpd = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        base = f"http://127.0.0.1:{httpd.server_address[1]}/images"
        jpeg = b"\xff\xd8\xff" + b"x" * 100 + b"\xff\xd9"
        self.assertTrue(upload_image(image_upload_url(base, "m1", 1, "20"), jpeg).startswith("uploaded 1-1"))
        self.assertEqual((received["type"], received["body"]), ("image/jpeg", jpeg))
        self.assertTrue(upload_image(image_upload_url(base, "m1", 2, "20"), jpeg).startswith("upload rejected: HTTP 400"))
        self.assertTrue(upload_image("http://127.0.0.1:1/images?mission_id=m&obstacle_id=1", jpeg,
                                     timeout_s=1).startswith("upload failed"))


class PickTargetTests(unittest.TestCase):
    NAMES = {0: "11", 1: "20", 30: "marker"}

    def test_largest_non_bullseye(self):
        choice = pick_target(self.NAMES, [30, 0, 1], [0.99, 0.6, 0.9],
                             [[0, 0, 100, 100], [0, 0, 50, 50], [0, 0, 20, 20]])
        self.assertEqual(choice, ("11", 0.6))

    def test_only_bullseye(self):
        self.assertIsNone(pick_target(self.NAMES, [30], [0.9], [[0, 0, 10, 10]]))


class FakeAndroid:
    def __init__(self):
        self.inbox = queue.Queue()
        self.connected = threading.Event()
        self.connected.set()
        self.sent = []

    def send(self, message):
        self.sent.append(message)
        return True


class FakeRecognition:
    def __init__(self, image_id, obstacle_id=None):
        self.image_id = image_id
        self.obstacle_id = obstacle_id


class FakeVision:
    """Background recognition stand-in: a photo's result becomes available
    `lag` polls after it was submitted (0 = by the next poll)."""

    def __init__(self, ids, centres=(), lag=0):
        self.ids = dict(ids)
        self.centres = list(centres)
        self.lag = lag
        self.queued = []          # [polls still to wait, obstacle_id]
        self.submitted = []

    def centre_x(self, obstacle_id, count):
        return self.centres.pop(0) if self.centres else None

    def new_mission(self, mission_id):
        pass

    def recognise(self, obstacle_id):
        return FakeRecognition(self.ids.get(obstacle_id), obstacle_id)

    def submit(self, obstacle_id):
        self.submitted.append(obstacle_id)
        self.queued.append([self.lag, obstacle_id])

    def poll(self):
        ready = [q for q in self.queued if q[0] <= 0]
        for q in self.queued:
            q[0] -= 1
        self.queued = [q for q in self.queued if q not in ready]
        return [FakeRecognition(self.ids.get(o), o) for _, o in ready]

    def finish(self):
        done, self.queued = self.queued, []
        return [FakeRecognition(self.ids.get(o), o) for _, o in done]

    def stitch(self):
        return None


def pose(x, y, h):
    return {"x_cm": x, "y_cm": y, "heading_deg": h}


PLAN = {
    "mission_id": "m1", "stm_setup": SETUP, "start_pose": pose(25, 25, 90),
    "actions": [
        {"seq": 0, "type": "MOV", "stm_command": "MOV,F,30", "start_pose": pose(20, 10, 90),
         "end_pose": pose(20, 40, 90)},
        {"seq": 1, "type": "SNAP", "obstacle_id": 3},
        {"seq": 2, "type": "MOV", "stm_command": "MOV,FR,90", "start_pose": pose(20, 40, 90),
         "end_pose": pose(54, 74, 0)},
        {"seq": 3, "type": "SNAP", "obstacle_id": 5},
    ],
}


def mission(script, ids, centres=None, headings=None):
    args = argparse.Namespace(profile="PRECISION", heading_fix=headings is not None,
                              align=centres is not None, align_max_bursts=8,
                              align_threshold=5.0, align_turn=5, align_straight=8, align_max_forward=8.0,
                              align_distance=25.0, hfov=62.2)
    stm, port = link(script, headings)
    android = FakeAndroid()
    m = Mission(args, android, stm, FakeVision(ids, centres or ()))
    m.plan = json.loads(json.dumps(PLAN))
    return m, android, port


class MissionTests(unittest.TestCase):
    def test_recognition_runs_while_driving_and_is_all_reported_before_done(self):
        # obstacle 3's result is only ready two polls later (after the robot moved on);
        # obstacle 5 is the last action, so its result comes from finish()
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {3: "20", 5: "11"})
        m.vision.lag = 1
        m.execute()
        self.assertEqual(m.vision.submitted, [3, 5])
        self.assertEqual(port.written[2:], ["MOV,F,30", "MOV,FR,90"])   # nothing waited on the camera model
        targets = [s for s in android.sent if s.startswith("TARGET")]
        self.assertEqual(targets, ["TARGET,B3,20", "TARGET,B5,11"])
        self.assertTrue(android.sent[-1].startswith("MSG,[Mission done"))

    def test_full_run(self):
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {3: "20", 5: "11"})
        m.execute()
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET", "MOV,F,30", "MOV,FR,90"])
        self.assertIn("TARGET,B3,20", android.sent)
        self.assertIn("TARGET,B5,11", android.sent)
        self.assertTrue(android.sent[-1].startswith("MSG,[Mission done"))

    def test_stm_error_stops_and_never_resends(self):
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": ["ERR,MOTION_TIMEOUT"]}, {})
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30"])
        self.assertFalse(any(s.startswith("TARGET") for s in android.sent))
        self.assertTrue(any("position unknown" in s for s in android.sent))

    def test_profile_failure_sends_no_mov(self):
        m, _, port = mission({"PROFILE,PRECISION": ["ERR,BUSY"]}, {})
        m.execute()
        self.assertEqual(port.written, ["PROFILE,PRECISION"])

    def test_unrecognised_image_reports_msg_not_target(self):
        m, android, _ = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {5: "11"})
        m.execute()
        self.assertNotIn("TARGET,B3,None", android.sent)
        self.assertTrue(any("Obstacle 3: image not recognised" in s for s in android.sent))


# offset_cm(norm_x) at 25 cm with a 62.2 deg FOV: 0.3 -> 4.5 cm, 0.4 -> 6.0 cm, 0.5 -> 7.5 cm
RIGHT, LEFT = burst(1), burst(-1)


def align_mission(centres, extra=None):
    script = {**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"],
              **{c: ["ACK"] for c in RIGHT + LEFT}, **(extra or {})}
    return mission(script, {3: "20", 5: "11"}, centres=centres)


class AlignTests(unittest.TestCase):
    def test_burst_shifts_sideways_and_returns_to_distance(self):
        # forward first (straight trimmed from 8 to 2 cm to fit the 8 cm cap), then straight back
        self.assertEqual(RIGHT, ["MOV,FR,5", "MOV,F,2", "MOV,FL,5", "MOV,B,7"])
        self.assertEqual(LEFT, ["MOV,FL,5", "MOV,F,2", "MOV,FR,5", "MOV,B,7"])
        right, forward, heading = simulate(RIGHT)
        self.assertAlmostEqual(right, 0.40, places=2)
        self.assertLess(abs(forward), 0.5)
        self.assertAlmostEqual(heading, 0.0, places=6)
        self.assertLess(simulate(LEFT)[0], 0)
        self.assertEqual(burst(1, 5, 8, 13.5), ["MOV,FR,5", "MOV,F,8", "MOV,FL,5", "MOV,B,13"])  # room for all of it
        self.assertEqual(burst(1, 5, 8, 4.0), [])                                    # the turns alone need ~5 cm

    def test_burst_never_reverses_behind_its_start_or_past_the_cap(self):
        for cap in (6.0, 8.0, 10.0, 13.5):
            for shift in (1, -1):
                moves = burst(shift, 5, 8, cap)
                travelled = [simulate(moves[:i])[1] for i in range(1, len(moves) + 1)]
                self.assertLessEqual(max(travelled), cap)
                self.assertGreater(min(travelled), -0.5)      # ends where it started, never further back

    def test_image_offset(self):
        self.assertEqual(offset_cm(0.0), 0.0)
        self.assertGreater(offset_cm(0.2), 0)          # right of centre -> shift right
        self.assertAlmostEqual(offset_cm(1.0, 25.0, 90.0), 25.0)
        self.assertEqual(target_centre_x([0.9, 0.8], [[0, 0, 10, 10], [400, 100, 560, 300]], 640), 0.5)
        self.assertIsNone(target_centre_x([], [], 640))

    def test_within_threshold_moves_on(self):
        m, android, port = align_mission([0.3, -0.3])
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", "MOV,FR,90"])

    def test_stops_as_soon_as_within_threshold(self):
        # 7.5 -> 6.0 -> 4.5 cm: two bursts, then SNAP, then the next obstacle
        m, android, port = align_mission([0.5, 0.4, 0.3, 0.0])
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", *RIGHT, *RIGHT, "MOV,FR,90"])
        self.assertIn("TARGET,B3,20", android.sent)
        self.assertIn("TARGET,B5,11", android.sent)

    def test_gives_up_when_a_burst_does_not_get_closer(self):
        m, android, port = align_mission([-0.4, -0.5, 0.0])
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", *LEFT, "MOV,FR,90"])
        self.assertIn("TARGET,B3,20", android.sent)

    def test_burst_limit(self):
        centres = [0.9, 0.89, 0.88, 0.87, 0.0]     # creeping closer, never within 5 cm
        m, android, port = align_mission(centres)
        m.args.align_max_bursts = 3
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", *RIGHT * 3, "MOV,FR,90"])

    def test_nothing_detected_moves_on(self):
        m, android, port = align_mission([])
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", "MOV,FR,90"])

    def test_alignment_error_stops_mission(self):
        m, android, port = align_mission([0.5], {RIGHT[0]: ["ERR,MOTION_TIMEOUT"]})
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", RIGHT[0]])
        self.assertFalse(any(s.startswith("TARGET") for s in android.sent))
        self.assertTrue(any("alignment" in s and "position unknown" in s for s in android.sent))


class HeadingFixTests(unittest.TestCase):
    def test_corrected_turn(self):
        command, error = corrected_turn("MOV,FL,10", 100.0, 99.2)                       # short of a left turn
        self.assertEqual(command, "MOV,FL,11")
        self.assertAlmostEqual(error, 0.8)
        self.assertEqual(corrected_turn("MOV,BL,10", 100.0, 99.2)[0], "MOV,BL,11")
        self.assertEqual(corrected_turn("MOV,FR,10", 80.0, 80.8)[0], "MOV,FR,11")       # short of a right turn
        self.assertEqual(corrected_turn("MOV,BR,10", 80.0, 80.8)[0], "MOV,BR,11")
        self.assertEqual(corrected_turn("MOV,FL,10", 100.0, 100.6)[0], "MOV,FL,9")      # overshot
        self.assertEqual(corrected_turn("MOV,FL,10", 0.0, 359.2)[0], "MOV,FL,11")       # across 0/360
        self.assertEqual(corrected_turn("MOV,FL,10", 100.0, 80.0), ("MOV,FL,15", 5.0))  # capped
        self.assertEqual(corrected_turn("MOV,FR,2", 0.0, -5.0)[0], "MOV,FR,1")          # never below 1

    def test_heading_reading(self):
        stm, _ = link({}, [12.3, None])
        self.assertEqual(stm.heading(), 12.3)
        self.assertIsNone(stm.heading())

    def test_turn_is_corrected_from_imu(self):
        # start 0.0; at SNAP 3 it reads 0.5 (under 1 deg: no fix); before FR,90 the
        # plan expects 0.0 but the robot reads +0.7 (CCW); at SNAP 5 -90.2 (no fix)
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,91": ["ACK"]}, {3: "20", 5: "11"},
                                   headings=[0.0, 0.5, 0.7, -90.2])
        m.execute()
        self.assertEqual(port.written[2:], ["IMU,GET", "MOV,F,30", "IMU,GET", "IMU,GET", "MOV,FR,91", "IMU,GET"])
        self.assertIn("TARGET,B5,11", android.sent)

    def test_expected_heading_follows_the_plan(self):
        m, _, port = mission({**PROFILE_OK, "MOV,FL,10": ["ACK"], "MOV,FL,11": ["ACK"]}, {}, headings=[0.0, 0.0, 9.3])
        # (no SNAPs in this plan, so the readings are: start, before turn 1, before turn 2)
        m.plan["actions"] = [
            {"seq": 0, "type": "MOV", "stm_command": "MOV,FL,10", "end_pose": pose(20, 10, 100)},
            {"seq": 1, "type": "MOV", "stm_command": "MOV,FL,10", "end_pose": pose(20, 10, 110)},
        ]
        m.execute()
        self.assertEqual([c for c in port.written if c.startswith("MOV")], ["MOV,FL,10", "MOV,FL,11"])

    def test_waits_for_rotation_to_stop(self):
        stm, port = link({}, [(10.0, 15.0), (11.2, 4.0), (11.5, 0.2)])
        self.assertEqual(stm.heading(settle_s=60), 11.5)   # FakeClock ticks 0.5 s per call
        self.assertEqual(port.written, ["IMU,GET"] * 3)
        stm, port = link({}, [(10.0, 15.0)] * 50)
        self.assertEqual(stm.heading(settle_s=3), 10.0)     # gives up and returns the last reading
        self.assertLess(len(port.written), 50)

    def test_heading_fix_moves(self):
        self.assertEqual(heading_fix(0.4), [])
        self.assertEqual(heading_fix(3.0), ["MOV,FL,3"])                  # creeps 1.3 cm: no reverse
        self.assertEqual(heading_fix(-5.0), ["MOV,FR,5", "MOV,B,3"])
        self.assertEqual(heading_fix(12.0), ["MOV,FL,5", "MOV,B,2"])      # capped at 5 deg
        right, forward, turned = simulate(heading_fix(-5.0))
        self.assertAlmostEqual(turned, -5.0)
        self.assertLess(abs(forward), 0.5)

    def snap_only(self, headings, centres=None, script=None):
        m, android, port = mission({**PROFILE_OK, **(script or {})}, {3: "20"}, centres=centres, headings=headings)
        m.plan["actions"] = [{"seq": 0, "type": "SNAP", "obstacle_id": 3}]
        photo = m.vision.centre_x
        m.vision.centre_x = lambda *a: (port.written.append("PHOTO"), photo(*a))[1]
        return m, android, port

    def test_heading_is_fixed_before_the_photo(self):
        # plan expects 0.0 at the SNAP; the robot reads -3.0, then -0.4 after the fix
        m, android, port = self.snap_only([0.0, -3.0, -0.4], script={"MOV,FL,3": ["ACK"]})
        m.execute()
        self.assertEqual(port.written[2:], ["IMU,GET", "IMU,GET", "MOV,FL,3", "IMU,GET"])
        self.assertIn("TARGET,B3,20", android.sent)

    def test_heading_fix_runs_before_alignment_and_between_bursts(self):
        script = {"MOV,FR,2": ["ACK"], "MOV,FL,2": ["ACK"], **{c: ["ACK"] for c in RIGHT}}
        # heading -2 -> fixed; photo 7.5 cm right -> burst; heading +1.5 -> fixed; photo 0
        m, android, port = self.snap_only([0.0, 2.0, 0.1, -1.5, 0.2], centres=[0.5, 0.0], script=script)
        m.execute()
        self.assertEqual(port.written[2:], ["IMU,GET", "IMU,GET", "MOV,FR,2", "IMU,GET", "PHOTO", *RIGHT,
                                            "IMU,GET", "MOV,FL,2", "IMU,GET", "PHOTO"])

    def test_no_imu_sends_turns_as_planned(self):
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {3: "20", 5: "11"},
                                   headings=[])
        m.execute()
        self.assertEqual([c for c in port.written if c.startswith("MOV")], ["MOV,F,30", "MOV,FR,90"])
        self.assertTrue(android.sent[-1].startswith("MSG,[Mission done"))

    def test_off_sends_no_imu_queries(self):
        m, _, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {})
        m.execute()
        self.assertNotIn("IMU,GET", port.written)


class StartReadyTests(unittest.TestCase):
    """Plan -> READY -> wait -> START -> run (planner and STM faked)."""

    REQUEST = PlannerClientTests.ANDROID
    SCRIPT = {**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}

    def build(self, script=None, headings=None, start_on_plan=False, during_planning=(), mode="B3"):
        """A Mission whose planner call returns PLAN and first puts `during_planning`
        lines in the tablet's inbox (taps made while the Pi was still planning)."""
        args = argparse.Namespace(profile="PRECISION", heading_fix=headings is not None, align=False,
                                  planner_url="http://planner/plan/v2", planner_timeout=1.0,
                                  alt_viewing=True, start_on_plan=start_on_plan, mode=mode)
        stm, port = link(self.SCRIPT if script is None else script, headings)
        android = FakeAndroid()
        m = Mission(args, android, stm, FakeVision({3: "20", 5: "11"}))
        batches = [list(during_planning)]

        def planner(url, request, timeout):
            for line in batches.pop(0) if batches else []:
                android.inbox.put(line)
            return {**json.loads(json.dumps(PLAN)), "obstacle_order": [3, 5]}

        patcher = mock.patch("task1.request_plan", side_effect=planner)
        self.planner = patcher.start()
        self.addCleanup(patcher.stop)
        return m, android, port

    def moved(self, port):
        return [c for c in port.written if c.startswith("MOV")]

    def test_request_is_always_v2_and_b3_whatever_android_sends(self):
        # REQUEST is Android-shaped with protocol_version 1 and mode B2
        m, android, port = self.build()
        self.assertEqual(json.loads(self.REQUEST)["protocol_version"], 1)
        self.assertEqual(json.loads(self.REQUEST)["mode"], "B2")
        m.handle_line(self.REQUEST)
        url, request, timeout = self.planner.call_args[0]
        self.assertEqual((request["protocol_version"], request["mode"]), (2, "B3"))
        self.assertTrue(url.endswith("/plan/v2"))

    def test_mode_option_is_honoured_for_testing(self):
        m, android, port = self.build(mode="B2")
        m.handle_line(self.REQUEST.replace('"B2"', '"B3"'))   # Android says B3, the Pi's --mode B2 wins
        self.assertEqual(self.planner.call_args[0][1]["mode"], "B2")

    def test_ready_after_setup_and_no_motion_before_start(self):
        m, android, port = self.build()
        m.handle_line(self.REQUEST)
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET"])
        self.assertTrue(m.ready)
        self.assertEqual(android.sent[-2:], ["READY", "MSG,[Ready: 2 moves, press Start]"])

    def test_start_runs_plan_once(self):
        m, android, port = self.build()
        m.handle_line(self.REQUEST)
        m.handle_line("START")
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET", "MOV,F,30", "MOV,FR,90"])  # no second setup
        self.assertTrue(any(s.startswith("MSG,[Mission done") for s in android.sent))
        self.assertFalse(m.ready)
        m.handle_line("START")
        self.assertEqual(self.moved(port), ["MOV,F,30", "MOV,FR,90"])
        self.assertEqual(android.sent[-1], "MSG,[START ignored: no plan ready, submit the obstacle map first]")

    def test_start_is_case_insensitive_and_resume_works(self):
        m, _, port = self.build()
        m.handle_line(self.REQUEST)
        m.handle_line("start")
        self.assertEqual(self.moved(port), ["MOV,F,30", "MOV,FR,90"])

    def test_start_with_no_plan_does_nothing(self):
        m, android, port = self.build()
        m.handle_line("START")
        self.assertEqual(port.written, [])
        self.assertEqual(android.sent, ["MSG,[START ignored: no plan ready, submit the obstacle map first]"])

    def test_start_tapped_during_planning_is_ignored(self):
        m, android, port = self.build(during_planning=["START"])
        m.handle_line(self.REQUEST)
        self.assertEqual(self.moved(port), [])
        self.assertTrue(m.ready)
        self.assertEqual(android.sent[-3:], ["MSG,[START ignored: not ready yet, wait for READY]",
                                             "READY", "MSG,[Ready: 2 moves, press Start]"])
        m.handle_line("START")                       # a tap after READY does run it
        self.assertEqual(self.moved(port), ["MOV,F,30", "MOV,FR,90"])

    def test_newer_request_during_planning_replaces_the_plan(self):
        m, android, port = self.build(during_planning=[self.REQUEST, "START"])
        m.handle_line(self.REQUEST)
        self.assertEqual(self.planner.call_count, 2)
        self.assertEqual(android.sent.count("READY"), 1)
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET"])

    def test_no_ready_when_profile_setup_fails(self):
        m, android, port = self.build({"PROFILE,PRECISION": ["ERR,BUSY"]})
        m.handle_line(self.REQUEST)
        self.assertNotIn("READY", android.sent)
        self.assertTrue(any(s.startswith("MSG,[Not ready: PROFILE,PRECISION") for s in android.sent))
        self.assertFalse(m.ready)
        m.handle_line("START")
        self.assertEqual(port.written, ["PROFILE,PRECISION"])      # nothing moved, nothing retried

    def test_no_ready_without_imu_when_heading_fix_is_on(self):
        m, android, port = self.build(headings=[None])
        m.handle_line(self.REQUEST)
        self.assertNotIn("READY", android.sent)
        self.assertIn("MSG,[Not ready: no IMU heading from the STM]", android.sent)
        self.assertEqual(self.moved(port), [])

    def test_ready_with_imu_then_start_uses_a_fresh_heading(self):
        m, android, port = self.build(headings=[0.0] * 10)
        m.handle_line(self.REQUEST)
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET", "IMU,GET"])
        self.assertIn("READY", android.sent)
        m.handle_line("START")
        self.assertEqual(port.written[3], "IMU,GET")
        self.assertIn("MOV,F,30", port.written)

    def test_planner_failure_sends_no_ready(self):
        m, android, port = self.build()
        self.planner.side_effect = PlannerError("no route")
        m.handle_line(self.REQUEST)
        self.assertNotIn("READY", android.sent)
        self.assertIn("MSG,[Plan failed: no route]", android.sent)
        self.assertEqual(port.written, [])

    def test_reconnect_repeats_ready_only_while_armed(self):
        m, android, _ = self.build()
        m.handle_line("__CONNECTED__")
        self.assertEqual(android.sent, ["MSG,[Pi connected: send the obstacle map]"])
        m.handle_line(self.REQUEST)
        android.sent.clear()
        m.handle_line("__CONNECTED__")
        self.assertEqual(android.sent, ["READY", "MSG,[Ready: 2 moves, press Start]"])
        m.handle_line("START")
        android.sent.clear()
        m.handle_line("__CONNECTED__")
        self.assertEqual(android.sent, ["MSG,[Pi connected: send the obstacle map]"])

    def test_resubmitting_replaces_the_armed_plan(self):
        m, android, port = self.build()
        m.handle_line(self.REQUEST)
        m.handle_line(self.REQUEST)
        self.assertEqual(self.planner.call_count, 2)
        self.assertEqual(android.sent.count("READY"), 2)
        self.assertEqual(self.moved(port), [])

    def test_start_on_plan_runs_without_start(self):
        m, android, port = self.build(start_on_plan=True)
        m.handle_line(self.REQUEST)
        self.assertEqual(port.written, ["PROFILE,PRECISION", "PROFILE,GET", "MOV,F,30", "MOV,FR,90"])
        self.assertIn("READY", android.sent)

    def test_paused_mission_repeats_ready_on_reconnect_and_resumes_on_start(self):
        m, android, _ = self.build()
        for line in ("__CONNECTED__", "START"):
            android.inbox.put(line)
        m.wait_for_resume()
        self.assertEqual(android.sent, ["READY", "MSG,[Mission paused: send START to resume]", "MSG,[Resuming mission]"])


class PushedPlanTests(unittest.TestCase):
    RESULT = {"type": "PLAN_RESULT", "valid": True, "mission_id": "web-1", "stm_setup": SETUP,
              "actions": [{"seq": 0, "type": "MOV", "stm_command": "MOV,F,30",
                           "start_pose": pose(20, 10, 90), "end_pose": pose(20, 40, 90)},
                          {"seq": 1, "type": "SNAP", "obstacle_id": 1, "pose": pose(20, 40, 90)}]}

    def test_accepts_v2_result(self):
        plan = plan_from_result(json.loads(json.dumps(self.RESULT)))
        self.assertEqual((plan["mission_id"], plan["start_pose"]), ("web-1", pose(20, 10, 90)))

    def test_rejects_unsafe_or_wrong_results(self):
        bad_mov = json.loads(json.dumps(self.RESULT))
        bad_mov["actions"][0]["stm_command"] = "MOV,L,90"
        v1 = {"type": "PLAN_RESULT", "valid": True, "commands": [{"type": "FW", "value": 10}]}
        for result in (bad_mov, v1, {"type": "PLAN_RESULT", "valid": False}, []):
            with self.assertRaises(ValueError):
                plan_from_result(result)


if __name__ == "__main__":
    unittest.main()
