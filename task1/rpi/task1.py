#!/usr/bin/env python3
"""Task 1 orchestrator: Android -> RPi -> planner -> STM, with recognition on SNAP.

Flow (PROTOCOL.md §4-§7, planner `POST /plan/v2`):
  1. Android sends its PLAN_REQUEST JSON line (Submit).
  2. The Pi requests a route from the planner server on the laptop
     (`algo/server.py`); the Pi itself never plans.
  3. The Pi gets ready: it runs the plan's `stm_setup` (PROFILE set + verify)
     and checks the IMU, then sends Android `READY` (plus a MSG line) and
     waits. The robot does not move until Android sends `START`. A `START`
     that arrives before `READY` (Android shows its Start button right after
     Submit) is ignored with a MSG reply. --start-on-plan skips the wait and
     moves as soon as the plan is ready.
  4. On `START` the Pi runs each action in order:
     MOV -> send `stm_command` unchanged and wait for ACK/ERR;
     SNAP -> first turn back to the plan's heading using the IMU (a 1-5 deg
     forward turn, then capture one frame and hand it to the vision worker
     process; the robot drives on while the model runs, and each TARGET is
     sent to Android as soon as its result is ready (all are collected before
     "Mission done"). With --align it first shuffles
     sideways in short bursts while the obstacle is more than 10 cm off
     centre (align.py); each look is another model run, so it is off by default.
     Before each turn the Pi reads the IMU heading (`IMU,GET`) and adjusts
     the turn by a degree or two so it ends at the plan's heading; otherwise
     the STM's small per-turn shortfall adds up (--no-heading-fix turns it off).
  5. After the last action the robot stops and the stitched image is saved.

Failure policy: any STM error or timeout ends the mission with the robot's
pose marked unknown; a command is never resent automatically. If Android
disconnects, the current command finishes and the mission pauses until the
tablet reconnects and sends `START` (or `RESUME`) again; `READY` is sent again
on reconnect.
"""

from __future__ import annotations

import argparse
import queue
import signal
import time
import uuid
from pathlib import Path

from align import burst, heading_fix, offset_cm
from android_link import AndroidLink
from planner_client import (PlannerError, android_robot_cell, build_plan_request,
                            images_url, parse_android_request, planner_url, request_plan)
from stm_link import TURN_SIGN, StmLink, corrected_turn, open_serial, validate_mov
from uart import UartOpenError

START_WORDS = {"START", "ALG:START", "RESUME"}


class Mission:
    def __init__(self, args: argparse.Namespace, android: AndroidLink, stm: StmLink, vision) -> None:
        self.args = args
        self.android = android
        self.stm = stm
        self.vision = vision
        self.plan: dict | None = None
        self.expected: float | None = None   # IMU heading the plan expects now (heading fix)
        self.ready = False                   # plan armed: STM set up, READY sent, waiting for START

    # -- Android messages ------------------------------------------------
    def msg(self, text: str) -> None:
        self.android.send(f"MSG,[{text}]")

    def robot(self, pose: dict) -> None:
        x, y, direction = android_robot_cell(pose)
        self.android.send(f"ROBOT,{x},{y},{direction}")

    def send_ready(self) -> None:
        """READY is the bare line the tablet can act on; the MSG line shows in its log."""
        moves = sum(1 for a in self.plan["actions"] if a["type"] == "MOV")
        self.android.send("READY")
        self.msg(f"Ready: {moves} moves, press Start")

    def target(self, obstacle_id: int, image_id: str) -> None:
        self.android.send(f"TARGET,B{obstacle_id},{image_id}")

    def report(self, recognitions) -> None:
        """Send finished recognitions (they arrive from the vision worker while
        the robot is already driving to the next obstacle)."""
        for result in recognitions:
            if getattr(result, "upload", None):
                print(f"Obstacle {result.obstacle_id}: photo {result.upload}")
            if result.image_id is not None:
                self.target(result.obstacle_id, result.image_id)
            else:
                self.msg(f"Obstacle {result.obstacle_id}: image not recognised")

    # -- Main loop -------------------------------------------------------
    def run_forever(self) -> None:
        while True:
            self.handle_line(self.android.inbox.get())

    def handle_line(self, line: str) -> None:
        if line == "__CONNECTED__":
            if self.ready:
                self.send_ready()   # the earlier READY went nowhere if the tablet was away
            else:
                self.msg("Pi connected: send the obstacle map")
        elif line.startswith("{"):
            self.handle_plan_request(line)
        elif line.upper() in START_WORDS:
            if self.plan is None or not self.ready:
                self.msg("START ignored: no plan ready, submit the obstacle map first")
            else:
                self.execute()
        elif line != "__DISCONNECTED__":
            self.msg(f"Ignored: {line[:40]}")

    def handle_plan_request(self, line: str | None) -> None:
        while line is not None:       # a newer request that arrived while planning replaces this one
            line = self.plan_and_arm(line)

    def plan_and_arm(self, line: str) -> str | None:
        """Plan, set up the STM and send READY. Returns a newer PLAN_REQUEST that
        arrived meanwhile (to be planned instead), otherwise None."""
        self.plan = None
        self.ready = False
        try:
            android_mode, start, obstacles = parse_android_request(line)
            mode = self.args.mode      # the Pi's own mode (B3 by default), never the tablet's
            if android_mode.upper() != mode:
                print(f"Android asked for mode {android_mode}; planning with {mode} (--mode)")
            mission_id = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
            request = build_plan_request(mission_id, mode, obstacles, start, self.args.profile,
                                         self.args.alt_viewing)
            self.msg(f"Planning route for {len(obstacles)} obstacles")
            started = time.monotonic()
            plan = request_plan(self.args.planner_url, request, self.args.planner_timeout)
        except PlannerError as exc:
            self.msg(f"Plan failed: {exc}"[:120])
            return self.drain_pending()
        plan["start_pose"] = {"x_cm": start.x_cm, "y_cm": start.y_cm, "heading_deg": start.heading_deg}
        moves = sum(1 for a in plan["actions"] if a["type"] == "MOV")
        print(f"Plan {plan['mission_id']}: {moves} moves, order {plan['obstacle_order']}, "
              f"{time.monotonic() - started:.1f}s")
        if plan.get("skipped_obstacle_ids"):
            self.msg(f"Planner skipped obstacles {plan['skipped_obstacle_ids']}")
        newer = self.drain_pending()
        if newer:
            return newer
        self.plan = plan
        if not self.prepare():
            self.plan = None
            return self.drain_pending()
        newer = self.drain_pending()   # STARTs tapped during setup came before READY too
        if newer:
            self.plan, self.ready = None, False
            return newer
        self.send_ready()
        if self.args.start_on_plan:
            self.execute()
        return None

    def prepare(self) -> bool:
        """Set up the STM for the plan so that START can move the robot at once:
        motion profile set and verified, and (with the heading fix on) an IMU
        reading. On success the plan is armed (`self.ready`); on failure the
        tablet is told why and nothing is armed. Nothing here moves the robot."""
        reason = self.setup_stm(self.plan)
        if not reason and self.args.heading_fix and self.stm.heading() is None:
            reason = "no IMU heading from the STM"
        if reason:
            self.msg(f"Not ready: {reason}"[:120])
            return False
        self.ready = True
        return True

    def setup_stm(self, plan: dict) -> str | None:
        self.stm.note(f"mission {plan['mission_id']}")
        return self.stm.apply_profile(plan["stm_setup"])

    def drain_pending(self) -> str | None:
        """Handle what the tablet sent while the Pi was busy planning or setting up.
        Every START/RESUME is answered "ignored" (it came before READY, and Android
        shows Start right after Submit). Returns the newest PLAN_REQUEST, if any."""
        newest = None
        while True:
            try:
                line = self.android.inbox.get_nowait()
            except queue.Empty:
                return newest
            if line.startswith("{"):
                newest = line
            elif line.upper() in START_WORDS:
                self.msg("START ignored: not ready yet, wait for READY")

    def wait_for_resume(self) -> None:
        """Android dropped mid-mission: hold until it reconnects and says START."""
        print("Android disconnected: mission paused after the current command")
        while True:
            line = self.android.inbox.get()
            if line.upper() in START_WORDS:
                self.msg("Resuming mission")
                return
            if line == "__CONNECTED__":
                self.android.send("READY")
                self.msg("Mission paused: send START to resume")

    def drain_inbox(self) -> None:
        """Handle messages that arrived while a command was running."""
        while True:
            try:
                line = self.android.inbox.get_nowait()
            except queue.Empty:
                return
            if line == "__DISCONNECTED__":
                self.wait_for_resume()

    def fix_heading(self, obstacle_id: int) -> str | None:
        """Before a photo: turn back to the plan's heading if the robot is 1 deg
        or more off it (two tries). Returns the reason to stop the mission, or None."""
        if self.expected is None:
            return None
        for _ in range(2):
            actual = self.stm.heading()
            if actual is None:
                return None
            error = (self.expected - actual + 180.0) % 360.0 - 180.0
            moves = heading_fix(error) if abs(error) >= 1.0 else []
            if not moves:
                return None
            print(f"Heading at obstacle {obstacle_id}: {error:+.1f} deg off plan, fixing with {moves}")
            for command in moves:
                reply = self.stm.move(command)
                if not reply.ok:
                    return f"heading fix {command}: {reply.line or 'no reply'}"
        return None

    def align(self, obstacle_id: int) -> str | None:
        """Before a SNAP: while the obstacle is more than --align-threshold cm
        off centre, shuffle one burst towards it and look again. Moves on to
        the SNAP as soon as it is within the threshold, when nothing is seen,
        when a burst did not bring it closer, or after --align-max-bursts.
        Returns the reason to stop the mission, or None."""
        previous = None
        for count in range(self.args.align_max_bursts + 1):
            if count > 0:
                reason = self.fix_heading(obstacle_id)   # bursts leave a little yaw behind
                if reason:
                    return reason
            norm_x = self.vision.centre_x(obstacle_id, count)
            if norm_x is None:
                print(f"Align obstacle {obstacle_id}: nothing detected, moving on")
                return None
            shift = offset_cm(norm_x, self.args.align_distance, self.args.hfov)
            print(f"Align obstacle {obstacle_id}: {shift:+.1f} cm off centre after {count} bursts")
            if abs(shift) <= self.args.align_threshold:
                return None
            if previous is not None and abs(shift) >= abs(previous):
                print(f"Align obstacle {obstacle_id}: last burst did not get closer "
                      f"({previous:+.1f} -> {shift:+.1f} cm), moving on")
                return None
            if count == self.args.align_max_bursts:
                print(f"Align obstacle {obstacle_id}: still off after {count} bursts, moving on")
                return None
            previous = shift
            moves = burst(shift, self.args.align_turn, self.args.align_straight, self.args.align_max_forward)
            if not moves:
                print(f"Align obstacle {obstacle_id}: a {self.args.align_turn} deg burst does not fit in "
                      f"--align-max-forward {self.args.align_max_forward:g} cm, moving on")
                return None
            for command in moves:
                reply = self.stm.move(command)
                if not reply.ok:
                    return f"alignment {command}: {reply.line or 'no reply'}"
        return None

    def execute(self) -> None:
        plan, self.plan = self.plan, None   # one run per plan; resubmit to rerun
        armed, self.ready = self.ready, False
        self.vision.new_mission(plan["mission_id"])
        if not armed:   # not set up by prepare() (--listen, or called directly)
            reason = self.setup_stm(plan)
            if reason:
                self.msg(f"Aborted before moving: {reason}")
                return
        self.msg("Mission started")
        started = time.monotonic()
        self.robot(plan["start_pose"])
        # IMU heading the plan expects; alignment bursts leave it unchanged.
        expected = self.expected = self.stm.heading() if self.args.heading_fix else None
        if self.args.heading_fix and expected is None:
            print("Heading fix off: no IMU heading from the STM")

        completed = False
        for action in plan["actions"]:
            if not self.android.connected.is_set():
                self.wait_for_resume()
            self.drain_inbox()
            self.report(self.vision.poll())
            if action["type"] == "MOV":
                command = planned = action["stm_command"]
                _, direction, value = planned.split(",")
                is_turn = direction in TURN_SIGN
                if is_turn and expected is not None:
                    actual = self.stm.heading()
                    if actual is None:
                        print(f"Heading fix skipped for {planned}: no IMU reading")
                    else:
                        command, error = corrected_turn(planned, expected, actual)
                        print(f"Heading fix: robot {error:+.1f} deg from plan, {planned} -> {command}")
                reply = self.stm.move(command)
                if not reply.ok:
                    self.msg(f"Stopped at seq {action['seq']} {command}: "
                             f"{reply.line or 'no reply'}; robot position unknown")
                    break
                if is_turn and expected is not None:
                    expected = self.expected = expected + TURN_SIGN[direction] * int(value)
                self.robot(action["end_pose"])
            elif action["type"] == "SNAP":
                reason = self.fix_heading(action["obstacle_id"])
                if not reason and self.args.align:
                    reason = self.align(action["obstacle_id"])
                if reason:
                    self.msg(f"Stopped at seq {action['seq']} {reason}; robot position unknown")
                    break
                self.vision.submit(action["obstacle_id"])   # photo now, model runs while we drive on
                print(f"Obstacle {action['obstacle_id']}: photo taken, recognising in the background")
        else:
            completed = True

        self.report(self.vision.finish())
        if completed:
            self.msg(f"Mission done in {time.monotonic() - started:.0f}s")

        stitched = self.vision.stitch()
        if stitched:
            print(f"Stitched image: {stitched}")
            self.msg(f"Stitched image saved: {stitched.name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--planner", "--planner-url", dest="planner_url", metavar="LAPTOP",
                        help="laptop running algo/server.py: IP, IP:port or full /plan/v2 URL "
                             "(port 5000 by default); required unless --listen")
    parser.add_argument("--no-alt-viewing", dest="alt_viewing", action="store_false",
                        help="require the exact viewing pose (faster, but often finds no route)")
    parser.add_argument("--planner-timeout", type=float, default=75.0,
                        help="seconds; the server answers within its own 70 s ceiling (5 s margin)")
    parser.add_argument("--no-heading-fix", dest="heading_fix", action="store_false",
                        help="send turns exactly as planned instead of correcting them to the "
                             "plan's heading with IMU,GET")
    parser.add_argument("--align", action="store_true",
                        help="shuffle sideways before each SNAP until the obstacle is centred "
                             "(one extra model run per look; off so each obstacle costs one run)")
    parser.add_argument("--align-threshold", type=float, default=10.0,
                        help="only correct when the obstacle is more than this many cm off centre")
    parser.add_argument("--align-turn", type=int, default=5,
                        help="degrees of each turn in a shuffle burst")
    parser.add_argument("--align-straight", type=int, default=8,
                        help="cm of straight between the two turns, at most; shortened to fit --align-max-forward")
    parser.add_argument("--align-max-forward", type=float, default=8.0,
                        help="cm a burst may drive towards the obstacle before reversing back (the bumper is "
                             "~16 cm from the face at the 25 cm standoff, ~11.6 cm at the closest viewing poses)")
    parser.add_argument("--align-max-bursts", type=int, default=8,
                        help="most shuffle bursts per obstacle before snapping anyway")
    parser.add_argument("--align-distance", type=float, default=25.0,
                        help="camera to obstacle face at SNAP, cm (the planner's standoff)")
    parser.add_argument("--hfov", type=float, default=62.2,
                        help="camera horizontal field of view, degrees (Pi Camera v2: 62.2)")
    parser.add_argument("--stm", default="/dev/ttyACM0",
                        help="STM serial device, 115200 8N1. /dev/ttyACM0 can change when the board is "
                             "replugged; on the Pi prefer the stable /dev/serial/by-id/... path "
                             "(list it with: ls -l /dev/serial/by-id/)")
    parser.add_argument("--bt-channel", type=int, default=1)
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parent / "mdp_circle_v2_ncnn_model")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--out", type=Path, default=Path.home() / "mdp/task1_runs")
    parser.add_argument("--profile", choices=("PRECISION", "STANDARD"), default="PRECISION")
    parser.add_argument("--mode", choices=("B2", "B3"), default="B3",
                        help="planner mode for every request; the mode Android sends is ignored "
                             "(B3 is the team default; B2 is only for testing)")
    parser.add_argument("--no-image-upload", dest="upload_images", action="store_false",
                        help="do not send each annotated photo to the laptop's POST /images "
                             "(PROTOCOL.md 7.1; on by default when --planner is given)")
    parser.add_argument("--start-on-plan", action="store_true",
                        help="move as soon as the plan is ready instead of waiting for START from "
                             "Android (the default is to send READY and wait for START)")
    parser.add_argument("--wait-for-start", action="store_true", help=argparse.SUPPRESS)  # old default, now a no-op
    parser.add_argument("--map", type=Path,
                        help="bench test without the tablet: read an Android-format PLAN_REQUEST "
                             "JSON file, plan, ask for Enter, run once and exit (Bluetooth unused)")
    parser.add_argument("--listen", type=int, metavar="PORT",
                        help="wait for plans pushed by the laptop planner (POST /run) and run each "
                             "one as soon as it arrives (Bluetooth unused)")
    args = parser.parse_args()
    if args.planner_url:
        try:
            args.planner_url = planner_url(args.planner_url)
        except ValueError as exc:
            parser.error(str(exc))
    elif not args.listen:
        parser.error("--planner <laptop-ip> is required: planning runs on the laptop (algo/server.py)")
    return args


class ConsoleLink:
    """Stands in for AndroidLink in --map mode: prints what would go to the tablet."""

    def __init__(self) -> None:
        import threading

        self.inbox: queue.Queue[str] = queue.Queue()
        self.connected = threading.Event()
        self.connected.set()

    def send(self, message: str) -> bool:
        print(f"[to tablet] {message}", flush=True)
        return True


def run_map(args: argparse.Namespace, stm: StmLink, vision) -> None:
    mission = Mission(args, ConsoleLink(), stm, vision)
    mission.handle_plan_request(" ".join(args.map.read_text().split()))
    if mission.plan is None:
        return
    for action in mission.plan["actions"]:
        print(f"  {action['seq']:3d}  {action.get('stm_command') or 'SNAP obstacle ' + str(action['obstacle_id'])}")
    try:
        input("Robot at the start pose and area clear? Press Enter to run (Ctrl+C to cancel) ")
    except EOFError:
        print("\nNo input; cancelled before moving")
        return
    mission.execute()


def plan_from_result(result: dict) -> dict:
    """Validate a PLAN_RESULT pushed by the laptop and shape it for Mission.execute."""
    if not isinstance(result, dict) or result.get("type") != "PLAN_RESULT" or not result.get("valid"):
        raise ValueError("not a valid PLAN_RESULT")
    actions, setup = result.get("actions"), result.get("stm_setup")
    if not isinstance(actions, list) or not actions or not isinstance(setup, dict):
        raise ValueError("PLAN_RESULT needs actions and stm_setup (planner /plan/v2)")
    for action in actions:
        if action.get("type") == "MOV":
            validate_mov(action.get("stm_command", ""))
        elif action.get("type") != "SNAP":
            raise ValueError(f"unknown action type {action.get('type')!r}")
    first = actions[0]
    start = first.get("start_pose") or first.get("pose")
    return {"mission_id": str(result.get("mission_id") or time.strftime("%Y%m%d-%H%M%S")),
            "actions": actions, "stm_setup": setup, "start_pose": start}


def run_listen(args: argparse.Namespace, stm: StmLink, vision) -> None:
    """HTTP endpoint for plans pushed from the laptop: POST /run, GET /status."""
    import json
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    mission = Mission(args, ConsoleLink(), stm, vision)
    busy = threading.Lock()
    status = {"state": "idle", "mission_id": None}

    def execute(plan: dict) -> None:
        try:
            mission.plan = plan
            mission.execute()
        finally:
            status["state"] = "idle"
            busy.release()
            print("Waiting for the next plan", flush=True)

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code: int, body: dict) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            self.reply(200, status) if self.path == "/status" else self.reply(404, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/run":
                return self.reply(404, {"error": "not found"})
            length = int(self.headers.get("Content-Length") or 0)
            if not 0 < length <= 1_000_000:
                return self.reply(400, {"error": "body must be 1 byte to 1 MB of JSON"})
            try:
                plan = plan_from_result(json.loads(self.rfile.read(length)))
            except (ValueError, json.JSONDecodeError) as exc:
                return self.reply(400, {"error": str(exc)})
            if not busy.acquire(blocking=False):
                return self.reply(409, {"error": "robot busy", **status})
            status.update(state="running", mission_id=plan["mission_id"])
            moves = [a["stm_command"] for a in plan["actions"] if a["type"] == "MOV"]
            print(f"Plan {plan['mission_id']} received from {self.client_address[0]}: {moves}", flush=True)
            threading.Thread(target=execute, args=(plan,), daemon=True).start()
            self.reply(202, {"accepted": plan["mission_id"], "moves": len(moves)})

        def log_message(self, format: str, *args: object) -> None:
            return

    server = ThreadingHTTPServer(("0.0.0.0", args.listen), Handler)
    print(f"Listening for plans on port {args.listen} (POST /run). The robot moves as soon as one "
          "arrives; Ctrl+C stops the program, but a move already sent still finishes.", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def main() -> None:
    args = parse_args()
    from vision import Vision

    def on_sigterm(*_: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    upload_to = images_url(args.planner_url) if args.planner_url and args.upload_images else None
    vision = Vision(args.model, args.out, imgsz=args.imgsz, conf=args.conf, images_url=upload_to)
    try:
        port = open_serial(args.stm)
    except UartOpenError as exc:
        vision.close()
        raise SystemExit(f"Cannot open the STM serial port {args.stm}: {exc}")
    args.out.mkdir(parents=True, exist_ok=True)
    raw_log = open(args.out / "stm_telemetry.log", "a")
    stm = StmLink(port, raw_log=raw_log)
    android = None
    try:
        if args.listen:
            run_listen(args, stm, vision)
        elif args.map:
            args.start_on_plan = False
            run_map(args, stm, vision)
        else:
            android = AndroidLink(args.bt_channel)
            android.start()
            Mission(args, android, stm, vision).run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if android is not None:
            android.stop()
        vision.close()
        port.close()
        raw_log.close()


if __name__ == "__main__":
    main()
