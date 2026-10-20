"""TEST-ONLY worker for tests/test_bounded_prefix_process.py. Production code
never imports or starts it; it is reached only through the planner's
server-side `worker_command` test seam.

    python _bounded_test_worker.py real   <config.json>   the real protocol/search, with hooks
    python _bounded_test_worker.py script <config.json>   a scripted protocol conversation

`real` config (all optional):
    sleep_before_hello_s   delay before HELLO (slow start-up)
    sleep_per_call_s       delay before every real leg-planner call
    stall_at_call          sleep 10 minutes at this (1-based) leg-planner call
    crash_at_call          os._exit(7) at this leg-planner call
    stall_before_final     sleep 10 minutes after the search, before FINAL is built

`script` config: {"steps": [ {"op": ...}, ... ]}, ops listed in `run_script`.
"""

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import prefix_worker as pw  # noqa: E402
from executable_route_optimizer import plan_executable_leg  # noqa: E402

FOREVER_S = 600.0


def run_real(config, stdin, stdout):
    time.sleep(config.get("sleep_before_hello_s", 0.0))
    calls = [0]

    def leg_planner(start, target, obstacles, cfg):
        calls[0] += 1
        time.sleep(config.get("sleep_per_call_s", 0.0))
        if config.get("stall_at_call") == calls[0]:
            time.sleep(FOREVER_S)
        if config.get("crash_at_call") == calls[0]:
            os._exit(7)
        return plan_executable_leg(start, target, obstacles, cfg)

    if config.get("stall_before_final"):
        original = pw.final_payload

        def stalled(result):
            time.sleep(FOREVER_S)
            return original(result)

        pw.final_payload = stalled
    return pw.serve(stdin, stdout, leg_planner=leg_planner)


def run_script(config, stdin, stdout):
    writer = pw.FrameWriter(stdout)
    cancel = threading.Event()
    for step in config["steps"]:
        op = step["op"]
        if op == "sleep":
            time.sleep(step["s"])
        elif op == "hello":
            writer.send(pw.KIND_HELLO, {"protocol": pw.PROTOCOL_VERSION, "pid": os.getpid()})
        elif op == "read_spec":
            line = stdin.readline(pw.MAX_FRAME_BYTES + 1)
            _kind, payload = pw.decode_frame(line, 0, {pw.KIND_SPEC})
            if step.get("save"):
                Path(step["save"]).write_text(json.dumps(payload), encoding="utf-8")
            pw.start_control_thread(stdin, cancel)
        elif op == "incumbent":
            writer.send(pw.KIND_INCUMBENT, {"phase": step["phase"], "route": step["record"]})
        elif op == "final":
            writer.send(pw.KIND_FINAL, step["payload"])
        elif op == "error":
            writer.send(pw.KIND_ERROR, {"error": step["name"]})
        elif op == "frame":                       # explicit seq / kind, for protocol violations
            stdout.write(pw.encode_frame(step["seq"], step["kind"], step["payload"]))
            stdout.flush()
        elif op == "raw":
            stdout.write(bytes.fromhex(step["hex"]))
            stdout.flush()
        elif op == "oversized":
            stdout.write(b"x" * (pw.MAX_FRAME_BYTES + 10) + b"\n")
            stdout.flush()
        elif op == "print":
            print(step["text"])                   # stray stdout: main() redirected it to stderr
        elif op == "grandchild":
            child = subprocess.Popen([sys.executable, "-c", f"import time; time.sleep({FOREVER_S})"])
            Path(step["pidfile"]).write_text(str(child.pid), encoding="utf-8")
        elif op == "wait_cancel":
            cancel.wait(step.get("timeout", 30.0))
        elif op == "stall":
            time.sleep(FOREVER_S)
        elif op == "crash":
            os._exit(step.get("code", 9))
        elif op == "exit":
            return step.get("code", 0)
        else:
            raise ValueError(f"unknown op {op!r}")
    return 0


def main():
    mode, config_path = sys.argv[1], sys.argv[2]
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    sys.stdout = sys.stderr
    code = (run_real if mode == "real" else run_script)(config, stdin, stdout)
    try:
        stdout.flush()
    finally:
        os._exit(code)


if __name__ == "__main__":
    main()
