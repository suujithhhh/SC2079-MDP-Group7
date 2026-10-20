"""Original Roadmap Step 5 -- manual smoke client for the experimental
protocol-v2 planning endpoint (`POST /plan/v2`).

Kept as a separate script from `rpi_test_client.py` so that file stays at
zero diff: v1's request/response shape (`heading` strings/radians, float
`commands[].value`) is fundamentally different from v2's (required
`mission_id`, `start.heading_deg`, integer STM `actions[]`, `heading_deg`
poses everywhere), and mixing both behind one CLI would risk exactly the
kind of v1/v2 conflation this roadmap step is built to avoid.

This client supplies a fixed, deterministic `mission_id` (never generated
server-side, per `PROTOCOL.md` §6) and defaults to a small, one-obstacle
B3 request (about a minute: B3 uses the whole search budget; `--mode B2` is
fast); a custom v2-shaped JSON request can be supplied
via `--map` for manual experimentation (including the honest,
one-obstacle-at-a-time timing checks this module's own docstring calls
for -- see `planning_service_v2.py` for the measured/unmeasured obstacle
counts).

`--motion-profile {PRECISION,STANDARD}` sets the request's optional
`motion_profile`; omitted, the request is sent as-is (the default payload
has no such field, so the server resolves PRECISION). Either way the
client checks that every `PLAN_RESULT` echoes the profile that was actually
sent and carries the exact `stm_setup` strings of `PROTOCOL.md` §4.1, and
exits 1 on a mismatch. The strings are spelled out here independently of
`planning_service_v2.py` on purpose: this is a black-box HTTP check, not a
second consumer of the server's own helper. The real STM `PROFILE,*`
exchange is the RPi's job and is not exercised here.
"""

import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

DEFAULT_MISSION_ID = "smoke-test-v2-1"
MOTION_PROFILES = ("PRECISION", "STANDARD")
DEFAULT_MOTION_PROFILE = "PRECISION"

DEFAULT_PAYLOAD = {
    "type": "PLAN_REQUEST",
    "protocol_version": 2,
    "mission_id": DEFAULT_MISSION_ID,
    "mode": "B3",
    "coordinate_units": "cm",
    "allow_alternative_viewing": False,
    "start": {"x": 100, "y": 100, "heading_deg": 0},
    "obstacles": [{"id": 1, "x": 150, "y": 100, "face": "W"}],
}


def expected_stm_setup(profile: str) -> dict[str, str]:
    """The `stm_setup` object `PROTOCOL.md` §4.1 requires for `profile`."""
    return {
        "set_command": f"PROFILE,{profile}",
        "expected_set_reply": "PROFILE,OK",
        "verify_command": "PROFILE,GET",
        "expected_verify_reply": f"PROFILE,{profile}",
    }


def profile_echo_error(response: dict[str, Any], sent_profile: str) -> str | None:
    """Why `response` fails the motion-profile echo check, or None if it passes.

    Applies to every `PLAN_RESULT` -- valid, partial and no-route alike.
    """
    if response.get("motion_profile") != sent_profile:
        return (f"motion_profile mismatch: sent {sent_profile!r}, "
                f"got {response.get('motion_profile')!r} back")
    expected = expected_stm_setup(sent_profile)
    if response.get("stm_setup") != expected:
        return (f"stm_setup mismatch for {sent_profile}: expected {expected!r}, "
                f"got {response.get('stm_setup')!r}")
    return None


def send_plan_v2(url: str, payload: Any, timeout: float = 95.0) -> tuple[int, dict[str, Any]]:
    """POST one bounded protocol-v2 request.

    The server owns a 90-second ceiling; five seconds of client-side margin
    avoids abandoning an on-time response because of LAN/JSON overhead.
    """
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return reply.status, json.load(reply)
    except urllib.error.HTTPError as error:
        try:
            return error.code, json.load(error)
        finally:
            error.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual smoke client for the experimental protocol-v2 "
                     "planning endpoint (POST /plan/v2)."
    )
    parser.add_argument("--host", required=True, help="Algorithm PC IP or hostname")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--map", type=Path,
                         help="path to a v2-shaped PLAN_REQUEST JSON file; "
                              "defaults to a small built-in one-obstacle request")
    parser.add_argument("--mission-id", default=DEFAULT_MISSION_ID)
    parser.add_argument("--motion-profile", choices=MOTION_PROFILES, default=None,
                         help="set the request's motion_profile; omit to leave the "
                              "payload as-is (server default: PRECISION)")
    parser.add_argument("--mode", choices=("B2", "B3"), default=None,
                         help="set the request mode; omit to leave the payload as-is "
                              "(built-in request: B3, the team default, which uses the "
                              "whole 67 s search, so it takes over a minute; B2 is fast)")
    parser.add_argument("--timeout", type=float, default=95.0)
    parser.add_argument("--full", action="store_true",
                         help="print the full response including waypoints/actions")
    args = parser.parse_args()

    if args.map:
        payload = json.loads(args.map.read_text(encoding="utf-8"))
        payload.setdefault("mission_id", args.mission_id)
    else:
        payload = dict(DEFAULT_PAYLOAD)
        payload["mission_id"] = args.mission_id
    if args.mode is not None:
        payload["mode"] = args.mode
    if args.motion_profile is not None:
        payload["motion_profile"] = args.motion_profile
    sent_profile = payload.get("motion_profile", DEFAULT_MOTION_PROFILE)

    url = f"http://{args.host}:{args.port}/plan/v2"
    try:
        start_time = time.perf_counter()
        status, response = send_plan_v2(url, payload, args.timeout)
        elapsed = time.perf_counter() - start_time
    except (OSError, json.JSONDecodeError, urllib.error.URLError) as error:
        raise SystemExit(f"Request failed: {error}") from None

    print(f"Elapsed: {elapsed:.3f}s")
    if args.full or not response.get("valid"):
        print(json.dumps(response, indent=2))
    else:
        print(json.dumps({
            "type": response.get("type"),
            "protocol_version": response.get("protocol_version"),
            "mission_id": response.get("mission_id"),
            "valid": response.get("valid"),
            "complete": response.get("complete"),
            "motion_profile": response.get("motion_profile"),
            "stm_setup": response.get("stm_setup"),
            "obstacle_order": response.get("obstacle_order"),
            "termination": response.get("termination"),
            "delivery": response.get("delivery"),
            "optimality_proven": response.get("optimality_proven"),
            "message": response.get("message"),
        }, indent=2))

    if status == 200 and response.get("type") == "PLAN_RESULT":
        mismatch = profile_echo_error(response, sent_profile)
        if mismatch:
            raise SystemExit(mismatch)
    if status != 200 or not response.get("valid") or not response.get("complete"):
        raise SystemExit(1)
    if response.get("mission_id") != payload.get("mission_id"):
        raise SystemExit(
            f"mission_id mismatch: sent {payload.get('mission_id')!r}, "
            f"got {response.get('mission_id')!r} back"
        )
    print(f"Received {len(response['actions'])} actions; route is valid "
          f"(mission_id={response['mission_id']!r}, "
          f"motion_profile={response['motion_profile']!r}).")


if __name__ == "__main__":
    main()
