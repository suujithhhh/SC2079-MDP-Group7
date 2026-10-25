"""Bounded prefix-search execution: policy, framing, verification, decisions.

No subprocess is started here (tests/test_bounded_prefix_process.py does that).
The verifier tests run the REAL pipeline on one obstacle (a fraction of a
second) and tamper with a genuine route record to prove that the parent's
canonical replay rejects every kind of lie.

Run from algo/:
    python -m unittest tests.test_bounded_prefix_planning -v
"""

import copy
import inspect
import io
import re
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

import bounded_prefix_planning as bp  # noqa: E402
import prefix_worker as pw  # noqa: E402
import route_prefix_search as rps  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

START = Pose(100.0, 100.0, 0.0)
OBS_A = Obstacle(1, 150.0, 100.0, "W")
INPUT = bp.PlanInput(START, (OBS_A,), "B3", False)
SCALED = bp.ExecutionPolicy(planning_target_s=4.0, hard_stop_s=6.0, verification_deadline_s=9.0,
                            return_target_s=10.0, return_deadline_s=11.0, request_ceiling_s=14.0,
                            min_search_budget_s=0.5, death_wait_s=3.0, cancel_grace_s=1.0)

_REAL = {}


def real_result():
    if not _REAL:
        result = rps.optimise_prefix_b3_route(START, [OBS_A], allow_alternative_viewing=False)
        _REAL["result"] = result
        _REAL["record"] = pw.route_record(result.complete_route)
    return _REAL["result"], copy.deepcopy(_REAL["record"])


def fake_record(cost, order=(1,)):
    """Schema-valid route record with no real geometry (for the stub verifier)."""
    return {
        "obstacle_order": list(order), "candidate_indices": [0] * len(order),
        "skipped_obstacle_ids": [], "total_cost": cost, "total_distance_cm": cost,
        "final_pose": [0.0, 0.0, 0.0],
        "legs": [{"obstacle_id": o, "target_pose": [0.0, 0.0, 0.0], "commands": [["F", 10]],
                  "final_pose": [0.0, 0.0, 0.0], "total_cost": cost, "total_distance_cm": cost}
                 for o in order],
    }


def fake_final(stop="exhausted", route=None, **over):
    payload = {
        "stop_reason": stop, "work_limit_hit": None,
        "route_status": "complete" if route is not None else "none",
        "optimality_proven": False, "complete_route_proven_impossible": False,
        "search_space_exhausted": False, "lower_bound_certified": True, "bound_violation_count": 0,
        "unresolved_branch_count": 0, "min_unresolved_lower_bound": None,
        "unresolved_bound_known": True, "empty_candidate_obstacle_ids": [],
        "message": "m", "elapsed_s": 0.5, "leg_calls": 1, "lattice_expansions": 1,
        "nodes_expanded": 1, "incumbent_updates": 0, "complete_route": route,
    }
    payload.update(over)
    return payload


class PolicyAndInterface(unittest.TestCase):
    def test_production_values(self):
        policy = bp.PRODUCTION_POLICY
        self.assertEqual((policy.planning_target_s, policy.hard_stop_s, policy.request_ceiling_s),
                         (63.0, 66.0, 70.0))
        self.assertEqual((policy.verification_deadline_s, policy.return_target_s, policy.return_deadline_s),
                         (67.0, 67.5, 68.0))

    def test_invalid_chains_are_rejected(self):
        for changes in ({"planning_target_s": 86.0}, {"hard_stop_s": 50.0},
                        {"verification_deadline_s": 76.0}, {"return_deadline_s": 86.0},
                        {"return_deadline_s": 90.0}, {"request_ceiling_s": 60.0},
                        {"planning_target_s": float("nan")}, {"death_wait_s": 0},
                        {"hard_stop_s": True}, {"min_search_budget_s": 63.0},
                        {"tempdir_attempts": 0}):
            with self.assertRaises(ValueError, msg=str(changes)):
                bp.ExecutionPolicy(**changes)
        self.assertTrue(bp.PRODUCTION_POLICY.hard_stop_s + bp.PRODUCTION_POLICY.death_wait_s
                        <= bp.PRODUCTION_POLICY.verification_deadline_s)

    def test_no_client_timing_control_exists(self):
        self.assertEqual({f for f in bp.PlanInput.__dataclass_fields__},
                         {"start", "obstacles", "mode", "allow_alternative_viewing"})
        self.assertEqual(set(inspect.signature(bp.BoundedPrefixPlanner.plan).parameters),
                         {"self", "inp", "accepted_at", "cancel"})
        spec_keys = {"mode", "start", "obstacles", "allow_alternative_viewing", "search_budget_s"}
        with self.assertRaises(pw.FrameError):
            pw.parse_spec({**{k: None for k in spec_keys}, "deadline_s": 5})

    def test_only_the_v2_service_adopts_the_bounded_layer(self):
        service_v2 = (ALGO / "planning_service_v2.py").read_text(encoding="utf-8")
        self.assertIn("from bounded_prefix_planning import", service_v2)
        self.assertIn("BoundedPrefixPlanner", service_v2)

        # Protocol v1 and the reusable in-process route optimiser remain
        # independent.  Worker/process containment belongs at the v2 service
        # boundary, not inside either lower layer.
        for name in ("planning_service.py", "executable_route_optimizer.py"):
            text = (ALGO / name).read_text(encoding="utf-8")
            for module in ("bounded_prefix_planning", "prefix_worker", "worker_containment"):
                self.assertNotIn(module, text, f"{name} must not import {module}")

    def test_no_process_scanning_or_foreign_signals_in_production_code(self):
        forbidden = ("taskkill", "psutil", "pkill", "killall", "tasklist", "wmic", "EnumProcesses",
                     "CreateToolhelp32Snapshot", "process_iter", "os.kill(")
        for name in ("worker_containment.py", "bounded_prefix_planning.py", "prefix_worker.py"):
            text = (ALGO / name).read_text(encoding="utf-8")
            for token in forbidden:
                self.assertNotIn(token, text, f"{name} contains {token!r}")
        containment = (ALGO / "worker_containment.py").read_text(encoding="utf-8")
        for call in re.findall(r"os\.killpg\(([^,]+),", containment):
            self.assertEqual(call.strip(), "self.proc.pid", "killpg must target only the worker's own group")


class FrameCodec(unittest.TestCase):
    def test_round_trip_preserves_floats_exactly(self):
        payload = {"x": 0.1 + 0.2, "y": 1e-300, "z": [3, 2.5]}
        kind, back = pw.decode_frame(pw.encode_frame(4, pw.KIND_INCUMBENT, payload), 4)
        self.assertEqual((kind, back), (pw.KIND_INCUMBENT, payload))

    def test_violations_are_rejected(self):
        good = pw.encode_frame(0, pw.KIND_HELLO, {})
        cases = {
            "truncated": good[:-1],
            "oversized": b"x" * (pw.MAX_FRAME_BYTES + 1),
            "no json": b"not json\n",
            "not utf8": b"\xff\xfe\n",
            "not an object": b"[1]\n",
            "extra key": b'{"seq":0,"kind":"HELLO","payload":{},"x":1}\n',
            "missing key": b'{"seq":0,"kind":"HELLO"}\n',
            "bool seq": b'{"seq":true,"kind":"HELLO","payload":{}}\n',
            "unknown kind": b'{"seq":0,"kind":"BOGUS","payload":{}}\n',
            "nan": b'{"seq":0,"kind":"HELLO","payload":NaN}\n',
            "infinity": b'{"seq":0,"kind":"HELLO","payload":Infinity}\n',
        }
        for name, line in cases.items():
            with self.assertRaises(pw.FrameError, msg=name):
                pw.decode_frame(line, 0)

    def test_sequence_must_be_exact(self):
        line = pw.encode_frame(2, pw.KIND_HELLO, {})
        for expected in (0, 1, 3):
            with self.assertRaises(pw.FrameError):
                pw.decode_frame(line, expected)
        with self.assertRaises(pw.FrameError):
            pw.decode_frame(line, 2, allowed_kinds={pw.KIND_FINAL})

    def test_the_encoder_refuses_an_oversized_frame_and_nan(self):
        with self.assertRaises(pw.FrameError):
            pw.encode_frame(0, pw.KIND_FINAL, "x" * pw.MAX_FRAME_BYTES)
        with self.assertRaises(ValueError):
            pw.encode_frame(0, pw.KIND_FINAL, float("nan"))

    def test_the_writer_sequences_and_flushes(self):
        stream = io.BytesIO()
        writer = pw.FrameWriter(stream)
        writer.send(pw.KIND_HELLO, {})
        writer.send(pw.KIND_FINAL, {})
        lines = stream.getvalue().splitlines(keepends=True)
        self.assertEqual([pw.decode_frame(line, i)[0] for i, line in enumerate(lines)],
                         [pw.KIND_HELLO, pw.KIND_FINAL])


class RecordSchema(unittest.TestCase):
    def test_a_real_record_is_valid(self):
        _result, record = real_result()
        bp.check_route_record_schema(record)

    def test_malformed_records_are_rejected(self):
        _result, good = real_result()

        def broken(edit):
            record = copy.deepcopy(good)
            edit(record)
            return record

        mutations = {
            "extra key": lambda r: r.update(extra=1),
            "missing key": lambda r: r.pop("legs"),
            "string cost": lambda r: r.update(total_cost="1"),
            "nan cost": lambda r: r.update(total_cost=float("nan")),
            "bool id": lambda r: r["obstacle_order"].__setitem__(0, True),
            "negative index": lambda r: r["candidate_indices"].__setitem__(0, -1),
            "short pose": lambda r: r.update(final_pose=[1.0, 2.0]),
            "length mismatch": lambda r: r["candidate_indices"].append(0),
            "leg key": lambda r: r["legs"][0].pop("commands"),
            "command shape": lambda r: r["legs"][0]["commands"].__setitem__(0, ["F"]),
            "command value type": lambda r: r["legs"][0]["commands"].__setitem__(0, ["F", 1.5]),
            "too many legs": lambda r: r["legs"].extend([r["legs"][0]] * 9),
        }
        for name, edit in mutations.items():
            with self.assertRaises(pw.FrameError, msg=name):
                bp.check_route_record_schema(broken(edit))

    def test_final_schema(self):
        bp.check_final_schema(fake_final())
        bp.check_final_schema(fake_final("first_complete_route", fake_record(5.0)))
        for bad in (fake_final(stop="bogus"), fake_final(route_status="weird"),
                    fake_final(unresolved_branch_count=-1), fake_final(message=3),
                    fake_final(route_status="complete"),                        # complete without a route
                    fake_final(route=fake_record(1.0), route_status="none"),    # route without status
                    {k: v for k, v in fake_final().items() if k != "message"}):
            with self.assertRaises(pw.FrameError):
                bp.check_final_schema(bad)


class Verification(unittest.TestCase):
    def verify(self, record, **kwargs):
        return bp.verify_route_record(record, INPUT, **kwargs)

    def test_a_genuine_route_verifies_and_matches_the_search(self):
        result, record = real_result()
        route = self.verify(record)
        direct = result.complete_route
        self.assertEqual(route.stm_commands, direct.stm_commands)
        self.assertEqual(route.total_cost, direct.total_cost)
        self.assertEqual(route.total_distance_cm, direct.total_distance_cm)
        self.assertEqual(route.final_pose, direct.final_pose)
        self.assertEqual(route.obstacle_order, direct.obstacle_order)
        self.assertEqual(route.candidate_indices, direct.candidate_indices)
        self.assertEqual(route.skipped_obstacle_ids, ())
        self.assertGreater(len(route.stm_commands), 0)

    def test_every_kind_of_tampering_is_rejected(self):
        _result, good = real_result()
        first = good["legs"][0]["commands"]
        splittable = next(i for i, (_d, v) in enumerate(first) if v >= 2)

        def tampered(edit):
            record = copy.deepcopy(good)
            edit(record)
            return record

        def split_command(record):
            direction, value = record["legs"][0]["commands"][splittable]
            record["legs"][0]["commands"][splittable:splittable + 1] = [[direction, 1], [direction, value - 1]]

        mutations = {
            "command value": lambda r: r["legs"][0]["commands"][0].__setitem__(1, r["legs"][0]["commands"][0][1] + 1),
            "command direction": lambda r: r["legs"][0]["commands"][0].__setitem__(
                0, "B" if r["legs"][0]["commands"][0][0] != "B" else "F"),
            "illegal value zero": lambda r: r["legs"][0]["commands"][0].__setitem__(1, 0),
            "illegal value 1000": lambda r: r["legs"][0]["commands"].__setitem__(0, ["F", 1000]),
            "unknown direction": lambda r: r["legs"][0]["commands"][0].__setitem__(0, "X"),
            "unsafe sweep": lambda r: r["legs"][0]["commands"].__setitem__(0, ["F", 999]),
            "uncompressed commands": split_command,
            "candidate index": lambda r: r["candidate_indices"].__setitem__(0, 7),
            "target pose": lambda r: r["legs"][0]["target_pose"].__setitem__(0, r["legs"][0]["target_pose"][0] + 1.0),
            "leg cost": lambda r: r["legs"][0].update(total_cost=r["legs"][0]["total_cost"] + 1.0),
            "leg distance": lambda r: r["legs"][0].update(total_distance_cm=r["legs"][0]["total_distance_cm"] + 1.0),
            "leg final pose": lambda r: r["legs"][0]["final_pose"].__setitem__(0, r["legs"][0]["final_pose"][0] + 0.5),
            "route cost": lambda r: r.update(total_cost=r["total_cost"] - 1.0),
            "route distance": lambda r: r.update(total_distance_cm=r["total_distance_cm"] + 1.0),
            "route final pose": lambda r: r["final_pose"].__setitem__(1, r["final_pose"][1] + 1.0),
            "obstacle missing": lambda r: r.update(obstacle_order=[], candidate_indices=[], legs=[]),
            "obstacle skipped": lambda r: r.update(skipped_obstacle_ids=[1]),
            "unknown obstacle": lambda r: (r.update(obstacle_order=[9]), r["legs"][0].update(obstacle_id=9)),
            "leg for another obstacle": lambda r: r["legs"][0].update(obstacle_id=2),
            "legs dropped": lambda r: r["legs"].clear(),
            "commands dropped": lambda r: r["legs"][0].update(commands=[]),
        }
        for name, edit in mutations.items():
            with self.assertRaises((bp.RouteVerificationError, pw.FrameError), msg=name):
                self.verify(tampered(edit))

    def test_the_verification_deadline_is_enforced(self):
        _result, record = real_result()
        with self.assertRaises(bp.RouteVerificationError):
            self.verify(record, deadline=0.0, clock=lambda: 1.0)

    def test_partial_and_wrong_start_routes_are_rejected(self):
        _result, record = real_result()
        other = bp.PlanInput(Pose(101.0, 100.0, 0.0), (OBS_A,), "B3", False)
        with self.assertRaises(bp.RouteVerificationError):
            bp.verify_route_record(record, other)     # the chain no longer starts where the record says
        two = bp.PlanInput(START, (OBS_A, Obstacle(2, 100.0, 150.0, "S")), "B3", False)
        with self.assertRaises(bp.RouteVerificationError):
            bp.verify_route_record(record, two)       # a route that visits one of two obstacles


class HonestFlags(unittest.TestCase):
    def route(self, cost=10.0):
        return SimpleNamespace(total_cost=cost)

    def flags(self, mode="B3", termination=bp.TERM_SEARCH_FINISHED, route="default", source=bp.SOURCE_FINAL,
              **over):
        final = fake_final(**{"stop_reason": "exhausted", "optimality_proven": True, **over})
        route = self.route() if route == "default" else route
        return bp._honest_flags(mode, termination, final, route, source)

    def test_an_honest_closed_b3_search_may_be_proven(self):
        self.assertEqual(self.flags(), (True, False, True))
        self.assertEqual(self.flags(stop_reason="bound_closed"), (True, False, True))
        self.assertEqual(self.flags(unresolved_branch_count=2, min_unresolved_lower_bound=10.0),
                         (True, False, False))

    def test_every_bounded_or_failed_stop_reports_nothing_proven(self):
        for termination in (bp.TERM_PLANNER_BUDGET, bp.TERM_WORK_LIMIT, bp.TERM_CANCELLED,
                            bp.TERM_HARD_BACKSTOP, bp.TERM_STARTUP_OVERRAN, bp.TERM_WORKER_FAILED,
                            bp.TERM_EXECUTOR_FAILED):
            self.assertEqual(self.flags(termination=termination, complete_route_proven_impossible=True),
                             (False, False, False), termination)
        self.assertEqual(bp._honest_flags("B3", bp.TERM_HARD_BACKSTOP, None, self.route(), bp.SOURCE_B2),
                         (False, False, False))

    def test_a_claimed_proof_is_rechecked(self):
        self.assertFalse(self.flags(mode="B2")[0])
        self.assertFalse(self.flags(lower_bound_certified=False)[0])
        self.assertFalse(self.flags(bound_violation_count=1)[0])
        self.assertFalse(self.flags(stop_reason="first_complete_route")[0])
        self.assertFalse(self.flags(unresolved_branch_count=1, unresolved_bound_known=False)[0])
        self.assertFalse(self.flags(unresolved_branch_count=1, min_unresolved_lower_bound=9.0)[0])
        self.assertFalse(self.flags(source=bp.SOURCE_B3)[0])          # the FINAL route was not the one retained
        self.assertFalse(self.flags(route=None)[0])
        self.assertFalse(self.flags(optimality_proven=False)[0])

    def test_impossibility_needs_an_exhausted_search_with_no_route(self):
        self.assertEqual(self.flags(route=None, optimality_proven=False,
                                    complete_route_proven_impossible=True), (False, True, True))
        self.assertFalse(self.flags(route=self.route(), optimality_proven=False,
                                    complete_route_proven_impossible=True)[1])
        self.assertFalse(self.flags(route=None, optimality_proven=False, unresolved_branch_count=1,
                                    complete_route_proven_impossible=True)[1])
        self.assertFalse(self.flags(route=None, optimality_proven=False, stop_reason="bound_closed",
                                    complete_route_proven_impossible=True)[1])


class Decisions(unittest.TestCase):
    """`_accept` and `_run_loop` on a hand-built run: no process, fake clock."""

    def make_run(self, clock=lambda: 0.0, cancel=None, stub=True):
        verifier = (lambda record, inp, config=None, *, deadline=None, clock=None: SimpleNamespace(
            total_cost=record["total_cost"])) if stub else bp.verify_route_record
        planner = bp.BoundedPrefixPlanner(SCALED, verifier=verifier, clock=clock)
        run = bp._Run(planner, INPUT, 0.0, cancel)
        run.worker = SimpleNamespace(proc=SimpleNamespace(stdin=io.BytesIO()))
        return run

    def frame(self, kind, payload):
        return ("frame", kind, payload)

    def test_incumbent_rules(self):
        run = self.make_run()
        self.assertIsNone(run._accept(self.frame(pw.KIND_INCUMBENT, {"phase": "b2", "route": fake_record(50.0)})))
        self.assertIsNone(run._accept(self.frame(pw.KIND_INCUMBENT, {"phase": "b3", "route": fake_record(40.0)})))
        self.assertEqual([p for p, _ in run.incumbents], ["b2", "b3"])
        bad = {
            "wrong phase": {"phase": "b2", "route": fake_record(30.0)},
            "not cheaper": {"phase": "b3", "route": fake_record(40.0)},
            "costlier": {"phase": "b3", "route": fake_record(45.0)},
            "partial": {"phase": "b3", "route": {**fake_record(30.0), "skipped_obstacle_ids": [2]}},
            "bad schema": {"phase": "b3", "route": {"x": 1}},
            "bad payload": {"phase": "b3"},
        }
        for name, payload in bad.items():
            decision = run._accept(self.frame(pw.KIND_INCUMBENT, payload))
            self.assertEqual(decision.termination, bp.TERM_WORKER_FAILED, name)
            self.assertIn("protocol", decision.detail)
        self.assertEqual(len(run.incumbents), 2)
        first = self.make_run()
        decision = first._accept(self.frame(pw.KIND_INCUMBENT, {"phase": "b3", "route": fake_record(1.0)}))
        self.assertEqual(decision.termination, bp.TERM_WORKER_FAILED)     # the first incumbent must be B2

    def test_final_error_and_unexpected_frames(self):
        run = self.make_run()
        decision = run._accept(self.frame(pw.KIND_FINAL, fake_final("wall_clock_checkpoint_limit")))
        self.assertEqual((decision.termination, decision.from_final), (bp.TERM_PLANNER_BUDGET, True))
        for stop, term in (("work_limit", bp.TERM_WORK_LIMIT), ("cancelled", bp.TERM_CANCELLED),
                           ("exhausted", bp.TERM_SEARCH_FINISHED), ("bound_closed", bp.TERM_SEARCH_FINISHED),
                           ("first_complete_route", bp.TERM_SEARCH_FINISHED)):
            self.assertEqual(run._accept(self.frame(pw.KIND_FINAL, fake_final(stop))).termination, term)
        decision = run._accept(self.frame(pw.KIND_ERROR, {"error": "ValueError"}))
        self.assertEqual((decision.termination, decision.detail), (bp.TERM_WORKER_FAILED, "worker_error: ValueError"))
        decision = run._accept(self.frame(pw.KIND_ERROR, {"error": "not an identifier!"}))
        self.assertEqual(decision.detail, "worker_error: ?")
        self.assertEqual(run._accept(self.frame(pw.KIND_HELLO, {})).termination, bp.TERM_WORKER_FAILED)
        self.assertEqual(run._accept(self.frame(pw.KIND_FINAL, {"junk": 1})).termination, bp.TERM_WORKER_FAILED)
        for event in (("eof",), ("error", "protocol: truncated frame")):
            self.assertEqual(run._accept(event).termination, bp.TERM_WORKER_FAILED)

    def test_precedence_final_then_cancel_then_hard_stop(self):
        past_everything = lambda: 88.0          # after R0 + 66 s
        cancel = threading.Event()
        cancel.set()
        # nothing received: an external cancel outranks the hard stop
        run = self.make_run(past_everything, cancel)
        self.assertEqual(run._run_loop().termination, bp.TERM_CANCELLED)
        # no cancel: the hard stop
        run = self.make_run(past_everything)
        self.assertEqual(run._run_loop().termination, bp.TERM_HARD_BACKSTOP)
        # a valid FINAL already received outranks both
        for cancel_object in (cancel, None):
            run = self.make_run(past_everything, cancel_object)
            run.events.put((0.0, self.frame(pw.KIND_FINAL, fake_final("exhausted"))))
            decision = run._run_loop()
            self.assertEqual((decision.termination, decision.from_final), (bp.TERM_SEARCH_FINISHED, True))
        # a worker that already died outranks the hard stop
        run = self.make_run(past_everything)
        run.events.put((0.0, ("eof",)))
        self.assertEqual(run._run_loop().termination, bp.TERM_WORKER_FAILED)
        # incumbents received before the decision are kept, whatever ends the run
        run = self.make_run(past_everything)
        run.events.put((0.0, self.frame(pw.KIND_INCUMBENT, {"phase": "b2", "route": fake_record(9.0)})))
        self.assertEqual(run._run_loop().termination, bp.TERM_HARD_BACKSTOP)
        self.assertEqual(len(run.incumbents), 1)

    def test_the_hard_stop_boundary_is_the_receive_time(self):
        hard = SCALED.hard_stop_s
        just_before, at_stop = hard - 0.001, hard
        late_clock = lambda: hard + 5.0      # the loop only looks long after the hard stop

        # A FINAL received just before the stop instant wins, however late the loop looks.
        run = self.make_run(late_clock)
        run.events.put((just_before, self.frame(pw.KIND_FINAL, fake_final("exhausted"))))
        decision = run._run_loop()
        self.assertEqual((decision.termination, decision.from_final), (bp.TERM_SEARCH_FINISHED, True))

        # A FINAL received at the stop instant is never accepted.
        run = self.make_run(late_clock)
        run.events.put((at_stop, self.frame(pw.KIND_FINAL, fake_final("exhausted"))))
        self.assertEqual(run._run_loop().termination, bp.TERM_HARD_BACKSTOP)

        # Incumbents: before the stop instant kept, at or after it never accepted,
        # and nothing queued behind a late frame is accepted either.
        run = self.make_run(late_clock)
        run.events.put((1.0, self.frame(pw.KIND_INCUMBENT, {"phase": "b2", "route": fake_record(9.0)})))
        run.events.put((at_stop, self.frame(pw.KIND_INCUMBENT, {"phase": "b3", "route": fake_record(8.0)})))
        run.events.put((at_stop + 1, self.frame(pw.KIND_FINAL, fake_final("exhausted"))))
        self.assertEqual(run._run_loop().termination, bp.TERM_HARD_BACKSTOP)
        self.assertEqual([r["total_cost"] for _p, r in run.incumbents], [9.0])

        # A worker death seen before the stop instant outranks the hard stop; after it, it does not.
        run = self.make_run(late_clock)
        run.events.put((just_before, ("eof",)))
        self.assertEqual(run._run_loop().termination, bp.TERM_WORKER_FAILED)
        run = self.make_run(late_clock)
        run.events.put((at_stop, ("eof",)))
        self.assertEqual(run._run_loop().termination, bp.TERM_HARD_BACKSTOP)

    def test_nothing_is_accepted_after_the_decision(self):
        run = self.make_run(lambda: SCALED.hard_stop_s + 1.0)
        decision = run._run_loop()
        self.assertEqual(decision.termination, bp.TERM_HARD_BACKSTOP)
        run.events.put((0.0, self.frame(pw.KIND_INCUMBENT, {"phase": "b2", "route": fake_record(9.0)})))
        outcome = run._assemble(decision, bp.Cleanup(True, "job_object", True, True, True, True, -1, False, True))
        self.assertEqual(run.incumbents, [])
        self.assertIsNone(outcome.route)
        self.assertEqual(outcome.incumbents_received, 0)

    def test_a_cancel_escalation_uses_the_same_receive_time_boundary(self):
        cancel = threading.Event()
        cancel.set()
        run = self.make_run(lambda: 20.0, cancel)      # cancel sent at 20.0: stop instant min(21.0, hard 6.0)
        run.events.put((5.9, self.frame(pw.KIND_FINAL, fake_final("cancelled"))))
        self.assertEqual(run._run_loop().termination, bp.TERM_CANCELLED)
        run = self.make_run(lambda: 20.0, cancel)
        run.events.put((6.0, self.frame(pw.KIND_FINAL, fake_final("exhausted"))))
        decision = run._run_loop()
        self.assertEqual((decision.termination, decision.detail), (bp.TERM_CANCELLED, "cancel_escalated"))

    def test_a_cancel_is_sent_once_and_escalated_after_the_grace_period(self):
        cancel = threading.Event()
        cancel.set()
        times = iter([10.0, 10.5, 11.2, 11.2, 11.2])       # cancel at 10.0; grace is 1.0 s
        run = self.make_run(lambda: next(times), cancel)
        decision = run._run_loop()
        self.assertEqual((decision.termination, decision.detail), (bp.TERM_CANCELLED, "cancel_escalated"))
        frames = run.worker.proc.stdin.getvalue().splitlines(keepends=True)
        self.assertEqual([pw.decode_frame(line, i)[0] for i, line in enumerate(frames)], [pw.KIND_CANCEL])


if __name__ == "__main__":
    unittest.main()
