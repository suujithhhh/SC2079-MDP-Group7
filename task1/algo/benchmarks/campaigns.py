"""Benchmark campaigns, coverage manifest and dry-run arithmetic (measurement only).

A campaign is a list of sessions; a session is an ordered list of cases run
sequentially, one isolated worker per invocation. Nothing here runs a planner:
the runner (`bench_executable_planner.py campaign ...`) executes the cases.

Worst-case arithmetic (always printed by `campaign --dry-run`):

    worst_case_s = sum(case.timeout_s * case.repeats)
                   + ALLOWANCE_S * (number of worker invocations)

`ALLOWANCE_S` covers spawn/import, worker-side normalisation, parent-side
replay, timeout kill verification and temporary-directory cleanup. Each session
must fit its own `cap_s` (per session, when a campaign is resumable). The driver
refuses to START a case whose timeout plus allowance exceeds the remaining
session budget and reports every unstarted invocation as `missing`; it never
skips or reorders required evidence silently.

Accepted outcomes per case (`accept`): `C` a complete route, `P` a partial route
or a no-route result with honest metadata, `T` a harness hard timeout recorded
with verified cleanup (evidence of the attempt only: the route result stays
`not_measured`). Old-search cases (`b2`/`b3`) never exceed the old-search
safety cap of 120 s.

The `bounded` campaign measures the standalone bounded executor
(`bounded_prefix_planning.py`) through modes `bpb2`/`bpb3`: session BQ (the
nested-Job-Object check, run alone first), B1 (the 4-8 `A_spread`
representatives), B2 (layout breadth) and B3 (retention, stress stop and soak).
Every bounded case names its server-side `policy` (`production` or `stress`);
`stall` selects the test worker's deterministic stall and `soak_plans` runs
several plans on one executor instance. No bounded case accepts `T`: a harness
hard timeout of the bounded executor is itself a failure.

Item status: `satisfied`, `timeout` (a timeout where `T` is accepted),
`unsatisfied` (a record exists but its outcome is not accepted) or `missing`.
Coverage of a group of items: `complete` (all satisfied/timeout), `not_started`
(all missing) or `partial`. Invoking a campaign once does not mean its coverage
is complete; coverage comes from the manifest and the recorded results.
"""

from dataclasses import dataclass

MANIFEST_VERSION = 2          # 2: the `bounded` campaign was added
ALLOWANCE_S = 15
OLD_SEARCH_CAP_S = 120
MAX_TIMEOUT_S = 600
OPTION_KEYS = frozenset({
    "max_leg_calls", "max_expansions", "max_nodes", "max_frontier", "checkpoint_s",
    "cancel_after_legs", "profile", "tracemalloc",
    "policy", "stall", "soak_plans",          # bounded-executor modes only
})


@dataclass(frozen=True)
class Case:
    case_id: str
    fixture: str
    mode: str                  # b2 | b3 (old search) | pb2 | pb3 (prefix search) | bpb2 | bpb3 (bounded)
    alternatives: str          # "off" | "on"
    timeout_s: int
    accept: str                # subset of "CPT"
    purpose: str
    repeats: int = 1
    options: tuple = ()        # ((key, value), ...) benchmark-operator options

    def invocations(self) -> tuple:
        if self.repeats == 1:
            return (self.case_id,)
        return tuple(f"{self.case_id}#{i}" for i in range(1, self.repeats + 1))

    def option_dict(self) -> dict:
        return dict(self.options)


@dataclass(frozen=True)
class Session:
    campaign: str
    name: str
    cap_s: int
    cases: tuple


def _c(case_id, fixture, mode, alternatives, timeout_s, accept, purpose, repeats=1, **options):
    return Case(case_id, fixture, mode, alternatives, timeout_s, accept, purpose, repeats,
                tuple(sorted(options.items())))


QUICK = (
    _c("Q1", "A_spread_01", "pb2", "off", 20, "C", "smoke: n=1 B2"),
    _c("Q2", "A_spread_01", "pb3", "off", 20, "C", "smoke: n=1 B3"),
    _c("Q3", "A_spread_01", "b3", "off", 20, "C", "smoke: old search n=1"),
    _c("Q4", "A_spread_02", "pb2", "off", 60, "C", "smoke: n=2 B2"),
    _c("Q5", "A_spread_02", "pb3", "off", 60, "C", "smoke: n=2 B3, two invocations", repeats=2),
    _c("Q6", "A_spread_02", "b3", "off", 60, "C", "smoke: old search n=2"),
    _c("Q7", "C_no_safe_pose_02", "pb3", "off", 60, "P", "smoke: empty candidate group"),
    _c("Q8", "C_no_safe_pose_02", "pb2", "on", 60, "P", "smoke: empty group, alternatives on"),
    _c("Q9", "A_spread_02", "pb2", "off", 20, "P", "smoke: cancellation", cancel_after_legs=1),
    _c("Q10", "A_spread_02", "pb3", "off", 30, "PC", "smoke: leg-call budget", max_leg_calls=3),
)

S1 = (
    _c("R1", "A_spread_01", "pb2", "off", 60, "C", "n=1 B2"),
    _c("R2", "A_spread_01", "pb3", "off", 60, "C", "n=1 B3 proof"),
    _c("R3", "A_spread_01", "b3", "off", 60, "C", "n=1 exactness"),
    _c("R4", "A_spread_02", "pb2", "off", 120, "C", "n=2 B2"),
    _c("R5", "A_spread_02", "pb3", "off", 120, "C", "n=2 B3 proof"),
    _c("R6", "A_spread_02", "b3", "off", 120, "C", "n=2 exactness"),
    _c("R7", "A_spread_03", "pb2", "off", 180, "C", "n=3 B2"),
    _c("R8", "A_spread_03", "pb3", "off", 300, "C", "n=3 B3 proof"),
    _c("R9", "A_spread_03", "b3", "off", 120, "C", "n=3 exactness (measured 44.8 s)"),
    _c("R10", "A_spread_03", "pb3", "off", 300, "C", "n=3 determinism (second invocation of R8)"),
    _c("R19", "C_unsafe_primary_02", "pb2", "on", 180, "C", "unsafe primary B2"),
    _c("R20", "C_unsafe_primary_02", "pb3", "on", 300, "CT", "unsafe primary B3"),
    _c("R21", "C_unsafe_primary_02", "b3", "on", 120, "CT", "unsafe primary exactness (old b3 measured 151 s)"),
    _c("R22", "C_no_safe_pose_02", "pb2", "off", 60, "P", "no safe pose, n=2"),
    _c("R23", "C_no_safe_pose_08", "pb2", "off", 120, "P", "no safe pose, n=8, proof independent of budget",
       max_leg_calls=2),
    _c("R24", "C_reversal_02", "pb3", "off", 120, "C", "reversal B3"),
    _c("R25", "C_reversal_02", "b3", "off", 120, "C", "reversal exactness"),
    _c("R26", "A_spread_03", "pb3", "off", 120, "PC", "leg-call budget honesty", max_leg_calls=2),
    _c("R27", "A_spread_03", "pb2", "off", 60, "P", "cancellation honesty", cancel_after_legs=1),
    _c("R33", "A_spread_02", "pb2", "on", 180, "C", "alternatives on B2"),
    _c("R34", "A_spread_02", "b2", "on", 120, "CT", "alternatives on completeness (old b2 measured 103 s)"),
)

S2 = (
    _c("R11", "A_spread_04", "pb2", "off", 300, "CT", "n=4 B2"),
    _c("R12", "A_spread_04", "pb3", "off", 600, "CPT", "n=4 B3 budget run", max_leg_calls=30),
    _c("R13", "A_spread_04", "b3", "off", 120, "CT", "n=4 exactness if it completes"),
    _c("R14", "A_spread_06", "pb2", "off", 600, "CT", "n=6 B2"),
    _c("R15", "A_spread_06", "pb3", "off", 600, "CPT", "n=6 B3 budget run", max_leg_calls=12),
    _c("R28", "A_spread_04", "pb3", "on", 300, "CPT", "frontier cap honesty and memory",
       max_frontier=50, max_leg_calls=20),
    _c("R29", "A_spread_04", "pb2", "off", 120, "CPT", "planner checkpoint overshoot", checkpoint_s=10),
    _c("R30", "A_spread_03", "pb2", "off", 180, "CT", "hotspots at n=3", profile="hotspots"),
    _c("R31", "B_clutter_04", "pb2", "off", 300, "CT", "hotspots at n=4", profile="hotspots"),
    _c("R32", "A_spread_03", "pb2", "off", 180, "CT", "cProfile at n=3", profile="cprofile"),
)

S3 = (
    _c("R16", "A_spread_08", "pb2", "off", 600, "CT", "n=8 B2"),
    _c("R17", "A_spread_08", "pb2", "off", 600, "CT", "n=8 determinism (second invocation of R16)"),
    _c("R18", "A_spread_08", "pb3", "off", 600, "CPT", "n=8 B3 budget run", max_leg_calls=16),
)

E1 = (
    _c("X1", "A_spread_06", "pb2", "off", 600, "CT", "n=6 determinism (second invocation of R14)"),
    _c("X2", "A_spread_06", "pb2", "on", 600, "CT", "n=6 alternatives on"),
    _c("X3", "A_spread_08", "pb2", "on", 600, "CT", "n=8 alternatives on"),
    _c("X4", "B_clutter_06", "pb2", "off", 600, "CT", "clutter n=6"),
    _c("X5", "B_clutter_08", "pb2", "off", 600, "CT", "clutter n=8"),
)

E2 = (
    _c("X6", "D_seeded_08_s1", "pb2", "off", 600, "CT", "seeded n=8"),
    _c("X7", "D_seeded_08_s2", "pb2", "off", 600, "CT", "seeded n=8"),
    _c("X8", "D_seeded_08_s3", "pb2", "off", 600, "CT", "seeded n=8"),
    _c("X9", "D_seeded_04_s1", "pb2", "off", 300, "CT", "seeded n=4"),
    _c("X10", "D_seeded_04_s2", "pb2", "off", 300, "CT", "seeded n=4"),
    _c("X11", "D_seeded_04_s3", "pb2", "off", 300, "CT", "seeded n=4"),
    _c("X12", "A_spread_05", "pb2", "off", 300, "CT", "n=5 B2"),
)

E3 = (
    _c("X13", "A_spread_08", "pb3", "on", 600, "CPT", "n=8 memory (alternatives on, budget run)",
       max_leg_calls=16, tracemalloc=True),
    _c("X14", "C_unsafe_primary_05", "pb2", "on", 300, "CT", "unsafe primary n=5"),
    _c("X15", "C_unsafe_primary_08", "pb2", "on", 600, "CT", "unsafe primary n=8"),
    _c("X16", "D_seeded_06_s1", "pb2", "off", 600, "CT", "seeded n=6"),
    _c("X17", "C_no_safe_pose_05", "pb3", "off", 60, "P", "no safe pose n=5"),
    _c("X18", "A_spread_07", "pb2", "off", 600, "CT", "n=7 B2 (G4 representative for seven obstacles)"),
)

BQ = (
    _c("BQ1", "A_spread_02", "bpb3", "off", 90, "C",
       "nested Job Objects: self-exiting real worker, peak RSS after exit, cleanup, sentinel",
       policy="production"),
    _c("BQ2", "A_spread_02", "bpb3", "off", 90, "P",
       "nested Job Objects: guaranteed hard-stop kill of a stalled real worker",
       policy="stress", stall="at_call_1"),
)

B1 = (
    _c("BR1", "A_spread_04", "bpb3", "off", 90, "C", "n=4 representative: availability, repeat",
       repeats=3, policy="production"),
    _c("BR2", "A_spread_05", "bpb3", "off", 90, "C", "n=5 representative: availability", policy="production"),
    _c("BR3", "A_spread_06", "bpb3", "off", 90, "C", "n=6 representative: availability, repeat",
       repeats=3, policy="production"),
    _c("BR4", "A_spread_07", "bpb3", "off", 90, "C", "n=7 representative: availability", policy="production"),
    _c("BR5", "A_spread_08", "bpb3", "off", 90, "C", "n=8 representative: availability, repeat",
       repeats=3, policy="production"),
    _c("BR6", "A_spread_04", "bpb2", "off", 90, "C", "n=4 representative, B2 mode", policy="production"),
    _c("BR7", "A_spread_05", "bpb2", "off", 90, "C", "n=5 representative, B2 mode", policy="production"),
    _c("BR8", "A_spread_06", "bpb2", "off", 90, "C", "n=6 representative, B2 mode", policy="production"),
    _c("BR9", "A_spread_07", "bpb2", "off", 90, "C", "n=7 representative, B2 mode", policy="production"),
    _c("BR10", "A_spread_08", "bpb2", "off", 90, "C", "n=8 representative, B2 mode", policy="production"),
)

B2 = (
    _c("BX1", "B_clutter_04", "bpb3", "off", 90, "CP", "breadth n=4, clutter", policy="production"),
    _c("BX2", "D_seeded_04_s1", "bpb3", "off", 90, "CP", "breadth n=4, seeded 1", policy="production"),
    _c("BX3", "D_seeded_04_s2", "bpb3", "off", 90, "CP", "breadth n=4, seeded 2", policy="production"),
    _c("BX4", "D_seeded_04_s3", "bpb3", "off", 90, "CP", "breadth n=4, seeded 3", policy="production"),
    _c("BX5", "C_unsafe_primary_05", "bpb3", "on", 90, "CP", "alternative viewing, n=5",
       policy="production"),
    _c("BX6", "C_no_safe_pose_05", "bpb3", "off", 90, "P", "impossibility honesty, n=5", policy="production"),
    _c("BX7", "B_clutter_06", "bpb3", "off", 90, "CP", "breadth n=6, clutter", policy="production"),
    _c("BX8", "D_seeded_06_s1", "bpb3", "off", 90, "CP", "breadth n=6, seeded 1", policy="production"),
    _c("BX9", "D_seeded_06_s2", "bpb3", "off", 90, "CP", "breadth n=6, seeded 2", policy="production"),
    _c("BX10", "D_seeded_06_s3", "bpb3", "off", 90, "CP", "breadth n=6, seeded 3", policy="production"),
    _c("BX11", "B_clutter_08", "bpb3", "off", 90, "CP", "breadth n=8, clutter", policy="production"),
    _c("BX12", "D_seeded_08_s1", "bpb3", "off", 90, "CP", "breadth n=8, seeded 1", policy="production"),
    _c("BX13", "D_seeded_08_s2", "bpb3", "off", 90, "CP", "breadth n=8, seeded 2", policy="production"),
    _c("BX14", "D_seeded_08_s3", "bpb3", "off", 90, "CP", "breadth n=8, seeded 3", policy="production"),
    _c("BX15", "C_unsafe_primary_08", "bpb3", "on", 90, "CP", "alternative viewing, n=8",
       policy="production"),
    _c("BX16", "C_no_safe_pose_08", "bpb3", "off", 90, "P", "impossibility honesty, n=8", policy="production"),
    _c("BX17", "A_spread_08", "bpb3", "on", 90, "CP", "memory and timing, alternatives on, n=8",
       policy="production"),
)

B3 = (
    _c("BS1", "A_spread_02", "bpb3", "off", 90, "C",
       "incumbent retention through a guaranteed production hard stop (FINAL withheld)",
       repeats=2, policy="production", stall="before_final"),
    _c("BS2", "A_spread_04", "bpb3", "off", 90, "CP", "natural stress-timeline stop: honest bounded outcome",
       policy="stress"),
    _c("BS5", "A_spread_02", "bpb3", "off", 420, "P",
       "soak: 8 hard-stopped plans on one executor instance (handles, threads, RSS, tree, tempdirs)",
       policy="stress", stall="at_call_1", soak_plans=8),
)

SESSIONS = {
    "quick": (Session("quick", "Q", 720, QUICK),),
    "standard": (
        Session("standard", "S1", 3600, S1),
        Session("standard", "S2", 3600, S2),
        Session("standard", "S3", 3600, S3),
    ),
    "extended": (
        Session("extended", "E1", 3600, E1),
        Session("extended", "E2", 3600, E2),
        Session("extended", "E3", 3600, E3),
    ),
    "bounded": (
        Session("bounded", "BQ", 720, BQ),
        Session("bounded", "B1", 3600, B1),
        Session("bounded", "B2", 3600, B2),
        Session("bounded", "B3", 3600, B3),
    ),
}

# Minimum-evidence requirements: name -> case ids (standard campaign).
REQUIREMENTS = {
    "n1": ("R1", "R2", "R3"), "n2": ("R4", "R5", "R6"), "n3": ("R7", "R8", "R9", "R10"),
    "n4": ("R11", "R12", "R13"), "n6": ("R14", "R15"), "n8": ("R16", "R17", "R18"),
    "unsafe_primary": ("R19", "R20", "R21"), "no_safe_pose": ("R22", "R23"),
    "reversal": ("R24", "R25"), "budget": ("R12", "R15", "R18", "R26"),
    "cancellation": ("R27",), "frontier": ("R28",), "checkpoint_overshoot": ("R29",),
    "alternatives_on": ("R19", "R20", "R21", "R28", "R33", "R34"),
    "hotspots_3_4": ("R30", "R31", "R32"),
    "exactness": ("R3", "R6", "R9", "R21", "R25", "R34"),
}

# Determinism pairs (case ids whose recorded hash and work must be identical).
DETERMINISM_PAIRS = (("R8", "R10"), ("R14", "X1"), ("R16", "R17"), ("Q5#1", "Q5#2"))


def all_sessions() -> tuple:
    return tuple(s for group in SESSIONS.values() for s in group)


def find_session(campaign: str, name: str = None) -> Session:
    sessions = SESSIONS[campaign]
    if name is None:
        if len(sessions) != 1:
            raise KeyError(f"campaign {campaign!r} has several sessions; choose one of "
                           f"{[s.name for s in sessions]}")
        return sessions[0]
    for session in sessions:
        if session.name == name:
            return session
    raise KeyError(f"unknown session {name!r} for campaign {campaign!r}")


def all_cases() -> dict:
    return {case.case_id: case for session in all_sessions() for case in session.cases}


def session_invocations(session: Session) -> tuple:
    return tuple((case, inv) for case in session.cases for inv in case.invocations())


def session_arithmetic(session: Session) -> dict:
    rows = [{"case_id": c.case_id, "fixture": c.fixture, "mode": c.mode,
             "alternatives": c.alternatives, "timeout_s": c.timeout_s, "repeats": c.repeats,
             "subtotal_s": c.timeout_s * c.repeats} for c in session.cases]
    timeouts = sum(r["subtotal_s"] for r in rows)
    invocations = sum(c.repeats for c in session.cases)
    allowance = ALLOWANCE_S * invocations
    worst = timeouts + allowance
    return {"campaign": session.campaign, "session": session.name, "cap_s": session.cap_s,
            "rows": rows, "timeouts_s": timeouts, "invocations": invocations,
            "allowance_s": allowance, "worst_case_s": worst, "fits": worst <= session.cap_s}


def plan_session_run(session: Session, done: frozenset, budget_s: int = None) -> dict:
    """Which invocations to run now. Invocations already in `done` are skipped.
    Starting stops at the first invocation that does not fit the remaining
    budget; it and everything after it are `refused` (reported as missing),
    never skipped past or reordered."""
    remaining = session.cap_s if budget_s is None else budget_s
    to_run, refused = [], []
    stopped = False
    for case, invocation in session_invocations(session):
        if invocation in done:
            continue
        cost = case.timeout_s + ALLOWANCE_S
        if stopped or cost > remaining:
            stopped = True
            refused.append((case, invocation))
            continue
        to_run.append((case, invocation))
        remaining -= cost
    return {"to_run": to_run, "refused": refused, "remaining_s": remaining}


def outcome_letter(record: dict) -> str:
    """C complete, P partial or no-route, T harness timeout, E error, R refused."""
    status = record.get("status")
    return {"complete": "C", "partial": "P", "no_route": "P", "timeout": "T",
            "error": "E", "refused": "R"}.get(status, "E")


def item_status(case: Case, record) -> str:
    if record is None:
        return "missing"
    letter = outcome_letter(record)
    if letter not in case.accept:
        return "unsatisfied"
    return "timeout" if letter == "T" else "satisfied"


def coverage_status(statuses) -> str:
    statuses = list(statuses)
    if not statuses or all(s == "missing" for s in statuses):
        return "not_started"
    if all(s in ("satisfied", "timeout") for s in statuses):
        return "complete"
    return "partial"


def manifest() -> dict:
    """Deterministic description of every campaign, session and case."""
    return {
        "manifest_version": MANIFEST_VERSION, "allowance_s": ALLOWANCE_S,
        "campaigns": {
            name: [{
                "session": s.name, "cap_s": s.cap_s,
                "cases": [{
                    "case_id": c.case_id, "fixture": c.fixture, "mode": c.mode,
                    "alternatives": c.alternatives, "timeout_s": c.timeout_s,
                    "repeats": c.repeats, "accept": c.accept, "options": dict(c.options),
                } for c in s.cases],
            } for s in sessions] for name, sessions in SESSIONS.items()
        },
        "requirements": {k: list(v) for k, v in REQUIREMENTS.items()},
        "determinism_pairs": [list(p) for p in DETERMINISM_PAIRS],
    }
