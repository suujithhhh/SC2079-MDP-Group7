# Executable planner benchmarks

Measurement-only tooling for `executable_route_optimizer.py`. Nothing here is
imported by the planner, the HTTP service or the web UI, and nothing here
changes planner behaviour: it measures, it never optimises. Run everything
from `algo/`.

## Quick start

```bash
python benchmarks/bench_executable_planner.py list
python benchmarks/bench_executable_planner.py run --fixture A_spread_02 --mode b2 b3
python benchmarks/bench_executable_planner.py run --fixture A_spread_04 --mode b3 --timeout 60
python benchmarks/bench_executable_planner.py run --family A_spread --mode legs --timeout 400
python benchmarks/bench_executable_planner.py run --fixture A_spread_03 --mode b2 --profile hotspots
```

One JSON object per measurement is written to **stdout** (JSON lines);
progress goes to stderr. Redirect stdout to keep results. Every measurement
runs in its own subprocess with a hard timeout, so a slow search cannot hang
the harness. Exit code: 0 (timeouts included), 1 if any run errored, 2 if any
run was refused.

## Modes

| `--mode` | What runs |
|---|---|
| `b2` | `find_executable_b2_route()` (first complete route) |
| `b3` | `optimise_executable_b3_route()` (lowest cost over every order) |
| `legs` | A fixed set of individual legs through the real pipeline: start to each primary viewing pose (depth 0), then primary *i* to primary *i+1* in input order (depth 1). Every leg starts from an **ideal** pose, not from a previous leg's estimated final pose, so this measures per-leg cost, **not a route**. It is how per-leg cost is measured at 6-8 obstacles without running the permutation search. |

`pb2`/`pb3` run the standalone prefix-tree search on the real chain (see
"Prefix-search validation" below). `bpb2`/`bpb3` run it through the standalone
bounded executor (see "Bounded-executor validation" below). `--alternatives
off|on` sets `allow_alternative_viewing` for every search mode (default `off`;
ignored by `legs`).

## Baseline safety rules

The current search enumerates every obstacle order (n!), so it is bounded:

| Case | Rule |
|---|---|
| b2/b3, 1-3 obstacles | default timeout 180 s |
| b2/b3, 4-5 obstacles | timeout defaults to, and may not exceed, 120 s |
| b2/b3, 6+ obstacles | **refused** unless `--allow-large-baseline` **and** an explicit `--timeout` of at most 120 s are given |
| `legs` | default 120 s |
| any run | timeout must be finite, positive and at most 600 s |

A refusal is recorded (`status: "refused"`, exit code 2) and nothing is run. A
timeout is a recorded outcome (`status: "timeout"`), never a failure; the
statistics captured up to the last progress snapshot (`--progress-interval`,
default 1 s) are kept in `partial_stats`. These refusal rules apply to the
legacy baseline runner. Protocol v1 stays capped at 5 obstacles; bounded
`/plan/v2` accepts the 6-8 obstacle fixtures through its separate executor.

## Profiling modes (`--profile`)

* `none` (default) - use this for timing numbers.
* `hotspots` - wraps `footprint.pose_is_safe`, `footprint_in_arena`,
  `footprint_hits_obstacle` (the SAT test), `footprint.pose_is_safe_prepared`,
  `swept_collision.safe_primitive_samples` and `motion_primitives.pose_at_fraction`,
  and counts goal tests in the lattice search. Since the collision hot-path
  change, swept samples are checked by `pose_is_safe_prepared` (bit-exact to
  `pose_is_safe`), so the `pose_is_safe`/`footprint_hits_obstacle` rows count
  only the other callers (leg start/goal, viewing candidates, reference
  fallback); Phase A rows (R30-R32) predate this and are not like-for-like.
  Since the prepared-sampling change, `motion_primitives._uniform_primitive_samples`
  is also wrapped: on the normal path each primitive makes one
  `pose_at_fraction` call (the 0.0 validation call) and one
  `_uniform_primitive_samples` call, so `pose_at_fraction` no longer counts
  samples except on the original fallback path.
  Since the per-leg geometry change, `swept_collision._prepare_leg_geometry`
  (once per lattice leg that reaches the search loop),
  `swept_collision._safe_primitive_samples_for_leg` (once per primitive check)
  and `footprint.prepare_primitive_check` are wrapped too. In-domain lattice
  legs then reach `safe_primitive_samples` (and `prepare_pose_check`) only
  through other callers (compression/replay re-validation) or fallback inputs:
  the old `safe_primitive_samples` count equals its new count plus the
  `_safe_primitive_samples_for_leg` count, so rows from before this change are
  not like-for-like on that name. Times are **inclusive and nested** (SAT runs inside `pose_is_safe`,
  which runs inside `safe_primitive_samples`) - never add rows. The wrappers add
  overhead, so wall times in this mode are inflated; use the shares, not the
  seconds. `goal_tests.heading_blocked` counts popped poses already inside the
  goal position tolerance that still failed the goal test.
* `cprofile` - `cProfile` top-25 tables by own and cumulative time
  (`cprofile_top`); `--profile-out DIR` also writes `.pstats` files (point it
  outside the repository).

`--tracemalloc` additionally records peak Python heap (slower).
`--leg-records` includes every planned leg (depth, outcome, time, expansions,
distance, heading difference, cost) in the output.

## Output schema (schema_version 3)

Schema 2 appended four keys after `peak_memory`: `limits`, `validation`, `cleanup`
and `case` (see "Prefix-search validation" below). Schema 3 appends one more,
`bounded`, filled only by `bpb2`/`bpb3` and `null` for every other mode, whose
schema-2 keys keep exactly their schema-2 meaning. `report` reads schema-2 and
schema-3 records alike: a record without a `bounded` block is judged exactly as
before. Keys are `null` where they do not apply, including every planner and
validation field of a harness-timeout record. The list below is the version-1
key set, unchanged.

Every record has exactly these keys, in this order (`RESULT_KEYS`):
`schema_version`, `kind` (`benchmark_result`), `started_at_utc`, `fixture_id`,
`family`, `n_obstacles`, `expect`, `mode`, `alternatives` (null for `legs`),
`profile`, `tracemalloc`, `leg_records_included`, `timeout_s`, `status`,
`status_detail`, `wall_time_s` (parent-measured, includes interpreter start-up),
`planning_time_s` (optimizer only), `result`, `result_hash`, `stats`,
`partial_stats`, `analysis`, `hotspots`, `cprofile_top`, `leg_records`,
`machine`, `python`, `git`, `peak_memory`. Keys that do not apply are `null`.

* `status`: `complete` | `partial` | `no_route` (b2/b3 outcomes; `legs` reports
  `complete`) | `timeout` | `error` | `refused`.
* `result`: route summary (validity, order, cost, commands, reverse commands,
  order counters) or the `legs` summary. `result_hash` is a SHA-256 of the
  route's order, cost and exact `MOV` strings, so two runs are deterministic
  exactly when their hashes are equal.
* `stats` (from `RouteSearchStats.to_dict()`): `leg_requests`, `cache_hits`,
  `cache_misses`, `unique_legs`, `cache_hit_rate`, per-depth `legs_by_depth`,
  `candidates_attempted`, `bound_prunes_pre_call/post_call`,
  `invalid_leg_results`, `incumbent_updates`, `orders_searched/completed`,
  `leg_planner_s`, `lattice_s`, `postprocess_s`, `lattice_expansions` and
  per-`outcomes` count/time/expansions.
  `leg_requests == cache_hits + cache_misses`, and each miss is one planned leg.
* `analysis`: derived shares (leg planner share of planning, time by outcome,
  failed-or-limited share, lattice vs compression/generation, seconds per
  expansion, heading-difference bins).
* `machine`, `python`, `git`: CPU count/OS, Python version, commit SHA and a
  dirty flag. No host name, user name or absolute path is recorded.
* `peak_memory`: `peak_rss_bytes` of the worker process (and
  `tracemalloc_peak_bytes` with `--tracemalloc`).

## Fixtures

JSON files in `fixtures/`, in protocol-v2 request shape plus metadata
(`expect`: `feasible` | `infeasible` | `unverified`, `tags`, `verification`).
`expect` is a static geometry label, not a runtime promise.

**Fixture geometry is historical.** The committed fixtures, their generators
and the recorded evidence were produced with a 1 cm safety margin and a 20 cm
camera standoff (`config.BENCHMARK_FIXTURE_*`, applied by
`fixtures.FIXTURE_GEOMETRY`; `expect` labels, `viewing_report` and the
generator tests use it). The live defaults are now 3 cm and 25 cm. Under them
nine committed fixtures have **no safe viewing pose**: `C_reversal_02`,
`D_seeded_04_s1`, `D_seeded_06_s3`, `D_seeded_08_s1`/`_s2`/`_s3` and
`E_ordering_04_c2`/`_c5`/`_c6`. Benchmark *runs* use the live config, so they
measure the live planner on these fixtures; the recorded evidence still describes
the old geometry. Regenerating the fixtures for the live geometry is a separate,
deliberate step that would start new evidence.

* `A_spread_{01..08}` - nested prefixes of one 8-obstacle layout (`05` was added for the prefix-search validation);
  the first three complete in seconds with the real pipeline.
* `B_clutter_{04,06,08}` - four obstacles in a central cross facing outward
  (detours), plus wall-side obstacles at 6 and 8.
* `C_edge`: `C_unsafe_primary_02` (unsafe primary viewing pose, one safe
  alternative; `C_unsafe_primary_05`/`_08` add the same clipped obstacle to
  5- and 8-obstacle layouts), `C_no_safe_pose_02`/`_05`/`_08` (an obstacle with no safe
  viewing pose at all: infeasible), `C_reversal_02` (route contains reverse
  commands). A provably enclosed-target fixture (`C_enclosed_04`) is deferred.
* `D_seeded_{04,06,08}_s{1,2,3}` - deterministic random layouts from
  `benchmarks/make_seeded_fixtures.py` (`--write` regenerates, `--check` verifies).
* `E_ordering_04_c{1..6}` - six declared four-obstacle candidates for a G2 exactness pair
  that can be certified, from `benchmarks/make_ordering_fixtures.py` (seed `41000 + k`;
  two viewing faces, S/W or S/E, so every viewing heading is within 90 deg of the start
  heading; 50 cm apart). Static checks only. None qualified: all six failed the
  discovery (see "Four-obstacle exactness discovery (NO-GO)"); they stay as the
  recorded candidates and belong to no campaign case.

No fixture demonstrates a real-geometry "primary leg succeeds, its continuation
dead-ends, an alternative pose completes" case; none was found, and a fixture
claiming it would be unverified. That behaviour is to be validated with injected
fake leg planners in the search-core work.

## Instrumentation API

```python
import executable_route_optimizer as ero

stats = ero.RouteSearchStats()            # record_legs=False keeps only counters
result = ero.optimise_executable_b3_route(start, obstacles, stats=stats)
stats.to_dict()                           # JSON-serialisable summary
stats.to_dict(include_legs=True)          # plus one record per planned leg
```

`stats` is optional (`None` by default) on `find_executable_b2_route()` and
`optimise_executable_b3_route()`. It only counts and times what the search
already does. It never changes a returned value, the leg cache, candidate
order, pruning, an exception, or any protocol-v2 response, and adds no field
to `ExecutableRouteResult`. `candidates_attempted`, `bound_prunes_*` and
`invalid_leg_results` count candidate evaluations (cache hits included);
`outcomes`, per-leg records and the time/expansion totals cover only legs
actually planned (cache misses). For the default leg planner the lattice
search and the compression + command-generation stages are timed separately;
an injected leg planner is timed as a whole.

## Timing context (benchmark targets, not implemented)

Planning happens before Android Start, during the two-minute setup period, so
it does not consume the six-minute mission timer. The provisional operational
targets are:

* a 30-second planner checkpoint that covers the ENTIRE `optimise_prefix_b3_route`
  call, including its B2 seed phase; if a complete B2 incumbent exists when the
  budget stops the search, it returns the best complete incumbent;
* a separate 45-second production hard backstop.

Both are now implemented, together with a 60-second request ceiling, only in the
standalone `bounded_prefix_planning.py`; all three are measured from request
acceptance (`R0`) and the planning target includes worker start-up. Nothing in the
service calls it: `/plan/v2` and the service enforce no budget or deadline, and
requests carry no client-controlled budget or deadline field. The `bounded`
campaign (`bpb2`/`bpb3`, below) measures it, but no campaign has been run yet, so
these remain targets to be judged against measurements.

Update (2026-10-05): the production policy in `bounded_prefix_planning.py` is now
67/81/86/87/88/90 s (target, hard stop, verification, return target, return
deadline, ceiling); it was 60/74/79/80/81/83 s from 2026-10-04. The numbers and gates in this file were set or measured under
the earlier 30/45/55/57/58/60 s policy and have not been re-run.

## Prefix-search validation (`pb2` / `pb3`)

Measurement tooling that validates the standalone `route_prefix_search.py` with
the real chain (state lattice, swept collision, compression, STM generation).
It changes no planner behaviour and adopts nothing. The tooling is described
first; the recorded Quick and Standard results and the gate states follow in
"Recorded validation baseline" below, and the full evidence is in the generated
summary.

### Three different stops (never conflated)

| Stop | What it is | In a record |
|---|---|---|
| Planner checkpoint | `--checkpoint-s`: the search stops itself between legs and returns honest metadata; a running leg is never interrupted, so it can overshoot | `validation.raw.stop_reason = wall_clock_checkpoint_limit` |
| Harness hard timeout | The isolated worker is killed at its `--timeout` | `status: "timeout"`, `cleanup.timeout_kind = "harness_hard_timeout"`, every planner and validation field `null`. Never read it as planner metadata or as a no-route result |
| Production hard backstop | Implemented only as the standalone `bounded_prefix_planning.py` (the parent kills the worker process tree at `R0` + 45 s); not adopted by `/plan/v2` | `bpb2`/`bpb3` records: `bounded.plan.termination = hard_backstop` (see "Bounded-executor validation") |

Correctness runs (exactness, determinism, proof, unlimited search) carry no
planner checkpoint and no work limit (`--checkpoint-s` defaults to `None`); the
hard timeout is their only containment. Work limits (`--max-leg-calls`,
`--max-expansions`, `--max-nodes`, `--max-frontier`), `--checkpoint-s` and
`--cancel-after-legs` are supplied explicitly, only for budget, cancellation,
frontier and overshoot experiments. They are operator flags of this tool, never
HTTP or client fields.

### Modes and rules

`--mode pb2 pb3` runs `find_prefix_b2_route` / `optimise_prefix_b3_route` with the
real leg planner (timed by the existing `RouteSearchStats.plan_leg`; per-depth
statistics are not meaningful for these modes). `--repeat K` runs K separate
worker invocations. Hard timeouts: always finite and at most 600 s; 60 s by
default up to 3 obstacles and an explicit `--timeout` from 4 obstacles. The old-search
rules above are unchanged (old-search campaign cases never exceed 120 s).
Prefix flags on `b2`/`b3`/`legs` are refused. Exit codes: 0 ok, 1 error record,
2 refused, 3 aborted (a worker's death could not be verified), 4 replay,
metadata or bound-audit violations, 5 campaign session incomplete (a required
case did not fit the session budget), 6 provenance refusal (see "Provenance" below).

### Normalized record and replay

`normalized.py` maps both `ExecutableRouteResult` and `PrefixSearchResult` to one
plain dict (order, skipped ids, candidate indices into the safety-filtered
tuples, legs with exact `MOV` commands and poses, totals, and a hash over the
same fields as `route_hash()`), and keeps the raw result metadata alongside.
`route_replay.py` works only on that dict and checks: command legality and exact
wire strings, exact pose chaining, re-propagation, collision safety of every
swept path, goal tolerance and candidate identity, and accounting. Distance is
the unweighted sum of `primitive_length_cm`; cost applies `reverse_penalty` only
to `B`/`BL`/`BR`; each is checked independently and the route totals are the
exact ordered sums of the leg totals. `route_replay.invariant_violations()`
checks the prefix result's proof, wording and B3-not-worse-than-B2 guarantees.

Timing fields are separate: `planning_time_s` (search only), `validation.timing.normalize_s`
(worker, after the search), `wall_time_s` (worker spawn to exit), `validation.timing.replay_s`
(parent, after the worker exits) and, from `report`, `report_s`. Replay never
inflates planner timing or the worker's timeout budget.

### Process containment

Each case runs in its own worker. On Windows it is assigned to a Job Object with
kill-on-close and a per-process memory cap (2 GiB), so it dies if the harness
does; on POSIX it gets its own session. At the timeout the worker is killed and
its death is verified through its own handle; an unverifiable death is an error
that aborts the whole campaign. `cleanup` records `containment`, `killed`,
`death_verified`, `exit_code`, `job_closed` and `tempdir_removed`. Cases run one
at a time (concurrent runs disturb timings). To check for leftovers, list only
processes that belong to this work:

```powershell
Get-CimInstance Win32_Process -Filter "Name like 'python%'" |
  Where-Object { $_.CommandLine -match 'bench_executable_planner|SC2079_MDP_T7-optimizer' } |
  Select-Object ProcessId, CommandLine
```

Never terminate, or assert the absence of, unrelated Python processes. The
bounded modes do not use this listing at all (see "Bounded-executor validation").

### Campaigns

Worst case per session = sum(hard timeout x repeats) + 15 s x worker invocations
(spawn/import, normalization, replay, kill verification and cleanup). Standard and
extended caps are per session, resumable with `--resume`; the driver refuses to
start a case that does not fit the remaining session budget and reports it
`missing`. `campaign NAME --dry-run [--session S|all]` prints this arithmetic and
runs nothing.

| Session | Timeouts x repeats | Invocations | Allowance | Worst case | Cap |
|---|---|---|---|---|---|
| quick `Q` | 470 s | 11 | 165 s | 635 s | 720 s |
| standard `S1` | 2,940 s | 21 | 315 s | 3,255 s | 3,600 s |
| standard `S2` | 3,300 s | 10 | 150 s | 3,450 s | 3,600 s |
| standard `S3` | 1,800 s | 3 | 45 s | 1,845 s | 3,600 s |
| extended `E1` | 3,000 s | 5 | 75 s | 3,075 s | 3,600 s |
| extended `E2` | 3,000 s | 7 | 105 s | 3,105 s | 3,600 s |
| extended `E3` | 2,760 s | 6 | 90 s | 2,850 s | 3,600 s |
| bounded `BQ` | 180 s | 2 | 30 s | 210 s | 720 s |
| bounded `B1` | 1,440 s | 16 | 240 s | 1,680 s | 3,600 s |
| bounded `B2` | 1,530 s | 17 | 255 s | 1,785 s | 3,600 s |
| bounded `B3` | 690 s | 4 | 60 s | 750 s | 3,600 s |

The standard sessions (`S1`-`S3`, 8,550 s in total) are the minimum evidence set:
obstacle counts 1, 2, 3, 4, 6 and 8, unsafe primary, no safe pose, reversal,
budget, cancellation, frontier cap and checkpoint overshoot, alternatives on,
hotspots at 3-4 obstacles, and old-versus-prefix exactness. Each case accepts a
complete route (`C`), an honest partial or no-route result (`P`) and/or a
recorded harness timeout (`T`, evidence of the attempt only; the route result
stays `not_measured`). Invoking a campaign once does not make its coverage
complete: coverage comes from the manifest and is `complete`, `partial` or
`not_started` per session, requirement and campaign. cProfile is only asked at
3 obstacles, because it can double runtime and time out at larger sizes.

### Evidence summary and adoption gates

`python benchmarks/bench_executable_planner.py report --input RAW.jsonl ... --summary-out
benchmarks/results/prefix_validation_summary.json` writes the compact tracked
summary: environment, the SHA-256 of every raw input file, stable case IDs with
status, result hash and key work, timing, memory and proof fields, coverage, gate
verdicts, comparisons, overshoot and the work calibration. Raw and per-leg JSONL
stay outside the repository. Citations of evidence use those case IDs. With no
input it writes the empty initial structure. The committed file was generated
from the four accepted raw files listed under "Recorded validation baseline"; it
is a **historical** summary (below).

**Provenance.** Evidence is one exact clean commit. `report` refuses, before it
evaluates any coverage or gate, records with a missing or malformed `git.sha`
(not 40 lowercase hex digits), a `git.dirty` that is not exactly `false`,
records from more than one SHA, or an invocation recorded twice (`Q5#1` and
`Q5#2` are distinct invocations, not duplicates). It then exits 6, prints
the reasons to stderr and writes nothing: no summary, no partial file, no
directory. `campaign` (not `--dry-run`) refuses to start, exit 6, when the commit
is unknown or the tree is dirty (untracked files count), and refuses `--resume`
when the existing file holds anything `report` would refuse or any other commit.
The Git state is read once, when the session starts: check that HEAD and the
tree are unchanged after each session. The committed summary predates this rule:
its records came from two commits (`3324d97`, `0c4e328`), so the current `report`
would refuse them and cannot regenerate it. Keep it as history until a
single-commit summary replaces it.

**Exactness eligibility (G2).** A comparison counts only when the prefix route is
certified. For a B3 pair (all but `R34`/`R33`) that means a complete route with
`optimality_proven`, stop `exhausted` or `bound_closed`, no work limit hit and
zero unresolved branches and bound violations. The B2 pair `R34`/`R33` asks for a
complete route no budget stopped, and no proof. Anything else is `not_certified`:
not a pass and not a failure, and G2 stays `not_measured` with the pair named.
So a faster old search (for example `R13` finishing) cannot turn the
work-limited `R12` into a false failure. These remain failures: a complete,
replay-valid prefix route cheaper than the old route, and a mismatch no budget
stop explains. A prefix run stopped by a budget (`work_limit`, `cancelled`,
`wall_clock_checkpoint_limit`) is never certified.

| Gate | Requirement | Status |
|---|---|---|
| G1 correctness | zero replay, metadata and bound-audit violations, zero error records, zero unverified worker deaths and zero temporary directories left behind (`cleanup.tempdir_removed == false` fails; `job_closed == false` does not, since the fallback and POSIX paths use no Job Object), minimum evidence set complete | `pass` |
| G2 exactness | where the old search completed (pairs R3/R2, R6/R5, R9/R8, R21/R20, R25/R24, R34/R33 and the four-obstacle R13/R12; a missing pair keeps the gate `not_measured`): complete routes need prefix B3 proven and bit-equal in cost, and completeness must agree; partial routes must agree bit-exactly (`repr`) in BOTH `total_distance_cm` and `total_cost`, and missing partial accounting fails. Old-search hard timeouts are excluded. A pair whose prefix route is not certified (see "Exactness eligibility") is `not_certified` and keeps the gate `not_measured` | `pass` |
| G3 determinism | separate invocations give identical hash and work at 3, 6 and 8 obstacles | `not_measured` |
| G4 coverage | the representative spread fixture (unlimited pb2, alternatives off) completes at each obstacle count from 4 through 8 (`R11`, `X12`, `R14`, `X18`, `R16`), and the designated infeasible fixtures (`R22`, `R23`) are proven impossible. It does not claim every possible 4-8 obstacle layout was validated | `not_measured` |
| G5 timing | bounded campaign, production policy: every 4-8 `A_spread` representative (BR1-BR10) delivers a complete route on time, BR1/BR3/BR5 repeat (identical B2 seed; identical final route when `search_finished`), and every production plan outcome (route, no route or failure) returns by `R0` + 58 s with no harness timeout | `not_measured` |
| G6 overshoot | bounded campaign, production policy: every hard-stop decision by `R0` + 45.5 s, and at least one BR/BX run (no stall, no soak) whose recorded `decided_s` is after the 30 s target, whatever its termination label | `not_measured` |
| G7 memory | peak RSS at most 1 GiB on the 8-obstacle alternatives-on run; frontier cap honest | `not_measured` |
| G8 metadata | budget, cancellation, frontier and checkpoint stops each observed with clean metadata | `pass` |

A gate is `pass` only when recorded evidence supports it, `fail` only on
counter-evidence, and `not_measured` otherwise. A failure is reported, never
hidden. The Status column is the state in the committed summary (a test keeps
the two identical); the evidence behind it follows.

### Adoption policy (bounded `/plan/v2` use)

Live adoption of the prefix search is a separate change. This policy says which
evidence it needs; it adopts nothing by itself.

* **Mandatory, each `pass`:** G1 correctness (replay, swept-collision safety,
  bound audit, cleanup), G3 determinism, G4 4-8 obstacle coverage, G5 timing and G6
  overshoot (they need the hard backstop), G7 memory and G8 metadata.
* **G2 exactness: no `fail`.** `not_measured` is acceptable only for the
  four-obstacle pair.
* **G2 may stay `not_measured`, and is then never a pass.** Every comparison must be
  recorded, the six other pairs `ok` or `excluded`, and R13/R12 (a supplementary
  historical comparison, never certified four-obstacle exactness) `ok`, `excluded` or
  `not_certified`. Any `fail` (a replay-valid prefix route cheaper than the old
  route, a cost that is not bit-equal, a completeness mismatch no budget explains)
  blocks adoption. No report may relabel an uncertified route as exact or describe
  G2 as passed while a pair is `not_certified`.
* **Why four obstacles stay `not_measured`:** expansion-limited state-lattice legs
  leave unresolved branches whose certified lower bounds are below the incumbent, so
  optimality cannot be proven (next section). The 20,000-expansion cap, the 600 s
  timeout and the swept-collision checks are not relaxed to change that.
* **Production wording.** When `optimality_proven` is not true, the route is the
  "best complete safe route found within budget". It is never called optimal,
  shortest, exact or proven, and its unresolved count stays visible. Even a proven
  route is optimal only for the planner's cost model and candidate poses, never a
  time or fastest-route claim.
* **A certified pair can still be added later** (for example after a separate
  lower-bound or lattice change); until one exists the statements above hold.
* The committed summary stays historical (provenance rule above): its `pass` for G2
  predates the eligibility rules and is not evidence for this policy.

### Recorded validation baseline (Quick + Standard MES)

**Machine-specific and indicative only:** Windows 11, 12 logical CPUs (Intel64
Family 6 Model 186), CPython 3.11.5, one machine, sequential runs, tooling at
commits `3324d97` and `0c4e328`. Timings differ by roughly +/-15% between
identical invocations. The raw JSONL stays outside Git; the tracked summary
records each input's name and SHA-256:

| Raw file | Records | SHA-256 |
|---|---|---|
| `quick-fixed.jsonl` (Quick) | 11 | `12bc36008c35a9bf28dfca8cb23edd54f21d7a85ff172344b8177a0dcca9a24c` |
| `standard-s1.jsonl` (S1) | 21 | `6a1c5d889cbfd8b64ccbfb41152dc0e15bf6775a59802d98d8d001e9348264e4` |
| `standard-s2.jsonl` (S2) | 10 | `b4b39af1f0251a92229fa6f48c8f4ae85f7d6370d688af7d82f038acf0f571ed` |
| `standard-s3.jsonl` (S3) | 3 | `a3bce21b690e611713d12d25c8ecac0a057762302c5052526727b266d674e2f3` |

A first Quick run was superseded (a partial B2 seed was mislabelled; fixed in
`3324d97`) and is not an input. Coverage: Quick, S1, S2 and S3 are complete and
the Standard minimum evidence set is complete, from 45 accepted records; 18
extended invocations (`X1`-`X18`) are `missing`. Every returned route passed
replay and metadata checks (no violations, no error records), and every worker
death and temporary-directory removal was verified.

* **G1 pass, G8 pass.** G8 saw a leg-call budget, a cancellation, a frontier cap
  and a checkpoint stop, each with honest metadata.
* **G2 pass (historical):** six old-versus-prefix pairs agree (R3/R2, R6/R5, R9/R8,
  R21/R20, R25/R24, R34/R33). R13/R12 is *excluded*: R13 (old B3, four obstacles) hit its
  120 s hard timeout, so four-obstacle exactness has not been demonstrated.
  An exploratory run after the collision optimizations (not accepted evidence, not a
  `report` input) finished R13 in about 105 s on this machine (indicative, one run), so a
  new R13 would be compared with R12. R12 is work-limited and unproven, hence
  `not_certified`, and a campaign that records it yields G2 `not_measured`, never a
  pass (see "Four-obstacle exactness discovery" and the adoption policy). A bounded,
  declared search for a certifiable four-obstacle case found none, and the campaign
  is no longer held back for one.
* **G3 not measured:** the 3-obstacle (R8/R10) and 8-obstacle (R16/R17) pairs
  gave identical hashes and work, but the 6-obstacle pair needs `X1`.
* **G4 not measured:** four, six and eight obstacles complete and both
  infeasible fixtures are proven impossible, but five (`X12`) and seven (`X18`)
  are missing.
* **G5, G6 not measured (in this historical baseline):** no bounded campaign was part of it;
  see "Bounded-executor validation" below for the later BQ/B1/B2/B3 results.
* **G7 not measured:** it needs `X13`, the 8-obstacle alternatives-on memory
  run (`R28` is recorded at 84 MB peak RSS at four obstacles).

| Case | Result | Planning time |
|---|---|---|
| `R11` four obstacles, B2 | complete, cost 400.2, 15 leg calls (5 expansion-limited) | 172.7 s |
| `R12` four obstacles, B3, 30-leg-call budget | complete, same route as B2, not proven (8 unresolved), search exhausted | 536.6 s |
| `R14` six obstacles, B2 | complete, cost 527.0, 6 legs | 38.1 s |
| `R16` / `R17` eight obstacles, B2, two invocations | complete, cost 637.5, identical hash and work | 55.5 s / 48.0 s |
| `R18` eight obstacles, B3, 16-leg-call budget | complete, same route as B2, not proven (36 unresolved) | 140.8 s |
| `R29` four obstacles, B2, 10 s checkpoint | partial (3/4), checkpoint stop | 43.3 s |

* **Checkpoint overshoot:** the 10 s planner checkpoint stopped the search at
  43.3 s, 33.3 s late, because the leg in progress took 34.7 s. The checkpoint
  is only checked between legs; it is not a hard backstop.
* **Bottleneck:** legs that end in `expansion_limit` (about 23-47 s each) dominate
  four-obstacle time; successful legs average about 6-7 s. Layout matters more
  than obstacle count (eight obstacles took 55 s, four took 173 s).
* **Hotspots:** collision checking dominates. On the profiled `B_clutter_04` run
  (`R31`) `safe_primitive_samples` accounted for about 98% of instrumented time
  and `footprint_hits_obstacle` for about 65% over about 11.1 million calls
  (timers nest, so shares overlap). A cProfile run at three obstacles (`R32`)
  puts `footprint_hits_obstacle` at about 59% cumulative and `footprint_corners`
  at about 34%. These are inputs for a later optimization PR; nothing was
  optimized here.
* **Not ready for adoption:** the current backend is not ready for live 4-8
  obstacle use. G3, G4 and G7 lack evidence, G5 and G6 need a bounded-executor
  campaign, and unbounded four-obstacle planning takes minutes.

## Bounded-executor validation (`bpb2` / `bpb3`)

Measurement tooling for the standalone `bounded_prefix_planning.py`
(`benchmarks/bounded_modes.py`); it changes neither the executor nor the search.
Bounded sessions BQ/B1/B2/B3 were run on clean commit `9e86afb` (the first B2
file was invalidated by Modern Standby and excluded; the clean rerun replaced
it). BQ, clean B2 and B3 passed every case/check. B1's BR1 repetitions and BR6
failed only AVAIL on `A_spread_04`; all 5-8 `A_spread` representatives passed.
(`D_seeded_04_s1`, BX2, also returned an honest no-route in B2, which its
acceptance rule allows.)
Thus G5 is `fail` and G6 `pass`; the original 4-8 adoption policy is not met.
The external accepted files are `bounded-bq-9e86afb.jsonl`
(`ea11ea4514e3eb7bc611113c53d63a40e8eae56316d0c6c3cb5222d48a1002b6`),
`bounded-b1-9e86afb.jsonl`
(`933014fc1b7eaf2ded2fff87c99276acc4954faf96d415c3bac57d8bb8c241a9`),
`bounded-b2-clean-9e86afb.jsonl`
(`55e6389ef2d69d4431ca4005117fd72932ee62efe9d978dce351832915171fd1`)
and `bounded-b3-9e86afb.jsonl`
(`466c2846983ca1a87b47dd80f4702fd67f282c30d42998bf0f586acdb36f2a51`). No tracked summary
was written because the historical standard inputs cannot be regenerated under
the current single-SHA provenance rule.

* **One plan per invocation** (BS5: eight on one executor instance). `R0` is the
  bench worker's `time.monotonic()` just before `plan()`: the prefix worker's start-up
  counts, the bench worker's own start-up does not.
* **Policies** are server-side names, never request fields: `production` (30/45/55/57/58/
  60 s) and `stress` (5/10/15/17/18/20 s). Only production records count for G5/G6.
* **Stalls** (`stall`) replace the worker command, through the executor's own
  `worker_command` seam, with the test worker `tests/_bounded_test_worker.py real` (the
  real search and leg planner): `at_call_1` stalls the first leg call (a guaranteed kill,
  no incumbent), `before_final` withholds FINAL after the search (a guaranteed hard stop
  that must keep the incumbents).

| Session | Cases |
|---|---|
| `BQ` | BQ1 `A_spread_02` bpb3 production; BQ2 `A_spread_02` bpb3 stress, stall `at_call_1` |
| `B1` | BR1/BR3/BR5 `A_spread_04`/`06`/`08` bpb3 x3; BR2/BR4 `A_spread_05`/`07` bpb3; BR6-BR10 `A_spread_04`-`08` bpb2 (production) |
| `B2` | BX1-BX17: `B_clutter_*`, `D_seeded_*`, `C_unsafe_primary_05`/`08` and `A_spread_08` (alternatives on), `C_no_safe_pose_05`/`08` (bpb3, production) |
| `B3` | BS1 `A_spread_02` production, stall `before_final`, x2; BS2 `A_spread_04` stress; BS5 `A_spread_02` stress, stall `at_call_1`, soak of 8 plans |

**Record (`bounded` block).** `policy` (name and every value), `stall`, `soak_plans`,
`r0_anchor`, `planner_mode`, `harness` (the harness job's active processes before it was
closed, committed-memory diagnostics, the sentinel), `replay_s`, and `plan` or
`soak` (`handle_count_before`, `thread_count_before`, `rows`). A plan holds the
executor's termination, route status and source, delivery and `deliverable`, the three
proof flags, incumbents, verification failures, message, planner metadata, all `Timing`
offsets and `overshoot_s`, the executor `Cleanup`, `routes` (`route`, `b2_route`,
`diagnostic_route`: hash, normalized record, parent-side replay violations),
`b3_not_worse_than_b2`, `tree` (captured workers, worker PID and handle-verified exit,
the harness job's process count during and after the plan, leftover `prefix_plan_*`
folders), `memory` (the worker's peak RSS and whether it was read after exit) and
`healthy_after`; soak rows add `handle_count`, `thread_count` and `rss_bytes`. The
executor host's peak RSS is `peak_memory.peak_rss_bytes`. The harness job's 2 GiB per-process
memory cap (`process_control.DEFAULT_MEMORY_CAP_BYTES`) is a per-process limit that Windows
applies to every process in that job, the nested prefix worker included (documented Windows
behaviour, not separately tested here); the committed-memory readings stay diagnostic and never
gate.

**Checks** (`report.bounded_checks`): SAFE (every retained route replays clean, no
verification failure, status not error/timeout), RETAIN (never costlier than the B2
seed; a received incumbent is never lost), CLEAN (executor death, tree, reader, job and
tempdir; handle-verified worker exit; harness job back to 1 then 0; no leftover folder;
the sentinel survived and its own termination was verified), HONEST (no proof flag or
proof wording after a bounded stop; an ordinary no-route bounded stop (cleanup verified, no
route withheld) says "within budget", while the executor's cleanup-failure and late-route
wordings are left to CLEAN/G1 and RETURN/G5; a
`search_finished` claim meets the executor's own rules: optimality needs B3, a delivered
FINAL route, a closed stop, a certified bound, no bound violation and either zero
unresolved branches or a known minimum unresolved bound >= the route cost; exhaustion
needs a closed stop with zero unresolved; impossibility needs no retained route and an
empty candidate group or an exhaustive search with zero unresolved; unverified cleanup
allows no flag), RETURN (every production plan outcome returns by its policy deadline;
a missing time fails), MEM (host and worker peak RSS each <= 1 GiB; a
worker peak is `after_exit` only when the worker's own handle confirms it exited, a reading
of a still-running worker is kept as `last_sample` and is never a pass; job committed memory
never gates),
AVAIL, REPEAT, NESTED (BQ), SOAK (BS5: healthy, threads back to baseline, handles grow
by at most 20 from plan 1 to plan 8, every plan `hard_backstop`) and per-case
expectations. They feed G1 (SAFE, RETAIN, CLEAN, NESTED, SOAK, expectations, and a
stress decision later than its hard stop + 0.5 s, for BQ2 and BS2 but not for the BS5 soak
rows, whose decision times are only reported), G5, G6 (table above), G7 (MEM) and
G8 (HONEST); a record without a `bounded` block is judged as before. The runner exits 4
on a failing check and 3 if the harness job still held a process before it was closed.

**Cleanup without process listing.** Only processes the harness started are observed:
the bench worker (its `Popen` handle and its Job Object's own count), the prefix worker
(the exact `Popen` captured from `worker_containment.start_worker`, and the nested job
counted in the harness job), and one sentinel per invocation, the only process the
harness itself terminates, through its own handle. Nothing enumerates system processes.

**Summary.** Each bounded case entry carries its facts, checks and a `verdict`; the
summary's `bounded` section holds REPEAT results (a differing final route after a budget
or hard stop is reported as `wall_clock_variation`, not failed), production overshoot
and memory (committed memory labelled diagnostic). The tracked summary predates this
(summary schema 1, manifest 1) and stays historical until an evidence PR.

## Four-obstacle exactness discovery (NO-GO)

**Result: no certifiable four-obstacle pair was found, so four-obstacle G2 stays
`not_measured`.** A bounded search over six declared candidates `E_ordering_04_c1`-`c6`
(`benchmarks/make_ordering_fixtures.py`, rules fixed before any run in
`benchmarks/ordering_discovery.py`) asked for a prefix B3 run that is complete and
certified (proven, `exhausted`/`bound_closed`, no limit, 0 unresolved, 0 violations)
within 150 s, plus an old B3 that finishes within 90 s with a bit-equal cost. Every
candidate failed the first stage. The ledger
`benchmarks/results/ordering_fixture_discovery.json` records every attempt. It is
generated from `discovery-primary.jsonl` (6 records, SHA-256
`8724ca3096d0b5db213334f5d3dd4083c22b3d23c3613dd17e84434e6d36d415`), run from clean commit
`e1e28f3c64c65a6b47ddd1495bfe4a8bd685023e`, unlimited, alternatives off, 300 s hard timeout,
about 229 s in total. Raw JSONL stays outside Git.

**Voided run.** A first c1 invocation at `3c2c9b6` was discarded before any result was judged:
the evaluator read the runner's default frontier cap (`max_frontier_entries=200000`) as an
operator limit and refused the record. `e1e28f3` fixed the check, every candidate was run
from `e1e28f3`, and the voided file (SHA-256
`9ba1527e177bdb08e04800fe6f6deba45a0b616c0e15b77d5dededcd62035119`) is listed in the ledger.

| Candidate | Wall | Expansion-limited legs | Unresolved | Complete orders | Route cost (B2 seed) |
|---|---|---|---|---|---|
| `c1` | 58.9 s | 5 | 5 | 3 | 492.88 (519.58) |
| `c2` | 22.1 s | 4 | 4 | 1 | 495.89 (same) |
| `c3` | 65.4 s | 12 | 7 | 1 | 477.49 (same) |
| `c4` | 16.4 s | 2 | 2 | 1 | 444.28 (same) |
| `c5` | 20.9 s | 2 | 2 | 3 | 393.90 (413.66) |
| `c6` | 45.0 s | 4 | 4 | 3 | 453.18 (same) |

* **Verified from the records:** every run was complete, replay-clean and free of bound
  violations; none set `optimality_proven`; every unresolved branch count equals
  (or, for `c3`, is below) the number of expansion-limited leg calls, and in each run the
  smallest unresolved certified lower bound is below the incumbent (for example `c1`
  about 402 against 492.9, `c5` about 330 against 393.9; these two figures are in the raw
  file, not the ledger). Such a branch is unresolved because the state-lattice leg
  reached its 20,000-expansion cap: a cap stop proves nothing, and a bound below the
  incumbent cannot close the branch.
* **Route improvement and ordering were real, optimality was not proven.** `c1` and `c5`
  each found three complete orders and a cheaper route than their B2 seed (by 26.7 and
  19.8 cost units). `c6` found three orders with no improvement. Nothing here shows the
  returned routes are globally shortest.
* **Diagnostic, not evidence** (scratch `legs`-mode runs on the ideal start and ideal
  inter-target poses, not a route): 12 of 25 legs needing a 90 deg heading change hit the
  cap against 2 of 23 needing none. The heading rule of the candidate family did not keep
  legs under the cap. In a profiled run (`c3`) 1,171 of 1,176 arrivals inside the goal
  position tolerance were rejected on heading, and swept primitive sampling took most of the
  time.
* **Not done on purpose:** the expansion cap, the 600 s timeout and the swept-collision
  checks were not changed, and no further candidates or rule changes followed the result.
  Improving lower-bound accounting or the lattice heuristic are separate, unscheduled
  changes (either would need its own equivalence tests and a re-measurement).

## Per-leg obstacle geometry: measured effect (indicative)

Measured on the same machine as below, pb2 with alternatives off, alternating
before (`6332c61`) and after runs as separate contained workers, about 10-26%
CPU load at run start. Every before/after pair had an identical result hash (equal
to the Phase A golden hash), normalized route, cost, distance, work counters
and stop metadata, with zero replay or invariant violations. Median
`planning_time_s`, before -> after: `A_spread_03` 1.20 -> 1.07 s, `B_clutter_04`
17.1 -> 14.7 s, `A_spread_06` 3.91 -> 3.31 s, `A_spread_08` 3.81 -> 3.17 s
(three runs each; the ranges did not overlap). **`A_spread_04` (R11) is
inconclusive**: over eight runs each, after-runs ranged 29.0-48.6 s on
identical work (before: 32.7-39.4 s), so no effect is claimed for it. These are
single-machine, single-session numbers for these fixtures only; they are not a
general speed-up and say nothing about Task 1 mission timing.

## Caveats

* Numbers are machine-specific (record `machine`, `python`, `git`). Compare
  runs from the same machine and, ideally, the same load.
* Concurrent benchmark processes share the machine; use one at a time for
  timings you intend to compare.
* `wall_time_s` includes interpreter start-up and imports; use
  `planning_time_s` for the optimizer alone.
* A timeout means "not finished within the bound", not "no route exists".

## Measured baseline (development machine, current planner)

Windows 11, 12 logical CPUs, Python 3.11.5, planner as of `origin/main`
`d8c275a`. **Machine-specific and indicative only.** Rows marked `(idle)` ran on
an otherwise idle machine; the rest shared it with up to four other benchmark
processes and are noisier. Identical runs differ by up to about +/-20% in wall
time, but every result hash, route and counter was identical across two
independent runs. Costs are planner path cost, not seconds.

**Old permutation search, alternatives off (idle)**

| Fixture | B2 time / legs | B3 time / legs (orders valid/pruned) | Cost (B2 = B3) |
|---|---|---|---|
| `A_spread_01` | 0.13 s / 1 | 0.15 s / 1 | 60.4 |
| `A_spread_02` | 5.5 s / 2 | 14.5 s / 4 (1/1) | 149.6 |
| `A_spread_03` | 9.8 s / 3 | 44.8 s / 14 (1/5) | 216.8 |

**Bounded runs: every one of these hit its hard timeout, so nothing is known
about their completion time.** 4 obstacles, alternatives off, 120 s
(`A_spread_04` b2 and b3, `B_clutter_04` b2, `D_seeded_04_s1` b2). Alternatives
on: `A_spread_02` b2 103 s / 20 legs, cost 128.6 (alternatives off: 149.6),
b3 timeout at 180 s; `A_spread_03` b2 and b3 timeout at 180 s;
`C_unsafe_primary_02` b2 51 s, b3 151 s, cost 166.9. The old search was **not
run** at 6-8 obstacles (refused by design), so nothing here is a 6-8 obstacle
route time.

**Per-leg cost (`legs` mode, idle)** - legs start from ideal poses, so this is
per-leg cost, not a route.

| Fixture | n | legs planned (ok / expansion-limited) | expansions per leg | ms per expansion | s per leg |
|---|---|---|---|---|---|
| `A_spread_01` | 1 | 1 (1/0) | 226 | 0.58 | 0.13 |
| `A_spread_02` | 2 | 4 (4/0) | 3032 | 0.75 | 2.3 |
| `A_spread_03` | 3 | 6 (6/0) | 2125 | 0.94 | 2.0 |
| `A_spread_04` | 4 | 8 (6/2) | 8184 | 1.33 | 10.9 |
| `A_spread_08` | 8 | 16 (13/3) | 8439 | 2.82 | 23.8 |
| `B_clutter_04` | 4 | 8 (3/5) | 16195 | 1.85 | 30.0 |
| `D_seeded_04_s1` | 4 | 8 (3/4, 1 no path) | 13338 | 1.97 | 26.3 |

## Findings on the profiling questions (H1-H7)

* **H1 failed/limited legs.** Pooled over the idle leg runs, 14 of 51 legs
  (27%) ended at the 20,000-expansion limit and took **63%** of all leg time
  (20,000 expansions each versus about 5,000 on average for successful legs). Every
  leg of the completed 1-3 obstacle routes succeeded.
* **H2 collision.** `safe_primitive_samples` is 96-99% of leg time.
  `pose_is_safe` rises from 65% (n=1) to 73, 78, 82 and 88% (n=2, 3, 4, 8) and
  the SAT test `footprint_hits_obstacle` from 38% to 76% (wrapper timings;
  inflated, use the shares). `cProfile` at n=3 agrees (77% and 59%) and shows
  `footprint_corners` called 1.53 million times for 1.13 million SAT tests
  (35% of time): corners are recomputed for every obstacle. (A `cProfile` run
  at 6 obstacles hit the 600 s cap and produced no table.)
* **H3 scaling.** Cost per expansion grows roughly linearly with obstacle
  count, about 5x from 1 to 8 obstacles (0.58 to 2.82 ms). Leg difficulty
  varies far more than that: 0.13 s to 30 s per leg. A 16-leg set at 8 obstacles
  took 381 s.
* **H4 heading.** Legs needing a heading change of 150 deg or more averaged
  about 17,200 expansions and failed 7 of 11, versus about 2,800 and 1 of 12
  for changes under 30 deg. Goal-zone churn itself is small: only 1.8-10% of
  goal tests (1.8% in the 4- and 8-obstacle leg runs) were inside the 5 cm
  position tolerance, although 96-99.5% of those failed on heading. Heading
  matters mainly through how far the search must wander to arrive facing the
  right way, not through time spent at the goal.
* **H5 cache.** Hit rates were 0-27%, only at the first two chain depths
  (`A_spread_03` b3: 3 of 17 requests, all at depth 0). With alternatives the
  hit rate was 0%: legs from different estimated poses never coincide.
* **H6 pipeline.** Compression plus command generation is 0.02-0.6% of leg time
  (under 0.1% for most runs); the search outside the leg planner is under 0.05%.
* **H7 B2 versus B3.** Wherever both finished, they returned the same cost (five
  comparisons). B3 took 1x to 5x longer (`A_spread_03`: 45 s versus 9.8 s). There
  is no B3 improvement in this data, and it cannot be generalised beyond
  three obstacles.
