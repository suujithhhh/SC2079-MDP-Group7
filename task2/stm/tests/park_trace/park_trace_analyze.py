#!/usr/bin/env python3
"""Offline analyser for USPARK,TRACE lines in a SAVED STM32 UART log.

READ-ONLY: this script only reads the log files named on the command line. It
never opens a serial port, never sends anything to the robot and has no effect
on control. (A unit test checks that it does not import a serial library.)

Input: the monitor's log (``<timestamp>\\tRX\\t<text>``), the same lines as
printed to a console (``<timestamp> RX <text>``) or bare lines. Only RX lines
(or bare lines) are considered; TX and MARK lines are ignored, except that the
last MARK text before a run is shown as that run's label.

What it checks, per run (one run = ``EV:PRE``/``EV:ARM`` ... ``EV:DONE``):
  * every line parses and has the fields of its event;
  * record index N counts 1, 2, 3, ... with no gap (a gap means a lost line);
  * the events appear in the required order and exactly once each
    (ARM, REQ, ACC, TAKE, BRK, BEND, ZL, ZR, ZERO, DONE);
  * ticks never run backwards (wrap-safe 32-bit);
  * ultrasonic publication numbers (SEQ, 16-bit) have no gap;
  * nothing was dropped (DONE DR/RF), no timeout (ZERO TO, DONE ZT/R), no
    ultrasonic fault (ARM FM), the stop was a NEAR stop, the profile is T2T.
A run that fails any check is REJECTED and excluded from the statistics.

What it reports, per accepted run and as median / worst / spread across runs:
  sensor-to-request, request-to-accept, accept-to-takeover and
  takeover-to-brake latency; brake-to-zero time per wheel and total; total time
  from the request to confirmed zero; encoder counts travelled after REQ, TAKE
  and BRK until ZERO (left, right, mean); millimetres only with
  --counts-per-cm.

IMPORTANT definitions (see Core/Inc/app_park_trace.h):
  * Ticks are RTOS ticks, 1 tick = 1 ms.
  * Encoder counts on US/PRE lines (OEL/OER) were read when the firmware
    OBSERVED the publication, at tick T + OD. They are observation-time counts,
    NOT capture-time counts; this script never uses them for travel and prints
    them as such.
  * ZL/ZR/ZERO are CONFIRMATION times: a wheel is stationary the first time its
    counts vary by at most one count over a 50 ms rolling window, so the wheel
    actually stopped up to 50 ms earlier. Times and counts here include that
    confirmation window.
  * Right-encoder values are 16-bit (wrap-safe differences); the left encoder
    is 32-bit.

Exit status: 0 = every run accepted, 1 = at least one run rejected, 2 = no
accepted run / unreadable input / bad arguments.
"""

import argparse
import csv
import math
import re
import statistics
import sys

TICK_MOD = 1 << 32
SEQ_MOD = 1 << 16

# event -> (required keys); keys whose value may be NA are listed separately.
REQUIRED = {
    "US":   ["N", "T", "OD", "SEQ", "RAW", "FILT", "AGE", "PU", "OEL", "OER"],
    "PRE":  ["N", "T", "OD", "SEQ", "RAW", "FILT", "AGE", "PU", "OEL", "OER"],
    "ARM":  ["N", "T", "W", "SEQ", "FILT", "CN", "P", "FM", "EL", "ER"],
    "NEAR": ["N", "T", "K", "SEQ", "FILT", "EL", "ER"],
    "REQ":  ["N", "T", "R", "SEQ", "PD", "EL", "ER", "VL", "VR"],
    "ACC":  ["N", "T", "EL", "ER"],
    "TAKE": ["N", "T", "EL", "ER"],
    "BRK":  ["N", "T", "EL", "ER", "VL", "VR"],
    "BEND": ["N", "T", "EL", "ER"],
    "ZL":   ["N", "T", "EL", "ER"],
    "ZR":   ["N", "T", "EL", "ER"],
    "ZERO": ["N", "T", "EL", "ER", "TO"],
    "DONE": ["N", "T", "W", "R", "DR", "RF", "ZT", "SK"],
}
NA_ALLOWED = {"RAW", "FILT", "AGE", "PU", "PD", "VL", "VR"}
TEXT_KEYS = {"R", "P"}           # string valued; everything else is an integer
HEX_KEYS = {"FM"}

# Required order of the single-instance events inside a run.
CHAIN = ["ARM", "REQ", "ACC", "TAKE", "BRK"]
ONCE = ["ARM", "REQ", "ACC", "TAKE", "BRK", "BEND", "ZL", "ZR", "ZERO", "DONE"]

PROFILES = {"PREC", "STD", "T2T", "CUST"}

# metric name -> (unit, description); order = report order
TIME_METRICS = [
    ("sensor_to_request_ms", "ultrasonic publication -> STOP requested"),
    ("request_to_accept_ms", "STOP requested -> CommTask accepted it"),
    ("accept_to_takeover_ms", "accepted -> MotionTask consumed it"),
    ("takeover_to_brake_ms", "consumed -> electrical brake starts"),
    ("brake_to_zero_left_ms", "BRK -> left wheel confirmed stationary"),
    ("brake_to_zero_right_ms", "BRK -> right wheel confirmed stationary"),
    ("brake_to_zero_ms", "BRK -> both wheels confirmed stationary"),
    ("request_to_zero_ms", "STOP requested -> both confirmed stationary"),
    ("brake_duration_ms", "BRK -> BEND (the existing 120 ms brake)"),
    ("zero_minus_bend_ms", "ZERO - BEND (negative: zero confirmed first)"),
    ("zero_lr_diff_ms", "ZL - ZR confirmation time difference"),
]
COUNT_METRICS = [
    ("counts_after_req_left", "left counts, REQ -> ZERO"),
    ("counts_after_req_right", "right counts, REQ -> ZERO"),
    ("counts_after_req_mean", "mean of both, REQ -> ZERO"),
    ("counts_after_take_left", "left counts, TAKE -> ZERO"),
    ("counts_after_take_right", "right counts, TAKE -> ZERO"),
    ("counts_after_take_mean", "mean of both, TAKE -> ZERO"),
    ("counts_after_brk_left", "left counts, BRK -> ZERO"),
    ("counts_after_brk_right", "right counts, BRK -> ZERO"),
    ("counts_after_brk_mean", "mean of both, BRK -> ZERO"),
    ("counts_brk_lr_diff", "left - right counts, BRK -> ZERO"),
]
INFO_METRICS = [
    ("speed_at_req_left_cps", "left counts/s at REQ"),
    ("speed_at_req_right_cps", "right counts/s at REQ"),
    ("speed_at_brk_left_cps", "left counts/s at BRK"),
    ("speed_at_brk_right_cps", "right counts/s at BRK"),
    ("us_filtered_mm_at_request", "filtered mm of the triggering publication"),
    ("us_age_ms_at_observation", "measurement age when the publication was seen"),
    ("us_observation_delay_ms", "publication tick -> CommTask observed it"),
]


def d32(a, b):
    """Signed wrap-safe difference b - a of two 32-bit counter values."""
    return ((b - a + (1 << 31)) % (1 << 32)) - (1 << 31)


def d16(a, b):
    """Signed wrap-safe difference b - a of two 16-bit counter values."""
    return ((b - a + (1 << 15)) % (1 << 16)) - (1 << 15)


def tick_delta(a, b):
    """Unsigned wrap-safe b - a in ticks; None when it would be 'backwards'."""
    d = (b - a) % TICK_MOD
    return d if d < (1 << 31) else None


class ParseError(Exception):
    pass


def parse_trace_text(text):
    """'EV:US,N:3,...' -> dict. Raises ParseError on any malformation."""
    parts = text.split(",")
    if not parts or not parts[0].startswith("EV:"):
        raise ParseError("line does not start with EV:")
    ev = parts[0][3:]
    if ev not in REQUIRED:
        raise ParseError("unknown event %r" % ev)

    rec = {"ev": ev}
    for part in parts[1:]:
        if ":" not in part:
            raise ParseError("field without ':' (%r)" % part)
        key, value = part.split(":", 1)
        if key in rec:
            raise ParseError("duplicate field %s" % key)
        if value == "NA":
            if key not in NA_ALLOWED:
                raise ParseError("field %s may not be NA" % key)
            rec[key] = None
        elif key in TEXT_KEYS:
            if value == "":
                raise ParseError("empty %s" % key)
            rec[key] = value
        elif key in HEX_KEYS:
            try:
                rec[key] = int(value, 16)
            except ValueError:
                raise ParseError("field %s is not hex (%r)" % (key, value))
        else:
            try:
                rec[key] = int(value, 10)
            except ValueError:
                raise ParseError("field %s is not an integer (%r)" % (key, value))

    missing = [k for k in REQUIRED[ev] if k not in rec]
    if missing:
        raise ParseError("%s line is missing %s" % (ev, ",".join(missing)))
    extra = [k for k in rec if k != "ev" and k not in REQUIRED[ev]]
    if extra:
        raise ParseError("%s line has unexpected %s" % (ev, ",".join(extra)))
    return rec


LINE_SPLIT = re.compile(r"\t")


def extract_lines(path):
    """Yield (lineno, kind, text): kind is 'RX', 'MARK' or 'OTHER'."""
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.rstrip("\r\n")
            fields = LINE_SPLIT.split(line)
            if len(fields) >= 3 and fields[1] in ("RX", "TX", "MARK"):
                kind, text = fields[1], "\t".join(fields[2:])
            else:
                m = re.match(r"^\S+ (RX|TX|MARK) (.*)$", line)
                if m:
                    kind, text = m.group(1), m.group(2)
                else:
                    kind, text = "RX", line     # bare serial line
            yield lineno, kind, text.strip()


class Run:
    def __init__(self, path, label):
        self.path = path
        self.label = label
        self.records = []        # parsed dicts with 'lineno'
        self.errors = []
        self.warnings = []
        self.closed = False

    def first(self, ev):
        for r in self.records:
            if r["ev"] == ev:
                return r
        return None

    def name(self):
        first = self.records[0]["lineno"] if self.records else 0
        return "%s:%d%s" % (self.path, first,
                            (" [%s]" % self.label) if self.label else "")


def split_runs(paths):
    """Returns (runs, global_errors)."""
    runs = []
    global_errors = []
    for path in paths:
        current = None
        label = ""
        for lineno, kind, text in extract_lines(path):
            if kind == "MARK":
                label = text
                continue
            if kind != "RX":
                continue
            idx = text.find("USPARK,TRACE,")
            if idx < 0:
                continue
            body = text[idx + len("USPARK,TRACE,"):]
            try:
                rec = parse_trace_text(body)
            except ParseError as exc:
                msg = "%s:%d: malformed trace line (%s): %s" % (
                    path, lineno, exc, text)
                if current is not None:
                    current.errors.append(msg)
                else:
                    global_errors.append(msg)
                continue
            rec["lineno"] = lineno

            starts = rec["ev"] in ("PRE", "ARM") and rec["N"] == 1
            if starts:
                if current is not None and not current.closed:
                    current.errors.append(
                        "no DONE before the next window started (line %d)" % lineno)
                current = Run(path, label)
                runs.append(current)
            if current is None or current.closed:
                global_errors.append(
                    "%s:%d: %s record outside a trace window" % (
                        path, lineno, rec["ev"]))
                continue
            current.records.append(rec)
            if rec["ev"] == "DONE":
                current.closed = True
        if current is not None and not current.closed:
            current.errors.append("log ends before DONE (incomplete trace)")
    return runs, global_errors


def validate(run, profile, allow_non_near):
    recs = run.records
    errs = run.errors

    # N: contiguous from 1
    for i, r in enumerate(recs):
        if r["N"] != i + 1:
            errs.append("record index gap: expected N:%d, saw N:%d (line %d)"
                        % (i + 1, r["N"], r["lineno"]))
            break

    names = [r["ev"] for r in recs]
    for ev in ONCE:
        c = names.count(ev)
        if c != 1:
            errs.append("expected exactly one %s, found %d" % (ev, c))

    # order
    pos = {}
    for i, r in enumerate(recs):
        pos.setdefault(r["ev"], i)
    chain = [e for e in CHAIN if e in pos]
    for a, b in zip(chain, chain[1:]):
        if pos[a] > pos[b]:
            errs.append("%s appears before %s" % (b, a))
    if all(e in pos for e in ("BRK", "ZL", "ZR", "ZERO", "BEND", "DONE")):
        for e in ("ZL", "ZR", "BEND"):
            if pos[e] < pos["BRK"]:
                errs.append("%s appears before BRK" % e)
        if pos["ZERO"] < max(pos["ZL"], pos["ZR"]):
            errs.append("ZERO appears before both wheels were confirmed")
        if pos["DONE"] != len(recs) - 1:
            errs.append("DONE is not the last record")
        if pos["DONE"] < max(pos["ZERO"], pos["BEND"]):
            errs.append("DONE appears before ZERO and BEND")
    if "ARM" in pos:
        for i, r in enumerate(recs):
            if r["ev"] == "PRE" and i > pos["ARM"]:
                errs.append("PRE after ARM (line %d)" % r["lineno"])
            if r["ev"] in ("US", "NEAR") and i < pos["ARM"]:
                errs.append("%s before ARM (line %d)" % (r["ev"], r["lineno"]))
    if "REQ" in pos:
        for i, r in enumerate(recs):
            if r["ev"] == "NEAR" and i > pos["REQ"]:
                errs.append("NEAR after REQ (line %d)" % r["lineno"])

    # ticks of the event chain never run backwards (wrap-safe)
    prev = None
    for r in recs:
        if r["ev"] in ("US", "PRE", "NEAR"):
            continue
        if prev is not None and tick_delta(prev["T"], r["T"]) is None:
            errs.append("time runs backwards: %s T:%d after %s T:%d" % (
                r["ev"], r["T"], prev["ev"], prev["T"]))
        prev = r
    prev = None
    for r in recs:
        if r["ev"] not in ("US", "PRE"):
            continue
        if prev is not None and tick_delta(prev["T"], r["T"]) is None:
            errs.append("publication ticks run backwards (line %d)" % r["lineno"])
        prev = r

    # ultrasonic publication numbers
    pubs = [r for r in recs if r["ev"] in ("PRE", "US")]
    for a, b in zip(pubs, pubs[1:]):
        step = (b["SEQ"] - a["SEQ"]) % SEQ_MOD
        if step != 1:
            errs.append("ultrasonic publication gap: SEQ:%d -> SEQ:%d (line %d)"
                        % (a["SEQ"], b["SEQ"], b["lineno"]))
            break

    arm = run.first("ARM")
    if arm is not None:
        if arm["FM"] != 0:
            errs.append("ultrasonic fault mask FM:%X at ARM" % arm["FM"])
        if profile != "ANY" and arm["P"] != profile:
            errs.append("profile is %s, expected %s" % (arm["P"], profile))
        if arm["P"] not in PROFILES:
            errs.append("unknown profile %r" % arm["P"])
    req = run.first("REQ")
    if req is not None and req["R"] != "NEAR" and not allow_non_near:
        errs.append("stop cause is %s, not NEAR (use --allow-non-near)" % req["R"])
    zero = run.first("ZERO")
    if zero is not None and zero["TO"] != 0:
        errs.append("zero-speed timeout (ZERO TO:1)")
    done = run.first("DONE")
    if done is not None:
        if done["R"] != "OK":
            errs.append("DONE result is %s" % done["R"])
        if done["DR"] != 0:
            errs.append("%d record(s) dropped (DR)" % done["DR"])
        if done["RF"] != 0:
            errs.append("%d record(s) lost to a full ring (RF)" % done["RF"])
        if done["ZT"] != 0:
            errs.append("zero-speed timeout flag (ZT)")
        if done["SK"] != 0:
            run.warnings.append("%d earlier window(s) skipped (SK)" % done["SK"])
        if arm is not None and done["W"] != arm["W"]:
            errs.append("DONE window W:%d differs from ARM W:%d" % (
                done["W"], arm["W"]))


def metrics_for(run):
    """Metric dict for a run that passed validate()."""
    m = {}
    arm = run.first("ARM")
    req, acc, take = run.first("REQ"), run.first("ACC"), run.first("TAKE")
    brk, bend = run.first("BRK"), run.first("BEND")
    zl, zr, zero = run.first("ZL"), run.first("ZR"), run.first("ZERO")

    pub = None
    for r in run.records:
        if r["ev"] in ("PRE", "US") and r["SEQ"] == req["SEQ"]:
            pub = r                      # last one wins if SEQ repeats
    if pub is not None:
        m["sensor_to_request_ms"] = tick_delta(pub["T"], req["T"])
        m["us_observation_delay_ms"] = pub["OD"]
        m["us_filtered_mm_at_request"] = pub["FILT"]
        m["us_age_ms_at_observation"] = pub["AGE"]
    else:
        # The publication's own record was not stored (cap); the firmware still
        # recorded how old it was at the request (PD).
        m["sensor_to_request_ms"] = req["PD"]
    m["request_to_accept_ms"] = tick_delta(req["T"], acc["T"])
    m["accept_to_takeover_ms"] = tick_delta(acc["T"], take["T"])
    m["takeover_to_brake_ms"] = tick_delta(take["T"], brk["T"])
    m["brake_to_zero_left_ms"] = tick_delta(brk["T"], zl["T"])
    m["brake_to_zero_right_ms"] = tick_delta(brk["T"], zr["T"])
    m["brake_to_zero_ms"] = tick_delta(brk["T"], zero["T"])
    m["request_to_zero_ms"] = tick_delta(req["T"], zero["T"])
    m["brake_duration_ms"] = tick_delta(brk["T"], bend["T"])
    m["zero_minus_bend_ms"] = d32(bend["T"], zero["T"])
    m["zero_lr_diff_ms"] = d32(zr["T"], zl["T"])

    for name, ref in (("req", req), ("take", take), ("brk", brk)):
        left = d32(ref["EL"], zero["EL"])
        right = d16(ref["ER"], zero["ER"])
        m["counts_after_%s_left" % name] = left
        m["counts_after_%s_right" % name] = right
        m["counts_after_%s_mean" % name] = (left + right) / 2.0
    m["counts_brk_lr_diff"] = (m["counts_after_brk_left"]
                               - m["counts_after_brk_right"])

    for key, rec, field in (("speed_at_req_left_cps", req, "VL"),
                            ("speed_at_req_right_cps", req, "VR"),
                            ("speed_at_brk_left_cps", brk, "VL"),
                            ("speed_at_brk_right_cps", brk, "VR")):
        if rec[field] is not None:
            m[key] = rec[field]
    return m


def stats(values):
    n = len(values)
    s = {"n": n, "min": min(values), "max": max(values),
         "median": statistics.median(values),
         "mean": statistics.fmean(values),
         "stdev": statistics.stdev(values) if n > 1 else 0.0}
    s["spread"] = s["max"] - s["min"]
    s["worst"] = max(values, key=abs)
    return s


def mm_of(counts, counts_per_cm):
    return counts / counts_per_cm * 10.0


def fmt(v):
    if isinstance(v, float):
        return ("%.1f" % v).rstrip("0").rstrip(".") if v != int(v) else "%d" % v
    return str(v)


def build_report(accepted, rejected, global_errors, counts_per_cm):
    out = []
    out.append("USPARK trace analysis  (read-only; ticks = ms)")
    out.append("Notes: US/PRE encoder counts (OEL/OER) are observation-time and "
               "are not used for travel;")
    out.append("       ZL/ZR/ZERO are confirmation times (50 ms rolling "
               "window, <= 1 count).")
    out.append("")
    out.append("runs accepted: %d   rejected: %d" % (len(accepted), len(rejected)))
    for run in rejected:
        out.append("  REJECTED %s" % run.name())
        for e in run.errors:
            out.append("      - %s" % e)
    for e in global_errors:
        out.append("  ERROR %s" % e)
    for run in accepted:
        for w in run.warnings:
            out.append("  note %s: %s" % (run.name(), w))
    out.append("")
    if not accepted:
        return out

    groups = [("TIMES (ms)", TIME_METRICS, ""),
              ("ENCODER COUNTS (forward positive)", COUNT_METRICS, "counts"),
              ("INFORMATION", INFO_METRICS, "")]
    for title, metrics, kind in groups:
        out.append(title)
        out.append("  %-28s %3s %9s %9s %9s %9s %9s %9s" % (
            "metric", "n", "median", "worst", "min", "max", "spread", "stdev"))
        for name, desc in metrics:
            vals = [r.metrics[name] for r in accepted
                    if r.metrics.get(name) is not None]
            if not vals:
                continue
            s = stats(vals)
            out.append("  %-28s %3d %9s %9s %9s %9s %9s %9s   %s" % (
                name, s["n"], fmt(s["median"]), fmt(s["worst"]),
                fmt(s["min"]), fmt(s["max"]), fmt(s["spread"]),
                fmt(round(s["stdev"], 2)), desc))
            if counts_per_cm is not None and kind == "counts":
                mmv = [mm_of(v, counts_per_cm) for v in vals]
                t = stats(mmv)
                out.append("  %-28s %3d %9s %9s %9s %9s %9s %9s   mm @ %s counts/cm" % (
                    name.replace("counts", "mm"), t["n"],
                    fmt(round(t["median"], 1)), fmt(round(t["worst"], 1)),
                    fmt(round(t["min"], 1)), fmt(round(t["max"], 1)),
                    fmt(round(t["spread"], 1)), fmt(round(t["stdev"], 2)),
                    fmt(counts_per_cm)))
        out.append("")

    out.append("PER RUN")
    names = [n for n, _ in TIME_METRICS[:4]] + ["brake_to_zero_ms",
             "request_to_zero_ms", "counts_after_req_mean",
             "counts_after_brk_mean"]
    out.append("  %-34s " % "run" + " ".join("%12s" % n[:12] for n in names))
    for run in accepted:
        out.append("  %-34s " % run.name()[-34:] + " ".join(
            "%12s" % fmt(run.metrics.get(n, "")) for n in names))
    return out


def write_csv(path, accepted, counts_per_cm):
    cols = ([n for n, _ in TIME_METRICS] + [n for n, _ in COUNT_METRICS]
            + [n for n, _ in INFO_METRICS])
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        header = ["run", "label"] + cols
        if counts_per_cm is not None:
            header += [n.replace("counts", "mm") for n, _ in COUNT_METRICS]
        w.writerow(header)
        for run in accepted:
            row = [run.name(), run.label] + [run.metrics.get(c, "") for c in cols]
            if counts_per_cm is not None:
                row += [mm_of(run.metrics[n], counts_per_cm)
                        for n, _ in COUNT_METRICS]
            w.writerow(row)


def analyse(paths, profile="T2T", allow_non_near=False):
    runs, global_errors = split_runs(paths)
    accepted, rejected = [], []
    for run in runs:
        validate(run, profile, allow_non_near)
        if not run.errors:
            run.metrics = metrics_for(run)
            # A tick pair that is "backwards" (e.g. a publication stamped after
            # the request) has no meaningful latency: reject, never guess.
            for key in ("sensor_to_request_ms", "request_to_accept_ms",
                        "accept_to_takeover_ms", "takeover_to_brake_ms",
                        "brake_to_zero_left_ms", "brake_to_zero_right_ms",
                        "brake_to_zero_ms", "request_to_zero_ms",
                        "brake_duration_ms"):
                if run.metrics.get(key) is None:
                    run.errors.append("inconsistent ticks: %s is negative" % key)
        if run.errors:
            rejected.append(run)
        else:
            accepted.append(run)
    return accepted, rejected, global_errors


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Offline analysis of USPARK,TRACE lines in a saved UART log "
                    "(never opens a serial port).")
    ap.add_argument("logs", nargs="+", help="saved log file(s)")
    ap.add_argument("--counts-per-cm", type=float, default=None,
                    help="encoder counts per centimetre: adds millimetre "
                         "columns (no default is assumed)")
    ap.add_argument("--csv", default=None, help="write the per-run table here")
    ap.add_argument("--profile", default="T2T",
                    choices=sorted(PROFILES | {"ANY"}),
                    help="required motion profile at ARM (default T2T)")
    ap.add_argument("--allow-non-near", action="store_true",
                    help="accept stops whose cause is not NEAR")
    args = ap.parse_args(argv)

    if args.counts_per_cm is not None:
        if not math.isfinite(args.counts_per_cm) or args.counts_per_cm <= 0:
            print("error: --counts-per-cm must be a positive number",
                  file=sys.stderr)
            return 2

    try:
        accepted, rejected, global_errors = analyse(
            args.logs, args.profile, args.allow_non_near)
    except OSError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2

    for line in build_report(accepted, rejected, global_errors,
                             args.counts_per_cm):
        print(line)

    if args.csv and accepted:
        write_csv(args.csv, accepted, args.counts_per_cm)

    if not accepted:
        return 2
    return 1 if (rejected or global_errors) else 0


if __name__ == "__main__":
    sys.exit(main())
