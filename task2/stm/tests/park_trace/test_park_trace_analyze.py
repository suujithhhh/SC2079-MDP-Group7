"""Tests for park_trace_analyze.py (the offline USPARK trace analyser).

Run from the repository root:
    python -I -m unittest discover -s tests/park_trace -p "test_*.py"

The synthetic runs use exactly the line grammar that app_park_trace_fmt.c
produces. Set PARK_TRACE_FIRMWARE_LOG to a log written by the firmware-side host
test (tests/comm/app_comm_park_trace_test.c) to also analyse REAL formatter
output (the runner script does this).
"""

import contextlib
import io
import os
import re
import statistics
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import park_trace_analyze as pta  # noqa: E402


def fmt_fields(fields):
    return ",".join("%s:%s" % (k, v) for k, v in fields)


def render(records, start_n=1):
    """records: list of (event, [(key, value), ...]) without N; adds N."""
    lines = []
    for i, (ev, fields) in enumerate(records):
        kv = [("N", i + start_n)] + list(fields)
        lines.append("USPARK,TRACE,EV:%s,%s" % (ev, fmt_fields(kv)))
    return lines


def good_records(base=1000, w=7, cause="NEAR", profile="T2T", bz_left=80,
                 bz_right=60, el0=2000, er0=2000, left_travel=200,
                 right_travel=160, take_travel=80, speed=8000, fm=0, seq0=1,
                 to=0, dr=0, rf=0, res="OK", pd=10, zero_to=None, done_zt=None):
    t = base
    recs = []

    def pub(ev, dt, seq, filt, age=10):
        recs.append((ev, [("T", t + dt), ("OD", 10), ("SEQ", seq),
                          ("RAW", filt + 1), ("FILT", filt), ("AGE", age),
                          ("PU", filt * 6), ("OEL", el0 - 500), ("OER", er0 - 500)]))

    pub("PRE", 0, seq0, 900)
    pub("PRE", 70, seq0 + 1, 800)
    pub("PRE", 140, seq0 + 2, 400)
    recs.append(("ARM", [("T", t + 150), ("W", w), ("SEQ", seq0 + 2), ("FILT", 400),
                         ("CN", 0), ("P", profile), ("FM", "%X" % fm),
                         ("EL", el0 - 400), ("ER", er0 - 400)]))
    pub("US", 210, seq0 + 3, 400)
    pub("US", 280, seq0 + 4, 290)
    recs.append(("NEAR", [("T", t + 290), ("K", 1), ("SEQ", seq0 + 4), ("FILT", 290),
                          ("EL", el0 - 200), ("ER", er0 - 200)]))
    pub("US", 350, seq0 + 5, 285)
    recs.append(("NEAR", [("T", t + 360), ("K", 2), ("SEQ", seq0 + 5), ("FILT", 285),
                          ("EL", el0 - 100), ("ER", er0 - 100)]))
    pub("US", 420, seq0 + 6, 280)
    recs.append(("NEAR", [("T", t + 430), ("K", 3), ("SEQ", seq0 + 6), ("FILT", 280),
                          ("EL", el0), ("ER", er0)]))
    recs.append(("REQ", [("T", t + 430), ("R", cause), ("SEQ", seq0 + 6), ("PD", pd),
                         ("EL", el0), ("ER", er0), ("VL", speed), ("VR", speed)]))
    recs.append(("ACC", [("T", t + 430), ("EL", el0), ("ER", er0)]))
    recs.append(("TAKE", [("T", t + 440), ("EL", el0 + take_travel),
                          ("ER", er0 + take_travel)]))
    brk = t + 440
    recs.append(("BRK", [("T", brk), ("EL", el0 + take_travel),
                         ("ER", er0 + take_travel), ("VL", speed), ("VR", speed)]))
    zero_t = brk + max(bz_left, bz_right)
    bend_t = brk + 120
    tail = [
        # (time, tie-break, event, extra fields) -- the order the firmware
        # records them: a latched BEND is submitted before that cycle's sample,
        # ZL before ZR, ZERO after both wheels.
        (bend_t, -1, "BEND", []),
        (brk + bz_left, 0, "ZL", []),
        (brk + bz_right, 1, "ZR", []),
        (zero_t, 2, "ZERO", [("TO", to if zero_to is None else zero_to)]),
    ]
    tail.sort(key=lambda x: (x[0], x[1]))
    for tick, _, ev, extra in tail:
        recs.append((ev, [("T", tick), ("EL", el0 + left_travel),
                          ("ER", er0 + right_travel)] + extra))
    recs.append(("DONE", [("T", max(zero_t, bend_t)), ("W", w), ("R", res),
                          ("DR", dr), ("RF", rf),
                          ("ZT", to if done_zt is None else done_zt), ("SK", 0)]))
    return wrap_values(recs)


def wrap_values(recs):
    """Ticks are 32-bit, the left encoder 32-bit, the right encoder 16-bit."""
    out = []
    for ev, fields in recs:
        f2 = []
        for k, v in fields:
            if k in ("T",):
                v = v % (1 << 32)
            elif k in ("EL", "OEL"):
                v = ((v + (1 << 31)) % (1 << 32)) - (1 << 31)
            elif k in ("ER", "OER"):
                v = ((v + (1 << 15)) % (1 << 16)) - (1 << 15)
            f2.append((k, v))
        out.append((ev, f2))
    return out


def write_log(lines, style="tab", mark=None):
    f = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False,
                                    newline="", encoding="utf-8")
    ts = "2026-01-01T00:00:00.000+00:00"
    if mark:
        f.write("%s\tMARK\t%s\n" % (ts, mark))
    for ln in lines:
        if style == "tab":
            f.write("%s\tRX\t%s\n" % (ts, ln))
        elif style == "console":
            f.write("%s RX %s\n" % (ts, ln))
        else:
            f.write("%s\r\n" % ln)
    f.close()
    return f.name


class Base(unittest.TestCase):
    def setUp(self):
        self.files = []

    def tearDown(self):
        for p in self.files:
            try:
                os.unlink(p)
            except OSError:
                pass

    def log(self, lines, **kw):
        p = write_log(lines, **kw)
        self.files.append(p)
        return p

    def analyse(self, lines, profile="T2T", allow_non_near=False, **kw):
        return pta.analyse([self.log(lines, **kw)], profile, allow_non_near)

    def run_one(self, **kw):
        acc, rej, glob = self.analyse(render(good_records(**kw)))
        self.assertEqual((len(acc), len(rej), glob), (1, 0, []),
                         [r.errors for r in rej])
        return acc[0]

    def rejected_reasons(self, lines, **kw):
        acc, rej, glob = self.analyse(lines, **kw)
        self.assertEqual(len(acc), 0, "expected rejection")
        reasons = []
        for r in rej:
            reasons += r.errors
        return reasons + glob


class GoodRun(Base):
    def test_metrics_exact(self):
        m = self.run_one().metrics
        self.assertEqual(m["sensor_to_request_ms"], 10)        # 430 - 420
        self.assertEqual(m["request_to_accept_ms"], 0)
        self.assertEqual(m["accept_to_takeover_ms"], 10)
        self.assertEqual(m["takeover_to_brake_ms"], 0)
        self.assertEqual(m["brake_to_zero_left_ms"], 80)
        self.assertEqual(m["brake_to_zero_right_ms"], 60)
        self.assertEqual(m["brake_to_zero_ms"], 80)
        self.assertEqual(m["request_to_zero_ms"], 90)
        self.assertEqual(m["brake_duration_ms"], 120)
        self.assertEqual(m["zero_minus_bend_ms"], -40)         # zero came first
        self.assertEqual(m["zero_lr_diff_ms"], 20)
        self.assertEqual(m["counts_after_req_left"], 200)
        self.assertEqual(m["counts_after_req_right"], 160)
        self.assertEqual(m["counts_after_req_mean"], 180.0)
        self.assertEqual(m["counts_after_take_left"], 120)
        self.assertEqual(m["counts_after_take_right"], 80)
        self.assertEqual(m["counts_after_take_mean"], 100.0)
        self.assertEqual(m["counts_after_brk_left"], 120)
        self.assertEqual(m["counts_after_brk_right"], 80)
        self.assertEqual(m["counts_after_brk_mean"], 100.0)
        self.assertEqual(m["counts_brk_lr_diff"], 40)
        self.assertEqual(m["speed_at_req_left_cps"], 8000)
        self.assertEqual(m["us_filtered_mm_at_request"], 280)
        self.assertEqual(m["us_observation_delay_ms"], 10)

    def test_zero_after_bend(self):
        m = self.run_one(bz_left=200, bz_right=150).metrics
        self.assertEqual(m["zero_minus_bend_ms"], 80)           # 200 - 120
        self.assertEqual(m["brake_to_zero_ms"], 200)

    def test_log_styles_and_noise(self):
        lines = render(good_records())
        noisy = ["MM,EV:START,D:F,T:0,V:50", "USPARK,STOP,R:NEAR,D:280,V:7999,FM:00",
                 "ACK,STOPPED", ""] + lines + ["ERR,BUSY"]
        for style in ("tab", "console", "bare"):
            acc, rej, glob = self.analyse(noisy, style=style)
            self.assertEqual((len(acc), len(rej), glob), (1, 0, []), style)

    def test_mark_label_and_tx_ignored(self):
        lines = render(good_records())
        p = self.log(lines, mark="run 3 battery 7.9V")
        with open(p, "a", encoding="utf-8") as fh:
            fh.write("2026\tTX\tUSPARK,TRACE,EV:DONE,N:1\n")  # a TX echo: ignored
        acc, rej, glob = pta.analyse([p])
        self.assertEqual((len(acc), len(rej), glob), (1, 0, []))
        self.assertEqual(acc[0].label, "run 3 battery 7.9V")

    def test_tick_wraparound(self):
        acc = self.run_one(base=0xFFFFFFFF - 200)       # wraps inside the run
        m = acc.metrics
        self.assertEqual(m["sensor_to_request_ms"], 10)
        self.assertEqual(m["request_to_accept_ms"], 0)
        self.assertEqual(m["accept_to_takeover_ms"], 10)
        self.assertEqual(m["brake_to_zero_ms"], 80)
        self.assertEqual(m["request_to_zero_ms"], 90)
        self.assertEqual(m["brake_duration_ms"], 120)
        self.assertEqual(m["zero_minus_bend_ms"], -40)
        ticks = [r["T"] for r in acc.records]
        self.assertTrue(max(ticks) > 0xFFFFFF00 and min(ticks) < 0x1000)

    def test_encoder_wraparound(self):
        # left (32-bit) crosses INT32_MAX during the stop; right (16-bit) crosses
        # 32767. The printed values are the wrapped ones, as the firmware sends.
        recs = good_records(el0=(1 << 31) - 90, er0=32700)
        values = [(ev, dict(f)) for ev, f in recs]
        els = [f["EL"] for ev, f in values if "EL" in f]
        ers = [f["ER"] for ev, f in values if "ER" in f]
        self.assertTrue(min(els) < 0 < max(els) and max(els) > (1 << 30))
        self.assertTrue(min(ers) < -30000 and max(ers) > 30000)
        acc, rej, glob = self.analyse(render(recs))
        self.assertEqual((len(acc), len(rej), glob), (1, 0, []),
                         [r.errors for r in rej])
        m = acc[0].metrics
        self.assertEqual(m["counts_after_req_left"], 200)
        self.assertEqual(m["counts_after_req_right"], 160)
        self.assertEqual(m["counts_after_take_left"], 120)
        self.assertEqual(m["counts_after_take_right"], 80)
        self.assertEqual(m["counts_after_brk_left"], 120)
        self.assertEqual(m["counts_after_brk_right"], 80)
        # and backwards through the other edge
        recs = good_records(el0=-(1 << 31) + 90, er0=-32700, left_travel=-200,
                            right_travel=-160, take_travel=-80)
        acc, rej, glob = self.analyse(render(recs))
        self.assertEqual(len(acc), 1, [r.errors for r in rej])
        self.assertEqual(acc[0].metrics["counts_after_req_left"], -200)
        self.assertEqual(acc[0].metrics["counts_after_req_right"], -160)

    def test_sequence_numbers_wrap(self):
        acc = self.run_one(seq0=65533)                  # 65533 .. 65539 -> 3
        self.assertEqual(acc.metrics["sensor_to_request_ms"], 10)

    def test_negative_direction_counts(self):
        run = self.run_one(left_travel=-200, right_travel=-160, take_travel=-80)
        self.assertEqual(run.metrics["counts_after_req_left"], -200)
        self.assertEqual(run.metrics["counts_after_req_mean"], -180.0)


class Statistics(Base):
    def test_median_worst_spread_across_runs(self):
        lines = []
        for bz in (60, 70, 110):                  # mean 80 differs from median 70
            lines += render(good_records(bz_left=bz, bz_right=bz - 20,
                                         left_travel=100 + bz, right_travel=90 + bz))
        acc, rej, glob = self.analyse(lines)
        self.assertEqual((len(acc), len(rej), glob), (3, 0, []))
        vals = [r.metrics["brake_to_zero_ms"] for r in acc]
        self.assertEqual(vals, [60, 70, 110])
        s = pta.stats(vals)
        self.assertEqual(s["median"], 70)
        self.assertEqual(s["worst"], 110)
        self.assertEqual((s["min"], s["max"], s["spread"]), (60, 110, 50))
        self.assertAlmostEqual(s["mean"], 80.0)
        self.assertAlmostEqual(s["stdev"], statistics.stdev([60, 70, 110]))
        counts = [r.metrics["counts_after_req_left"] for r in acc]
        self.assertEqual(counts, [160, 170, 210])
        cs = pta.stats(counts)
        self.assertEqual((cs["median"], cs["worst"], cs["spread"]), (170, 210, 50))
        self.assertEqual(pta.stats([5])["stdev"], 0.0)
        self.assertEqual(pta.stats([4, 8])["median"], 6.0)
        # the report prints the same median and worst case
        text = "\n".join(pta.build_report(acc, rej, glob, None))
        row = [l for l in text.splitlines() if l.strip().startswith("brake_to_zero_ms")][0]
        self.assertEqual(row.split()[2:4], ["70", "110"])

    def test_worst_is_largest_magnitude(self):
        self.assertEqual(pta.stats([-9, 4, 7])["worst"], -9)

    def test_report_contains_every_requested_quantity(self):
        lines = render(good_records()) + render(good_records(base=9000, w=8))
        acc, rej, glob = self.analyse(lines)
        text = "\n".join(pta.build_report(acc, rej, glob, None))
        for name in ("sensor_to_request_ms", "request_to_accept_ms",
                     "accept_to_takeover_ms", "takeover_to_brake_ms",
                     "brake_to_zero_left_ms", "brake_to_zero_right_ms",
                     "brake_to_zero_ms", "request_to_zero_ms",
                     "counts_after_req_mean", "counts_after_take_mean",
                     "counts_after_brk_mean", "median", "worst", "spread"):
            self.assertIn(name, text)
        self.assertNotIn("mm @", text)                 # no mm without --counts-per-cm
        self.assertIn("observation-time", text)       # never mislabelled

    def test_millimetres_only_with_counts_per_cm(self):
        acc, rej, glob = self.analyse(render(good_records()))
        no_mm = "\n".join(pta.build_report(acc, rej, glob, None))
        with_mm = "\n".join(pta.build_report(acc, rej, glob, 75.0))
        self.assertNotIn("mm_after", no_mm)
        self.assertIn("mm_after_req_left", with_mm)
        self.assertIn("mm @ 75 counts/cm", with_mm)
        self.assertAlmostEqual(pta.mm_of(200, 75.0), 200 / 75.0 * 10.0)
        self.assertAlmostEqual(pta.mm_of(72, 72.0), 10.0)

    def test_csv_has_mm_columns_only_when_requested(self):
        acc, _, _ = self.analyse(render(good_records()))
        for cpc, expect in ((None, False), (72.0, True)):
            fd, path = tempfile.mkstemp(suffix=".csv")
            os.close(fd)
            self.files.append(path)
            pta.write_csv(path, acc, cpc)
            with open(path, encoding="utf-8") as fh:
                header = fh.readline()
            self.assertEqual("mm_after_req_left" in header, expect)
            self.assertIn("brake_to_zero_ms", header)

    def test_pd_fallback_when_publication_not_stored(self):
        # A non-NEAR stop (NOUPDATE) refers to an OLD publication that has no
        # US/PRE record in the window: the firmware's own PD is used.
        recs = good_records(cause="NOUPDATE", pd=265)
        recs = [(ev, [(k, (60000 if k == "SEQ" else v)) if ev == "REQ" else (k, v)
                      for k, v in f]) for ev, f in recs]
        acc, rej, glob = self.analyse(render(recs), allow_non_near=True)
        self.assertEqual(len(acc), 1, [r.errors for r in rej])
        self.assertEqual(acc[0].metrics["sensor_to_request_ms"], 265)


class Rejections(Base):
    def edit(self, recs, fn):
        out = fn(list(recs))
        return render(out)

    def test_missing_events(self):
        for ev in ("ARM", "REQ", "ACC", "TAKE", "BRK", "BEND", "ZL", "ZR",
                   "ZERO", "DONE"):
            recs = [r for r in good_records() if r[0] != ev]
            lines = render(recs)
            acc, rej, glob = self.analyse(lines)
            self.assertEqual(len(acc), 0, ev)

    def test_out_of_order(self):
        def swap(recs):
            i = [r[0] for r in recs].index("ACC")
            recs[i], recs[i + 1] = recs[i + 1], recs[i]       # TAKE before ACC
            return recs
        reasons = self.rejected_reasons(self.edit(good_records(), swap))
        self.assertTrue(any("ACC" in r or "TAKE" in r for r in reasons), reasons)

        def zero_early(recs):
            names = [r[0] for r in recs]
            zi = names.index("ZERO")
            z = recs.pop(zi)
            recs.insert(names.index("ZL"), z)
            return recs
        reasons = self.rejected_reasons(self.edit(good_records(), zero_early))
        self.assertTrue(any("ZERO" in r for r in reasons), reasons)

    def test_record_index_gap_from_lost_line(self):
        lines = render(good_records())
        del lines[6]                                  # one line lost on the wire
        reasons = self.rejected_reasons(lines)
        self.assertTrue(any("record index gap" in r for r in reasons), reasons)

    def test_publication_sequence_gap(self):
        recs = good_records()
        names = [r[0] for r in recs]
        first_us = names.index("US")
        recs.pop(first_us)                            # one publication missing
        reasons = self.rejected_reasons(render(recs))   # N is re-numbered
        self.assertTrue(any("publication gap" in r for r in reasons), reasons)

    def test_dropped_and_ring_full_records(self):
        reasons = self.rejected_reasons(render(good_records(dr=3)))
        self.assertTrue(any("dropped" in r for r in reasons), reasons)
        reasons = self.rejected_reasons(render(good_records(dr=1, rf=1)))
        self.assertTrue(any("full ring" in r for r in reasons), reasons)

    def test_timeouts(self):
        reasons = self.rejected_reasons(render(good_records(to=1, res="TO")))
        text = " ".join(reasons)
        self.assertIn("timeout", text)
        self.assertIn("DONE result is TO", text)
        # each timeout indicator is enough on its own
        reasons = self.rejected_reasons(render(good_records(zero_to=1)))
        self.assertTrue(any("ZERO TO:1" in r for r in reasons), reasons)
        reasons = self.rejected_reasons(render(good_records(done_zt=1)))
        self.assertTrue(any("(ZT)" in r for r in reasons), reasons)
        reasons = self.rejected_reasons(render(good_records(res="TO")))
        self.assertTrue(any("DONE result is TO" in r for r in reasons), reasons)

    def test_non_near_cause(self):
        reasons = self.rejected_reasons(render(good_records(cause="INVALID")))
        self.assertTrue(any("not NEAR" in r for r in reasons), reasons)
        acc, rej, glob = self.analyse(render(good_records(cause="INVALID")),
                                      allow_non_near=True)
        self.assertEqual(len(acc), 1)

    def test_profile_and_fault_mask(self):
        reasons = self.rejected_reasons(render(good_records(profile="PREC")))
        self.assertTrue(any("profile is PREC" in r for r in reasons), reasons)
        acc, _, _ = self.analyse(render(good_records(profile="PREC")),
                                 profile="ANY")
        self.assertEqual(len(acc), 1)
        reasons = self.rejected_reasons(render(good_records(fm=0x20)))
        self.assertTrue(any("fault mask" in r for r in reasons), reasons)

    def test_time_runs_backwards(self):
        def back(recs):
            i = [r[0] for r in recs].index("BRK")
            recs[i] = ("BRK", [("T", 5) if k == "T" else (k, v) for k, v in recs[i][1]])
            return recs
        reasons = self.rejected_reasons(self.edit(good_records(), back))
        self.assertTrue(any("backwards" in r for r in reasons), reasons)

    def test_wrong_window_in_done(self):
        def w(recs):
            i = [r[0] for r in recs].index("DONE")
            recs[i] = ("DONE", [("W", 99) if k == "W" else (k, v) for k, v in recs[i][1]])
            return recs
        reasons = self.rejected_reasons(self.edit(good_records(), w))
        self.assertTrue(any("window" in r for r in reasons), reasons)

    def test_malformed_lines(self):
        good = render(good_records())
        i = next(i for i, l in enumerate(good) if "EV:ACC" in l)
        bad_variants = {
            "missing field": good[i].replace(",EL:2000", ""),
            "non-integer": good[i].replace("EL:2000", "EL:20x0"),
            "unknown event": good[i].replace("EV:ACC", "EV:XYZ"),
            "NA where forbidden": good[i].replace("EL:2000", "EL:NA"),
            "duplicate field": good[i] + ",EL:2000",
            "extra field": good[i] + ",QQ:1",
            "truncated": good[i][:25],
            "no colon": good[i] + ",XYZ",
        }
        for name, bad in bad_variants.items():
            lines = list(good)
            lines[i] = bad
            acc, rej, glob = self.analyse(lines)
            self.assertEqual(len(acc), 0, name)
            blob = " ".join(e for r in rej for e in r.errors) + " ".join(glob)
            self.assertIn("malformed", blob, name)

    def test_incomplete_log(self):
        lines = render(good_records())[:-3]            # log ends before DONE
        reasons = self.rejected_reasons(lines)
        self.assertTrue(any("incomplete" in r or "log ends" in r for r in reasons),
                        reasons)

    def test_record_outside_a_window(self):
        lines = render(good_records())
        stray = ["USPARK,TRACE,EV:US,N:7,T:5,OD:1,SEQ:1,RAW:1,FILT:1,AGE:1,PU:1,"
                 "OEL:1,OER:1"]
        acc, rej, glob = self.analyse(stray + lines)
        self.assertEqual(len(acc), 1)                   # the real run survives
        self.assertTrue(any("outside" in g for g in glob), glob)

    def test_one_bad_run_does_not_hide_a_good_one(self):
        lines = render(good_records(dr=2)) + render(good_records(base=9000, w=8))
        acc, rej, glob = self.analyse(lines)
        self.assertEqual((len(acc), len(rej)), (1, 1))

    def test_window_not_closed_before_the_next(self):
        a = render(good_records())[:-1]                 # no DONE
        b = render(good_records(base=9000, w=8))
        acc, rej, glob = self.analyse(a + b)
        self.assertEqual((len(acc), len(rej)), (1, 1))
        self.assertTrue(any("no DONE" in e for e in rej[0].errors), rej[0].errors)


class CommandLine(Base):
    def call(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = pta.main(argv)
            except SystemExit as exc:
                code = exc.code
        return code, out.getvalue(), err.getvalue()

    def test_exit_codes(self):
        ok = self.log(render(good_records()))
        bad = self.log(render(good_records(dr=1)))
        mixed = self.log(render(good_records()) + render(good_records(dr=1, base=9000)))
        self.assertEqual(self.call([ok])[0], 0)
        self.assertEqual(self.call([bad])[0], 2)
        self.assertEqual(self.call([mixed])[0], 1)
        self.assertEqual(self.call([os.path.join(tempfile.gettempdir(),
                                                 "no_such_trace_log.txt")])[0], 2)
        self.assertEqual(self.call([ok, "--counts-per-cm", "0"])[0], 2)
        self.assertEqual(self.call([ok, "--counts-per-cm", "-3"])[0], 2)
        self.assertEqual(self.call([ok, "--counts-per-cm", "nan"])[0], 2)
        self.assertEqual(self.call([ok, "--counts-per-cm", "72"])[0], 0)
        self.assertEqual(self.call([ok, "--profile", "PREC"])[0], 2)

    def test_output_mentions_rejections(self):
        bad = self.log(render(good_records(dr=1)))
        code, out, _ = self.call([bad])
        self.assertIn("REJECTED", out)
        self.assertIn("dropped", out)

    def test_never_touches_a_serial_port(self):
        src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "park_trace_analyze.py")
        with open(src_path, encoding="utf-8") as fh:
            src = fh.read()
        code_lines = [l for l in src.splitlines()
                      if not l.lstrip().startswith(("#", '"""'))]
        code = "\n".join(code_lines)
        self.assertIsNone(re.search(r"^\s*(import|from)\s+serial\b", code, re.M))
        self.assertNotIn("serial.Serial", code)
        self.assertNotIn("pyserial", code)
        self.assertNotIn("socket", code)
        # the only thing it opens is a log file for reading
        self.assertEqual(len(re.findall(r"\bopen\(", code)), 2)  # read log, write csv


@unittest.skipUnless(os.environ.get("PARK_TRACE_FIRMWARE_LOG"),
                     "PARK_TRACE_FIRMWARE_LOG not set")
class FirmwareOutput(Base):
    def test_real_formatter_output_is_accepted(self):
        path = os.environ["PARK_TRACE_FIRMWARE_LOG"]
        # The host test runs the default PRECISION profile.
        acc, rej, glob = pta.analyse([path], profile="PREC")
        self.assertEqual((len(acc), len(rej), glob), (1, 0, []),
                         [r.errors for r in rej])
        m = acc[0].metrics
        self.assertEqual(m["brake_duration_ms"], 120)
        self.assertEqual(m["request_to_accept_ms"], 0)
        self.assertEqual(m["accept_to_takeover_ms"], 10)
        self.assertEqual(m["speed_at_req_left_cps"], 8000)
        # and the same file is rejected under the production profile gate
        acc, rej, glob = pta.analyse([path], profile="T2T")
        self.assertEqual(len(acc), 0)


if __name__ == "__main__":
    unittest.main()
