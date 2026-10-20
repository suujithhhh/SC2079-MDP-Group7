"""Terminable benchmark workers (`algo/benchmarks/process_control.py`).

Uses only throw-away Python children that this test starts itself (a sleeper
and a printer). It never scans for, signals or kills any other process, and it
never asserts anything about other Python processes on the machine.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import os
import subprocess
import sys
import textwrap
import time
import unittest
from pathlib import Path
from unittest import mock

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import process_control as pc  # noqa: E402

SLEEPER = [sys.executable, "-c", "import time; time.sleep(60)"]


class Timeout(unittest.TestCase):
    def test_a_sleeper_is_killed_at_the_hard_timeout_and_death_is_verified(self):
        started = time.perf_counter()
        outcome = pc.run_worker(SLEEPER, 1.0)
        elapsed = time.perf_counter() - started
        self.assertTrue(outcome.timed_out)
        self.assertLess(elapsed, 15)
        self.assertGreaterEqual(elapsed, 1.0)
        cleanup = outcome.cleanup
        self.assertTrue(cleanup["killed"])
        self.assertTrue(cleanup["death_verified"])
        self.assertEqual(cleanup["timeout_kind"], pc.TIMEOUT_KIND)
        self.assertIsNotNone(cleanup["exit_code"])
        if sys.platform == "win32":
            self.assertIn(cleanup["containment"], ("job_object", "process_kill_fallback"))
        else:
            self.assertEqual(cleanup["containment"], "process_group")

    @unittest.skipUnless(sys.platform == "win32", "Job Objects are Windows-only")
    def test_windows_uses_a_job_object_and_closes_it(self):
        outcome = pc.run_worker(SLEEPER, 1.0)
        self.assertEqual(outcome.cleanup["containment"], "job_object")
        self.assertTrue(outcome.cleanup["job_closed"])

    def test_a_worker_that_finishes_is_not_marked_killed(self):
        outcome = pc.run_worker([sys.executable, "-c", "print('hello'); import sys; print('warn', file=sys.stderr)"], 30)
        self.assertFalse(outcome.timed_out)
        self.assertEqual(outcome.returncode, 0)
        self.assertEqual(outcome.stdout.strip(), "hello")
        self.assertIn("warn", outcome.stderr)
        self.assertFalse(outcome.cleanup["killed"])
        self.assertIsNone(outcome.cleanup["timeout_kind"])
        self.assertTrue(outcome.cleanup["death_verified"])

    @unittest.skipUnless(sys.platform == "win32", "Job Objects are Windows-only")
    def test_the_job_report_is_read_before_the_job_is_closed(self):
        clean = pc.run_worker([sys.executable, "-c", "print('ok')"], 30)
        self.assertEqual(clean.job_report["active_processes"], 0)
        self.assertIsInstance(clean.job_report["peak_job_commit_bytes"], int)
        self.assertGreater(clean.job_report["peak_process_commit_bytes"], 0)
        # A worker that leaves its own sleeper child behind: the job still counts it before
        # close (closing the kill-on-close job then ends that test-owned child).
        leaver = ("import subprocess, sys; "
                  "subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], "
                  "stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)")
        left = pc.run_worker([sys.executable, "-c", leaver], 30)
        self.assertFalse(left.timed_out)
        self.assertGreaterEqual(left.job_report["active_processes"], 1)
        self.assertTrue(left.cleanup["job_closed"])

    def test_a_failing_worker_reports_its_exit_code(self):
        outcome = pc.run_worker([sys.executable, "-c", "raise SystemExit(3)"], 30)
        self.assertEqual(outcome.returncode, 3)
        self.assertFalse(outcome.timed_out)


class Liveness(unittest.TestCase):
    def test_process_alive_tracks_a_child_this_test_owns(self):
        child = subprocess.Popen(SLEEPER)
        try:
            self.assertTrue(pc.process_alive(child.pid))
        finally:
            child.kill()
            child.wait(timeout=10)
        self.assertFalse(pc.process_alive(child.pid))


class UnverifiableDeath(unittest.TestCase):
    def test_unverified_death_raises_cleanup_error_with_details(self):
        with mock.patch.object(pc.ContainedProcess, "verify_dead", return_value=False):
            with self.assertRaises(pc.CleanupError) as raised:
                pc.run_worker(SLEEPER, 1.0)
        cleanup = raised.exception.cleanup
        self.assertFalse(cleanup["death_verified"])
        self.assertTrue(cleanup["killed"])
        # The worker was still killed by the containment cleanup.


@unittest.skipUnless(sys.platform == "win32", "kill-on-parent-exit is a Windows Job Object property")
class KillOnParentExit(unittest.TestCase):
    def test_the_worker_dies_when_the_harness_process_dies(self):
        script = textwrap.dedent(f"""
            import os, sys
            sys.path.insert(0, {str(ALGO)!r})
            from benchmarks import process_control as pc
            contained = pc.start_contained({SLEEPER!r})
            print(contained.pid, contained.containment, flush=True)
            os._exit(0)        # no cleanup: only the job handle closing can kill the worker
        """)
        parent = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
        self.assertEqual(parent.returncode, 0, parent.stderr)
        pid_text, containment = parent.stdout.split()
        pid = int(pid_text)
        try:
            self.assertEqual(containment, "job_object")
            deadline = time.perf_counter() + 15
            while pc.process_alive(pid) and time.perf_counter() < deadline:
                time.sleep(0.2)
            self.assertFalse(pc.process_alive(pid), "worker survived its harness")
        finally:
            if pc.process_alive(pid):       # test hygiene: only the exact child this test started
                os.system(f"taskkill /PID {pid} /F >NUL 2>&1")


if __name__ == "__main__":
    unittest.main()
