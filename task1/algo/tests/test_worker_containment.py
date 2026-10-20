"""Process-tree containment (`algo/worker_containment.py`).

Every process these tests kill was started by the test itself: a contained
sleeper, the grandchild that sleeper starts, or a sentinel the test owns. Nothing
enumerates, scans or signals any other process, and nothing asserts anything
about other Python processes on the machine.

Run from algo/:
    python -m unittest tests.test_worker_containment -v
"""

import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from pathlib import Path
from unittest import mock

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

import worker_containment as wc  # noqa: E402
from benchmarks.process_control import process_alive  # noqa: E402  (test-only liveness probe)

SLEEPER = [sys.executable, "-c", "import time; time.sleep(120)"]
# Starts a grandchild sleeper, prints its pid, then sleeps itself.
PARENT_OF_SLEEPER = [sys.executable, "-c", textwrap.dedent("""
    import subprocess, sys, time
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)"])
    print(child.pid, flush=True)
    time.sleep(120)
""")]


def _wait_until_dead(pid, seconds=10.0):
    deadline = time.monotonic() + seconds
    while process_alive(pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    return not process_alive(pid)


class _Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="wc_test_")
        self.addCleanup(self._tmp.cleanup)
        self._log = open(os.path.join(self._tmp.name, "stderr.log"), "wb")
        self.addCleanup(self._log.close)
        self._started = []

    def start(self, command):
        worker = wc.start_worker(command, cwd=self._tmp.name, stderr=self._log)
        self._started.append(worker)
        return worker

    def tearDown(self):
        # Hygiene for workers this test itself started, and only those.
        for worker in self._started:
            try:
                if worker.proc.poll() is None:
                    worker.terminate_tree()
                worker.wait_dead(10.0)
            except OSError:
                pass
            worker.close_pipes()
            worker.close()


class TerminateTree(_Base):
    def test_terminate_kills_the_worker_and_death_is_verified(self):
        worker = self.start(SLEEPER)
        self.assertTrue(process_alive(worker.pid))
        self.assertFalse(worker.wait_dead(0.05))
        self.assertFalse(worker.tree_dead(0.05))
        worker.terminate_tree()
        self.assertTrue(worker.wait_dead(10.0))
        self.assertTrue(worker.tree_dead(10.0))
        self.assertIsNotNone(worker.proc.returncode)
        self.assertFalse(process_alive(worker.pid))
        self.assertTrue(worker.close())
        self.assertEqual(worker.containment, "job_object" if sys.platform == "win32" else "process_group")

    def test_the_whole_tree_dies_including_a_grandchild(self):
        worker = self.start(PARENT_OF_SLEEPER)
        grandchild = int(worker.proc.stdout.readline().decode().strip())
        self.assertTrue(process_alive(grandchild))
        worker.terminate_tree()
        self.assertTrue(worker.wait_dead(10.0))
        self.assertTrue(worker.tree_dead(10.0))
        self.assertTrue(_wait_until_dead(grandchild), "the grandchild survived the tree kill")

    def test_a_tree_with_a_live_grandchild_is_not_reported_dead(self):
        worker = self.start(PARENT_OF_SLEEPER)
        grandchild = int(worker.proc.stdout.readline().decode().strip())
        worker.proc.kill()                       # test-owned: kill ONLY the direct worker
        self.assertTrue(worker.wait_dead(10.0))
        try:
            self.assertFalse(worker.tree_dead(0.2), "tree_dead ignored a live descendant")
        finally:
            worker.terminate_tree()
        self.assertTrue(worker.tree_dead(10.0))
        self.assertTrue(_wait_until_dead(grandchild))

    def test_an_unrelated_sentinel_process_survives(self):
        sentinel = subprocess.Popen(SLEEPER)     # owned by this test, never contained
        try:
            worker = self.start(PARENT_OF_SLEEPER)
            worker.proc.stdout.readline()
            worker.terminate_tree()
            self.assertTrue(worker.tree_dead(10.0))
            self.assertTrue(process_alive(sentinel.pid), "an unrelated process was killed")
            self.assertIsNone(sentinel.poll())
        finally:
            sentinel.kill()                      # through its own handle
            sentinel.wait(timeout=10)

    def test_terminating_an_already_finished_worker_is_harmless(self):
        worker = self.start([sys.executable, "-c", "pass"])
        self.assertTrue(worker.await_exit(10.0))
        worker.terminate_tree()                  # the executor always sweeps the tree, even after a clean exit
        self.assertTrue(worker.wait_dead(10.0))
        self.assertEqual(worker.proc.returncode, 0, "sweeping an exited worker must not change its exit code")
        self.assertTrue(worker.tree_dead(1.0))

    def test_await_exit_waits_for_a_self_exit_and_times_out_on_a_live_worker(self):
        live = self.start(SLEEPER)
        self.assertFalse(live.await_exit(0.2))
        live.terminate_tree()
        self.assertTrue(live.await_exit(10.0))
        self.assertTrue(live.wait_dead(10.0))
        if sys.platform != "win32":
            # POSIX: await_exit observed the exit without reaping, so the group id
            # stayed reserved until wait_dead reaped the worker.
            self.assertIsNotNone(live.proc.returncode)

    def test_close_pipes_closes_both_parent_ends(self):
        worker = self.start([sys.executable, "-c", "pass"])
        self.assertTrue(worker.wait_dead(10.0))
        worker.close_pipes()
        self.assertTrue(worker.proc.stdin.closed and worker.proc.stdout.closed)
        worker.close_pipes()                     # idempotent


class NoWeakerFallback(_Base):
    def test_a_refused_terminate_is_raised_not_papered_over(self):
        worker = self.start(SLEEPER)
        target = "_Job.terminate" if sys.platform == "win32" else "os.killpg"
        owner = wc._Job if sys.platform == "win32" else wc.os
        name = "terminate" if sys.platform == "win32" else "killpg"
        with mock.patch.object(owner, name, side_effect=PermissionError("refused")):
            with self.assertRaises(OSError):
                worker.terminate_tree()
        self.assertIsNone(worker.proc.poll(), "terminate_tree must not fall back to Popen.kill()")
        self.assertTrue(process_alive(worker.pid))


class ContainmentFailure(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="wc_test_")
        self.addCleanup(self._tmp.cleanup)
        self._log = open(os.path.join(self._tmp.name, "stderr.log"), "wb")
        self.addCleanup(self._log.close)

    def test_a_spawn_failure_raises_containment_error(self):
        with mock.patch.object(wc.subprocess, "Popen", side_effect=FileNotFoundError("nope")):
            with self.assertRaises(wc.ContainmentError):
                wc.start_worker(["missing"], cwd=self._tmp.name, stderr=self._log)

    @unittest.skipUnless(sys.platform == "win32", "Job Objects are Windows-only")
    def test_a_failed_job_assignment_kills_and_reaps_the_gated_worker(self):
        created = []
        real_popen = subprocess.Popen

        def recording_popen(*args, **kwargs):
            proc = real_popen(*args, **kwargs)
            created.append(proc)
            return proc

        with mock.patch.object(wc.subprocess, "Popen", side_effect=recording_popen), \
                mock.patch.object(wc._Job, "assign", side_effect=OSError(5, "denied")):
            with self.assertRaises(wc.ContainmentError):
                wc.start_worker(SLEEPER, cwd=self._tmp.name, stderr=self._log)
        self.assertEqual(len(created), 1)
        self.assertIsNotNone(created[0].poll(), "the gated worker was not reaped")
        self.assertFalse(process_alive(created[0].pid))
        self.assertTrue(created[0].stdin.closed and created[0].stdout.closed,
                        "the failed setup left the gated worker's pipes open")

    @unittest.skipUnless(sys.platform == "win32", "Job Objects are Windows-only")
    def test_a_failed_job_creation_starts_no_worker(self):
        with mock.patch.object(wc.subprocess, "Popen") as popen, \
                mock.patch.object(wc._Job, "__init__", side_effect=OSError(5, "denied")):
            with self.assertRaises(wc.ContainmentError):
                wc.start_worker(SLEEPER, cwd=self._tmp.name, stderr=self._log)
        popen.assert_not_called()


@unittest.skipUnless(sys.platform == "win32", "kill-on-parent-exit is a Windows Job Object property")
class KillOnParentExit(unittest.TestCase):
    def test_the_worker_dies_when_the_parent_dies(self):
        script = textwrap.dedent(f"""
            import os, sys, tempfile
            sys.path.insert(0, {str(ALGO)!r})
            import worker_containment as wc
            tmp = tempfile.mkdtemp(prefix="wc_parent_")
            log = open(os.path.join(tmp, "stderr.log"), "wb")
            worker = wc.start_worker({SLEEPER!r}, cwd=tmp, stderr=log)
            print(worker.pid, tmp, flush=True)
            os._exit(0)        # no cleanup: only the job handle closing can kill the worker
        """)
        parent = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
        self.assertEqual(parent.returncode, 0, parent.stderr)
        pid_text, scratch = parent.stdout.split(maxsplit=1)
        pid = int(pid_text)
        self.addCleanup(shutil.rmtree, scratch.strip(), ignore_errors=True)
        try:
            self.assertTrue(_wait_until_dead(pid, 15.0), "the worker survived its parent")
        finally:
            if process_alive(pid):    # hygiene: only the exact child this test started
                os.system(f"taskkill /PID {pid} /F >NUL 2>&1")


if __name__ == "__main__":
    unittest.main()
