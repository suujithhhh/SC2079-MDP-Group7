"""Process-tree containment for the bounded prefix-search worker.

`start_worker()` starts ONE worker process together with the OS unit that lets
the parent kill everything the worker started, and later prove it is gone:

* Windows: the worker is assigned to a Job Object created with
  `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE`. `terminate_tree()` calls
  `TerminateJobObject`; `tree_dead()` reads the job's active-process count, so
  a grandchild is verified dead too. Because the parent holds the only job
  handle, the tree also dies if the parent dies. No memory limit is set.
* POSIX (straightforward, not exercised on POSIX hardware in this repository's
  Windows-targeted work): the worker gets its own session and process group and
  is killed with `killpg` BEFORE it is reaped, so the group id cannot have been
  reused. `tree_dead()` asks the group whether any member is left.

Nothing here enumerates, scans or signals any process other than the single
worker this module started and its own descendants (through the job or the
process group it created). There is no name- or pid-based lookup.

`Popen.kill()` is used in exactly one place: `start_worker()` cleaning up the
still-gated direct worker when containment setup itself fails (the worker has
not been sent its SPEC, so it has started nothing). It is never the hard-stop
path; `terminate_tree()` has no such fallback.
"""

import os
import select
import signal
import subprocess
import sys
import time

DEATH_POLL_S = 0.02

_WIN = sys.platform == "win32"


class ContainmentError(RuntimeError):
    """Containment could not be established; no SPEC was sent to any worker.
    `worker_reaped` is False only if a started direct worker could not be
    verified dead (the caller must then keep its working directory)."""

    def __init__(self, message: str, worker_reaped: bool = True):
        super().__init__(message)
        self.worker_reaped = worker_reaped


if _WIN:
    import ctypes
    from ctypes import wintypes

    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x00002000
    _JOB_OBJECT_BASIC_ACCOUNTING_INFORMATION = 1
    _JOB_EXTENDED_LIMIT_INFORMATION = 9
    _PROCESS_SET_QUOTA = 0x0100
    _PROCESS_TERMINATE = 0x0001
    _CREATE_NEW_PROCESS_GROUP = 0x00000200

    class _IoCounters(ctypes.Structure):
        _fields_ = [(name, ctypes.c_ulonglong) for name in (
            "ReadOperationCount", "WriteOperationCount", "OtherOperationCount",
            "ReadTransferCount", "WriteTransferCount", "OtherTransferCount")]

    class _BasicLimits(ctypes.Structure):
        _fields_ = [
            ("PerProcessUserTimeLimit", ctypes.c_int64), ("PerJobUserTimeLimit", ctypes.c_int64),
            ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
            ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
            ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD),
            ("SchedulingClass", wintypes.DWORD),
        ]

    class _ExtendedLimits(ctypes.Structure):
        _fields_ = [
            ("BasicLimitInformation", _BasicLimits), ("IoInfo", _IoCounters),
            ("ProcessMemoryLimit", ctypes.c_size_t), ("JobMemoryLimit", ctypes.c_size_t),
            ("PeakProcessMemoryUsed", ctypes.c_size_t), ("PeakJobMemoryUsed", ctypes.c_size_t),
        ]

    class _BasicAccounting(ctypes.Structure):
        _fields_ = [
            ("TotalUserTime", ctypes.c_int64), ("TotalKernelTime", ctypes.c_int64),
            ("ThisPeriodTotalUserTime", ctypes.c_int64), ("ThisPeriodTotalKernelTime", ctypes.c_int64),
            ("TotalPageFaultCount", wintypes.DWORD), ("TotalProcesses", wintypes.DWORD),
            ("ActiveProcesses", wintypes.DWORD), ("TotalTerminatedProcesses", wintypes.DWORD),
        ]

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _k32.CreateJobObjectW.argtypes = [wintypes.LPVOID, wintypes.LPCWSTR]
    _k32.CreateJobObjectW.restype = wintypes.HANDLE
    _k32.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD]
    _k32.SetInformationJobObject.restype = wintypes.BOOL
    _k32.QueryInformationJobObject.argtypes = [
        wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    _k32.QueryInformationJobObject.restype = wintypes.BOOL
    _k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _k32.OpenProcess.restype = wintypes.HANDLE
    _k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _k32.AssignProcessToJobObject.restype = wintypes.BOOL
    _k32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    _k32.TerminateJobObject.restype = wintypes.BOOL
    _k32.CloseHandle.argtypes = [wintypes.HANDLE]
    _k32.CloseHandle.restype = wintypes.BOOL

    class _Job:
        """A kill-on-close Job Object that holds one worker and its descendants."""

        def __init__(self):
            handle = _k32.CreateJobObjectW(None, None)
            if not handle:
                raise OSError(ctypes.get_last_error(), "CreateJobObjectW failed")
            self.handle = handle
            info = _ExtendedLimits()
            info.BasicLimitInformation.LimitFlags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if not _k32.SetInformationJobObject(handle, _JOB_EXTENDED_LIMIT_INFORMATION,
                                                ctypes.byref(info), ctypes.sizeof(info)):
                error = ctypes.get_last_error()
                self.close()
                raise OSError(error, "SetInformationJobObject failed")

        def assign(self, pid: int) -> None:
            process = _k32.OpenProcess(_PROCESS_SET_QUOTA | _PROCESS_TERMINATE, False, pid)
            if not process:
                raise OSError(ctypes.get_last_error(), "OpenProcess failed")
            try:
                if not _k32.AssignProcessToJobObject(self.handle, process):
                    raise OSError(ctypes.get_last_error(), "AssignProcessToJobObject failed")
            finally:
                _k32.CloseHandle(process)

        def terminate(self) -> None:
            if not _k32.TerminateJobObject(self.handle, 1):
                raise OSError(ctypes.get_last_error(), "TerminateJobObject failed")

        def active_processes(self) -> int:
            info = _BasicAccounting()
            if not _k32.QueryInformationJobObject(
                    self.handle, _JOB_OBJECT_BASIC_ACCOUNTING_INFORMATION,
                    ctypes.byref(info), ctypes.sizeof(info), None):
                raise OSError(ctypes.get_last_error(), "QueryInformationJobObject failed")
            return int(info.ActiveProcesses)

        def close(self) -> bool:
            handle, self.handle = self.handle, None
            return bool(handle) and bool(_k32.CloseHandle(handle))


class ContainedWorker:
    """One started, contained worker. `proc.stdin`/`proc.stdout` are binary
    pipes; stderr goes to the file the caller supplied."""

    def __init__(self, proc: subprocess.Popen, job):
        self.proc = proc
        self._job = job
        self.containment = "job_object" if _WIN else "process_group"
        self._terminated = False

    @property
    def pid(self) -> int:
        return self.proc.pid

    def terminate_tree(self) -> None:
        """Kill the worker and everything it started. Harmless if they have
        already exited. Raises `OSError` if the OS refuses; there is
        deliberately no weaker fallback. On POSIX, call it only while the
        direct worker is unreaped (see `await_exit`)."""
        self._terminated = True
        if _WIN:
            self._job.terminate()
            return
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)   # session leader: pgid == pid
        except ProcessLookupError:
            pass
        except PermissionError:
            # macOS answers EPERM when the group's only members are exited,
            # unreaped processes (Linux signals them and succeeds). A live member
            # of our own session would have been signalled, so if the direct
            # worker is confirmed exited this means nothing is left to kill;
            # `tree_dead()` still has to verify the group is empty afterwards.
            if hasattr(os, "waitid") or not self._await_exit_kqueue(0.0):
                raise

    def await_exit(self, timeout_s: float) -> bool:
        """Wait for the direct worker to exit on its own, WITHOUT making its
        process-group id reusable: on POSIX the child is observed with
        `waitid(..., WNOWAIT)` and left unreaped, so a following
        `terminate_tree()` can still only reach this worker's own group. On
        Windows the kill target is the job handle, never a pid, so the normal
        handle wait is used."""
        if _WIN:
            return self.wait_dead(timeout_s)
        if not hasattr(os, "waitid"):   # macOS/BSD: no waitid in Python, use kqueue
            return self._await_exit_kqueue(timeout_s)
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            if self.proc.returncode is not None:
                return True
            try:
                info = os.waitid(os.P_PID, self.proc.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
            except ChildProcessError:
                return self.proc.returncode is not None
            if info is not None:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(DEATH_POLL_S)

    def _await_exit_kqueue(self, timeout_s: float) -> bool:
        """`await_exit` for POSIX systems without `os.waitid` (macOS): a kqueue
        NOTE_EXIT event reports the exit and, like `WNOWAIT`, leaves the child
        unreaped. Registering on a child that has already exited fails with
        ESRCH; as the parent has not reaped it, that also means "exited"."""
        if self.proc.returncode is not None:
            return True
        kq = select.kqueue()
        try:
            event = select.kevent(self.proc.pid, filter=select.KQ_FILTER_PROC,
                                  flags=select.KQ_EV_ADD | select.KQ_EV_ONESHOT,
                                  fflags=select.KQ_NOTE_EXIT)
            try:
                ready = kq.control([event], 1, max(0.0, timeout_s))
            except ProcessLookupError:
                return True
            return any(item.ident == self.proc.pid and item.fflags & select.KQ_NOTE_EXIT for item in ready)
        finally:
            kq.close()

    def wait_dead(self, timeout_s: float) -> bool:
        """Whether the direct worker has exited, verified through its own
        `Popen` handle (never by polling a bare pid)."""
        try:
            self.proc.wait(timeout=max(0.0, timeout_s))
        except subprocess.TimeoutExpired:
            return False
        return self.proc.returncode is not None

    def tree_dead(self, timeout_s: float) -> bool:
        """Whether the worker AND every descendant is gone. Windows: the job's
        active-process count reaches 0. POSIX: the process group has no member."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            if self._tree_empty():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(DEATH_POLL_S)

    def _tree_empty(self) -> bool:
        if self.proc.poll() is None:
            return False
        if _WIN:
            return self._job.active_processes() == 0
        # Signal 0 delivers nothing; it only asks whether the worker's own group
        # still has a member. Any doubt reads as "not dead" (the caller fails closed).
        try:
            os.killpg(self.proc.pid, 0)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False
        return False

    def close_pipes(self) -> None:
        """Close the parent's ends of the worker's stdin/stdout pipes."""
        for pipe in (self.proc.stdin, self.proc.stdout):
            if pipe is not None:
                try:
                    pipe.close()
                except (OSError, ValueError):
                    pass

    def close(self) -> bool:
        """Release the OS containment unit. Returns whether it was released
        (POSIX has nothing to release, so it returns True)."""
        if _WIN:
            return self._job.close()
        return True


def start_worker(command, *, cwd, stderr) -> ContainedWorker:
    """Start `command` under containment with binary stdin/stdout pipes.

    The caller must not send the worker any work until this returns: on
    `ContainmentError` the direct worker has been killed and reaped, and it had
    not yet been sent anything. Raises `ContainmentError` on any failure."""
    kwargs = {"stdin": subprocess.PIPE, "stdout": subprocess.PIPE, "stderr": stderr, "cwd": cwd}
    job = None
    if _WIN:
        kwargs["creationflags"] = _CREATE_NEW_PROCESS_GROUP
        try:
            job = _Job()
        except OSError as error:
            raise ContainmentError(f"job object could not be created: {error}") from error
    else:
        kwargs["start_new_session"] = True
    try:
        proc = subprocess.Popen(command, **kwargs)
    except OSError as error:
        if job is not None:
            job.close()
        raise ContainmentError(f"worker could not be started: {error}") from error
    if _WIN:
        try:
            job.assign(proc.pid)
        except OSError as error:
            # Containment setup failed while the direct worker is still gated
            # (it waits for a SPEC that will never come and has started
            # nothing), so killing that one child is the whole cleanup.
            proc.kill()
            try:
                proc.wait(timeout=5.0)
                reaped = True
            except subprocess.TimeoutExpired:
                reaped = False
            job.close()
            if reaped:
                for pipe in (proc.stdin, proc.stdout):
                    pipe.close()
            raise ContainmentError(f"worker could not be placed in a job object: {error}",
                                   worker_reaped=reaped) from error
    return ContainedWorker(proc, job)
