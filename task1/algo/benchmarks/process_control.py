"""Terminable benchmark workers (measurement tooling only).

Every benchmark case runs in its own worker process. `run_worker()` starts it
inside an OS containment unit, enforces a hard timeout, kills it, and then
VERIFIES it is dead before returning; if death cannot be verified it raises
`CleanupError` and the caller must abort the whole campaign.

* Windows: the worker is assigned to a Job Object created with
  `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` (and, optionally, a per-process memory
  limit). Terminating or closing the job kills the worker and anything it
  started, and because this process holds the only job handle, the worker dies
  if this process dies. If the job cannot be set up (for example a process
  that is already in a non-nestable job), the fallback is `Popen.kill()` and
  the result says so (`containment: "process_kill_fallback"`).
* POSIX: the worker gets its own session and is killed with `killpg`.

Death is verified through the child's own handle (`Popen.wait`), never by
polling a bare PID, and never with `os.kill(pid, 0)` on Windows (signal 0 is
`CTRL_C_EVENT` there). `process_alive()` is a separate helper used by tests.

On Windows, after the worker's death is verified and BEFORE the job is closed,
`run_worker()` reads the job's own accounting (`WorkerOutcome.job_report`): how
many processes are still active in it (anything the worker started and left
behind, nested jobs included) and the job's peak committed memory. That is the
harness-side proof that the tree is empty; nothing enumerates system processes.

A harness kill is a HARNESS timeout. It is never planner metadata: the planner's
own checkpoint stop and the future production hard backstop are different
things (see README).
"""

import os
import signal
import subprocess
import sys
from dataclasses import dataclass
from typing import Optional

KILL_WAIT_S = 5.0
DEFAULT_MEMORY_CAP_BYTES = 2 * 1024 ** 3
TIMEOUT_KIND = "harness_hard_timeout"

_WIN = sys.platform == "win32"


class CleanupError(RuntimeError):
    """A worker's death could not be verified. `cleanup` holds what is known."""

    def __init__(self, message: str, cleanup: dict):
        super().__init__(message)
        self.cleanup = cleanup


@dataclass(frozen=True)
class WorkerOutcome:
    stdout: str
    stderr: str
    returncode: int
    timed_out: bool
    cleanup: dict
    job_report: Optional[dict] = None   # Windows job accounting read before the job was closed


if _WIN:
    import ctypes
    from ctypes import wintypes

    _JOB_OBJECT_LIMIT_PROCESS_MEMORY = 0x00000100
    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x00002000
    _JOB_EXTENDED_LIMIT_INFORMATION = 9
    _JOB_BASIC_ACCOUNTING_INFORMATION = 1
    _PROCESS_SET_QUOTA = 0x0100
    _PROCESS_TERMINATE = 0x0001
    _PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
    _STILL_ACTIVE = 259
    _ERROR_ACCESS_DENIED = 5
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
    _k32.QueryInformationJobObject.argtypes = [
        wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    _k32.QueryInformationJobObject.restype = wintypes.BOOL
    _k32.CreateJobObjectW.argtypes = [wintypes.LPVOID, wintypes.LPCWSTR]
    _k32.CreateJobObjectW.restype = wintypes.HANDLE
    _k32.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD]
    _k32.SetInformationJobObject.restype = wintypes.BOOL
    _k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _k32.OpenProcess.restype = wintypes.HANDLE
    _k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _k32.AssignProcessToJobObject.restype = wintypes.BOOL
    _k32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    _k32.TerminateJobObject.restype = wintypes.BOOL
    _k32.CloseHandle.argtypes = [wintypes.HANDLE]
    _k32.CloseHandle.restype = wintypes.BOOL
    _k32.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    _k32.GetExitCodeProcess.restype = wintypes.BOOL

    class _Job:
        """A kill-on-close Job Object holding one worker."""

        def __init__(self, memory_cap_bytes):
            handle = _k32.CreateJobObjectW(None, None)
            if not handle:
                raise OSError(ctypes.get_last_error(), "CreateJobObjectW failed")
            self.handle = handle
            info = _ExtendedLimits()
            flags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if memory_cap_bytes:
                flags |= _JOB_OBJECT_LIMIT_PROCESS_MEMORY
                info.ProcessMemoryLimit = int(memory_cap_bytes)
            info.BasicLimitInformation.LimitFlags = flags
            if not _k32.SetInformationJobObject(handle, _JOB_EXTENDED_LIMIT_INFORMATION,
                                                ctypes.byref(info), ctypes.sizeof(info)):
                error = ctypes.get_last_error()
                self.close()
                raise OSError(error, "SetInformationJobObject failed")

        def assign(self, pid):
            process = _k32.OpenProcess(_PROCESS_SET_QUOTA | _PROCESS_TERMINATE, False, pid)
            if not process:
                raise OSError(ctypes.get_last_error(), "OpenProcess failed")
            try:
                if not _k32.AssignProcessToJobObject(self.handle, process):
                    raise OSError(ctypes.get_last_error(), "AssignProcessToJobObject failed")
            finally:
                _k32.CloseHandle(process)

        def terminate(self):
            if self.handle:
                _k32.TerminateJobObject(self.handle, 1)

        def report(self):
            """Active processes and peak committed memory of this job, or None if
            the job cannot be queried. Diagnostic only: committed memory is not RSS."""
            if not self.handle:
                return None
            basic, extended = _BasicAccounting(), _ExtendedLimits()
            if not _k32.QueryInformationJobObject(self.handle, _JOB_BASIC_ACCOUNTING_INFORMATION,
                                                  ctypes.byref(basic), ctypes.sizeof(basic), None):
                return None
            if not _k32.QueryInformationJobObject(self.handle, _JOB_EXTENDED_LIMIT_INFORMATION,
                                                  ctypes.byref(extended), ctypes.sizeof(extended), None):
                return None
            return {"active_processes": int(basic.ActiveProcesses),
                    "peak_process_commit_bytes": int(extended.PeakProcessMemoryUsed),
                    "peak_job_commit_bytes": int(extended.PeakJobMemoryUsed)}

        def close(self) -> bool:
            handle, self.handle = self.handle, None
            return bool(handle) and bool(_k32.CloseHandle(handle))


def process_alive(pid: int) -> bool:
    """Whether a process with this PID currently exists. For tests: production
    code verifies death through the child's own handle instead."""
    if _WIN:
        handle = _k32.OpenProcess(_PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not handle:
            return ctypes.get_last_error() == _ERROR_ACCESS_DENIED    # exists, but not ours to query
        try:
            code = wintypes.DWORD()
            if not _k32.GetExitCodeProcess(handle, ctypes.byref(code)):
                return True
            return code.value == _STILL_ACTIVE
        finally:
            _k32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class ContainedProcess:
    """A started worker plus its containment. Use `run_worker` unless a test
    needs the pieces separately."""

    def __init__(self, proc, job):
        self.proc = proc
        self.job = job
        self.containment = "job_object" if job is not None else (
            "process_kill_fallback" if _WIN else "process_group")

    @property
    def pid(self) -> int:
        return self.proc.pid

    def kill(self) -> None:
        if self.job is not None:
            self.job.terminate()
        elif _WIN:
            self.proc.kill()
        else:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                self.proc.kill()

    def verify_dead(self) -> bool:
        try:
            self.proc.wait(timeout=KILL_WAIT_S)
        except subprocess.TimeoutExpired:
            return False
        return self.proc.returncode is not None

    def job_report(self):
        """The job's accounting (`_Job.report`), or None without a job. Call it
        before `close()`."""
        return self.job.report() if self.job is not None else None

    def close(self) -> bool:
        """Kill anything still running, then release the job. Returns whether a
        job handle was closed."""
        if self.proc.poll() is None:
            self.kill()
        return self.job.close() if self.job is not None else False


def start_contained(command, *, cwd=None, memory_cap_bytes=None) -> ContainedProcess:
    kwargs = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE, "text": True, "cwd": cwd}
    if _WIN:
        kwargs["creationflags"] = _CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True
    proc = subprocess.Popen(command, **kwargs)
    job = None
    if _WIN:
        try:
            job = _Job(memory_cap_bytes)
            job.assign(proc.pid)
        except OSError:
            if job is not None:
                job.close()
            job = None
    return ContainedProcess(proc, job)


def run_worker(command, timeout_s: float, *, cwd=None,
               memory_cap_bytes=DEFAULT_MEMORY_CAP_BYTES) -> WorkerOutcome:
    """Run `command` under containment with a hard timeout. Raises
    `CleanupError` if the worker's death cannot be verified."""
    contained = start_contained(command, cwd=cwd, memory_cap_bytes=memory_cap_bytes)
    timed_out = False
    stdout = stderr = ""
    verified = False
    job_closed = False
    job_report = None
    try:
        try:
            stdout, stderr = contained.proc.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            contained.kill()
            try:
                stdout, stderr = contained.proc.communicate(timeout=KILL_WAIT_S)
            except subprocess.TimeoutExpired:
                stdout = stderr = ""
        verified = contained.verify_dead()
        if verified:
            job_report = contained.job_report()     # before close: anything left is still counted
    finally:
        job_closed = contained.close()
    cleanup = {
        "containment": contained.containment,
        "timeout_kind": TIMEOUT_KIND if timed_out else None,
        "killed": timed_out,
        "death_verified": verified,
        "exit_code": contained.proc.returncode,
        "job_closed": job_closed,
        "tempdir_removed": None,      # filled in by the caller that owns the scratch directory
    }
    if not verified:
        raise CleanupError(f"worker pid {contained.pid} could not be verified dead", cleanup)
    return WorkerOutcome(stdout, stderr, contained.proc.returncode, timed_out, cleanup, job_report)
