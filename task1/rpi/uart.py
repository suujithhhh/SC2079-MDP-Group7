"""Low-level RPi <-> STM32 UART transport: serial port, line framing, shutdown.

This module knows nothing about the STM command set. `stm_link.py` owns that
(MOV validation, timeouts, ACK/ERR classification, PROFILE, IMU, telemetry).
Here live only: opening and closing the port, writing one ASCII line, turning
the received byte stream into lines, failures, and shutdown.

Wire facts (PROTOCOL.md section 4, RTOSV/Core/Src/vision_rx.c and app_comm.c):

* 115200 baud, 8 data bits, no parity, 1 stop bit, ASCII, LF-terminated.
* The STM parser accepts at most 31 characters per command (VISION_RX_BUF is
  32) and treats any byte outside 0x20..0x7E as a bad line. It accepts a CR
  only directly before the LF; this module therefore sends a bare LF.
* There are no command ids on the wire, so nothing here ever resends.

Receive bound: the longest line the firmware can emit is the `MM,` movement
summary, built in `movementSummaryResponse[APP_COMM_MOVEMENT_BUFFER_SIZE]`
(256 bytes). `snprintf` output is accepted only when its length is less than
256, and that length includes the trailing LF, so a line is at most 255 bytes
with LF, i.e. 254 characters (the source comment estimates ~227). Every other
reply or telemetry buffer in app_comm.c is smaller (160, 96, 80, ...).
`STM_MAX_RX_LINE_CHARS` is that 254. The STM's 31-character limit applies to
what it *receives*, not to what it sends.

Threading: there is no reader thread. Whoever calls `read_line()` is the
reader, and a lock makes that single-reader by construction, so two threads
cannot consume each other's replies. A caller that needs a whole
command/reply exchange to be atomic holds `transaction()` around it.
`shutdown()` wakes a blocked reader (`cancel_read()` where the port has it,
otherwise the port's own read timeout) and closes the port. Nothing is
reconnected and nothing is resent automatically.
"""

from __future__ import annotations

import collections
import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol

DEFAULT_BAUDRATE = 115200
STM_MAX_COMMAND_CHARS = 31   # vision_rx.h VISION_RX_BUF (32) minus the NUL
STM_MAX_RX_LINE_CHARS = 254  # app_comm.c: 256-byte buffer, minus NUL, minus LF
PRINTABLE_ASCII = range(0x20, 0x7F)


class UartError(Exception):
    """Base class of every transport failure."""


class UartOpenError(UartError):
    """The serial device could not be opened (missing, busy, no permission, ...)."""


class UartCommandError(UartError, ValueError):
    """The text is not a sendable command line. Nothing was written."""


class UartWriteError(UartError):
    """Writing failed or was short. How much reached the STM is unknown."""


class UartDisconnectedError(UartError):
    """Reading failed, or the port had already failed. The device is gone or broken."""


class UartFramingError(UartError):
    """One received line was malformed or too long. It was discarded and the
    stream is back in sync; the next `read_line()` continues normally."""


class UartShutdownError(UartError):
    """The transport was shut down (or is shutting down)."""


class SerialPortLike(Protocol):
    """What the transport needs from a port (pyserial's `Serial` provides it).

    `read(size)` returns up to `size` bytes and `b""` when its timeout passes.
    Optional: `in_waiting`, `flush()`, `cancel_read()`.
    """

    def read(self, size: int) -> bytes: ...
    def write(self, data: bytes) -> int | None: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class UartConfig:
    baudrate: int = DEFAULT_BAUDRATE
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 1
    read_timeout_s: float = 0.1     # bounds how long one port read (and so a shutdown) can block
    write_timeout_s: float = 1.0
    max_command_chars: int = STM_MAX_COMMAND_CHARS
    max_line_chars: int = STM_MAX_RX_LINE_CHARS
    read_chunk_bytes: int = 512
    shutdown_wait_s: float = 1.0    # how long shutdown() lets a blocked reader leave before closing

    def __post_init__(self) -> None:
        positive = ("baudrate", "read_timeout_s", "write_timeout_s", "max_command_chars",
                    "max_line_chars", "read_chunk_bytes", "shutdown_wait_s")
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"UartConfig.{name} must be positive")


class UartTransport:
    def __init__(
        self,
        port: SerialPortLike,
        config: UartConfig = UartConfig(),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._port = port
        self._config = config
        self._clock = clock
        self._closed = threading.Event()
        self._state_lock = threading.Lock()
        self._transaction_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._fault: UartError | None = None
        self._buffer = bytearray()                  # bytes of the line being assembled
        self._discarding = False                    # inside an overlong line, dropping to its LF
        self._ready: collections.deque[str | UartFramingError] = collections.deque()
        self.lines_received = 0
        self.malformed_lines = 0
        self.overlong_lines = 0
        self.discarded_bytes = 0

    # -- state -----------------------------------------------------------
    @property
    def config(self) -> UartConfig:
        return self._config

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    @property
    def fault(self) -> UartError | None:
        """The first write/read failure; once set the transport refuses further I/O."""
        return self._fault

    def transaction(self) -> threading.RLock:
        """Lock to hold around one whole command/reply exchange."""
        return self._transaction_lock

    def __enter__(self) -> "UartTransport":
        return self

    def __exit__(self, *_: object) -> None:
        self.shutdown()

    def _fail(self, error: UartError) -> UartError:
        with self._state_lock:
            if self._fault is None:
                self._fault = error
        return error

    def _check_open(self) -> None:
        if self._closed.is_set():
            raise UartShutdownError("UART is shut down")

    def _check_usable(self) -> None:
        self._check_open()
        if self._fault is not None:
            raise UartDisconnectedError(f"UART unusable after an earlier failure: {self._fault}")

    # -- transmit --------------------------------------------------------
    def _encode(self, command: str) -> bytes:
        if not isinstance(command, str) or not command:
            raise UartCommandError("a command must be a non-empty string")
        if "\r" in command or "\n" in command:
            raise UartCommandError("a command must be a single line (no CR or LF)")
        try:
            raw = command.encode("ascii")
        except UnicodeEncodeError as exc:
            raise UartCommandError(f"command is not ASCII: {exc.reason} at {exc.start}") from exc
        if any(byte not in PRINTABLE_ASCII for byte in raw):
            raise UartCommandError("command contains a control character the STM would reject")
        if len(raw) > self._config.max_command_chars:
            raise UartCommandError(
                f"command is {len(raw)} characters; the STM accepts at most {self._config.max_command_chars}")
        return raw + b"\n"

    def send_line(self, command: str) -> None:
        """Write `command` plus exactly one LF, completely, or raise.

        After any failure here the outcome is unknown and the line is NOT
        retried; the transport also refuses further I/O (`fault`).
        """
        data = self._encode(command)
        with self._write_lock:
            self._check_usable()
            try:
                written = self._port.write(data)
                flush = getattr(self._port, "flush", None)
                if flush is not None:
                    flush()
            except (OSError, ValueError) as exc:
                self._check_open()
                raise self._fail(UartWriteError(f"write failed: {exc}")) from exc
            if written is not None and written != len(data):
                raise self._fail(UartWriteError(f"short write: {written} of {len(data)} bytes"))

    # -- receive ---------------------------------------------------------
    def read_line(self, timeout_s: float | None) -> str | None:
        """Next complete received line without its terminator, or None when
        `timeout_s` passes first (None waits until a line, a failure or shutdown).

        Empty lines are skipped. Raises `UartFramingError` for one malformed or
        overlong line (already discarded; keep calling), `UartDisconnectedError`
        when the port fails, and `UartShutdownError` once shut down.
        """
        deadline = None if timeout_s is None else self._clock() + max(0.0, timeout_s)
        with self._read_lock:
            while True:
                self._check_open()
                if self._ready:
                    item = self._ready.popleft()
                    if isinstance(item, UartFramingError):
                        raise item
                    return item
                self._check_usable()
                self._pump()
                if self._ready:
                    continue
                if deadline is not None and self._clock() >= deadline:
                    return None

    def _pump(self) -> None:
        """One bounded port read, fed to the line assembler."""
        try:
            waiting = getattr(self._port, "in_waiting", 0)
            size = min(max(int(waiting or 0), 1), self._config.read_chunk_bytes)
            data = self._port.read(size)
        except (OSError, ValueError) as exc:
            self._check_open()
            raise self._fail(UartDisconnectedError(f"read failed: {exc}")) from exc
        self._check_open()
        if data:
            self._feed(bytes(data))

    def _feed(self, data: bytes) -> None:
        limit = self._config.max_line_chars
        while data:
            newline = data.find(b"\n")
            if newline < 0:
                if self._discarding:
                    self.discarded_bytes += len(data)
                elif len(self._buffer) + len(data) > limit + 1:   # +1: a CR may precede the LF
                    self.discarded_bytes += len(self._buffer) + len(data)
                    self._buffer.clear()
                    self._discarding = True
                    self._report_overlong()
                else:
                    self._buffer += data
                return
            segment, data = data[:newline], data[newline + 1:]
            if self._discarding:                       # this LF ends the overlong line
                self._discarding = False
                self.discarded_bytes += len(segment)
            elif len(self._buffer) + len(segment) > limit + 1:
                self.discarded_bytes += len(self._buffer) + len(segment)
                self._buffer.clear()
                self._report_overlong()
            else:
                raw = bytes(self._buffer + segment)
                self._buffer.clear()
                self._finish_line(raw)

    def _report_overlong(self) -> None:
        self.overlong_lines += 1
        self._ready.append(UartFramingError(
            f"received line longer than {self._config.max_line_chars} characters; discarded to the next LF"))

    def _finish_line(self, raw: bytes) -> None:
        content = raw[:-1] if raw.endswith(b"\r") else raw
        if len(content) > self._config.max_line_chars:
            self.discarded_bytes += len(raw)
            self._report_overlong()
            return
        if not content:
            return
        for position, byte in enumerate(content):
            if byte not in PRINTABLE_ASCII:
                self.malformed_lines += 1
                self.discarded_bytes += len(raw)
                self._ready.append(UartFramingError(
                    f"received line has non-printable/non-ASCII byte 0x{byte:02x} at position {position}"))
                return
        self.lines_received += 1
        self._ready.append(content.decode("ascii"))

    def discard_input(self) -> int:
        """Drop everything received but not yet consumed (stale telemetry before a
        new command). Returns how many already-buffered bytes were dropped; bytes
        still inside the OS driver are cleared too but cannot be counted."""
        with self._read_lock:
            self._check_usable()
            dropped = len(self._buffer) + sum(len(item) + 1 for item in self._ready if isinstance(item, str))
            self._buffer.clear()
            self._ready.clear()
            self._discarding = False
            try:
                self._port.reset_input_buffer()
            except (OSError, ValueError) as exc:
                self._check_open()
                raise self._fail(UartDisconnectedError(f"input reset failed: {exc}")) from exc
            self.discarded_bytes += dropped
            return dropped

    # -- shutdown --------------------------------------------------------
    def shutdown(self) -> None:
        """Wake any blocked reader, then close the port. Safe to call repeatedly
        and from any thread; never writes to the port."""
        with self._state_lock:
            first = not self._closed.is_set()
            self._closed.set()
        if not first:
            return
        cancel = getattr(self._port, "cancel_read", None)
        if cancel is not None:
            try:
                cancel()
            except Exception:       # a best-effort wake-up; closing below is the fallback
                pass
        # A reader leaves within one port read timeout (or at once after cancel_read);
        # waiting here keeps close() from racing a read, but is bounded.
        got = self._read_lock.acquire(timeout=self._config.shutdown_wait_s)
        try:
            self._port.close()
        except (OSError, ValueError):
            pass
        finally:
            if got:
                self._read_lock.release()

    close = shutdown


def open_uart(
    device: str,
    config: UartConfig = UartConfig(),
    *,
    clock: Callable[[], float] = time.monotonic,
    serial_factory: Callable[..., SerialPortLike] | None = None,
) -> UartTransport:
    """Open `device` (e.g. /dev/ttyACM0 or a /dev/serial/by-id/... path) with
    pyserial using `config` and wrap it. Raises `UartOpenError`."""
    if not isinstance(device, str) or not device:
        raise UartOpenError("no serial device given")
    if serial_factory is None:
        try:
            import serial  # pyserial; imported here so tests need no hardware
        except ImportError as exc:
            raise UartOpenError("pyserial is not installed (pip install pyserial)") from exc
        serial_factory = serial.Serial
    try:
        port = serial_factory(
            port=device, baudrate=config.baudrate, bytesize=config.bytesize, parity=config.parity,
            stopbits=config.stopbits, timeout=config.read_timeout_s, write_timeout=config.write_timeout_s,
            xonxoff=False, rtscts=False, dsrdtr=False)
    except (OSError, ValueError) as exc:       # serial.SerialException is an OSError
        raise UartOpenError(f"cannot open {device}: {exc}") from exc
    return UartTransport(port, config, clock)
