"""Hardware-free tests for uart.py (the serial transport). Run from rpi_task1/:
    python -m unittest discover -s tests -v
The fake ports below stand in for pyserial's Serial; nothing opens a device.
"""

import collections
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uart import (STM_MAX_COMMAND_CHARS, STM_MAX_RX_LINE_CHARS, UartCommandError, UartConfig,  # noqa: E402
                  UartDisconnectedError, UartFramingError, UartOpenError, UartShutdownError,
                  UartTransport, UartWriteError, open_uart)


class ManualClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class ScriptedPort:
    """Byte-level fake of pyserial's Serial. `rx` holds chunks to be read one
    read() at a time; `replies` maps a written command to chunks queued when it
    arrives. An empty read advances the clock by one port timeout (0.1 s)."""

    def __init__(self, clock=None, replies=None):
        self.clock = clock
        self.replies = dict(replies or {})
        self.rx = collections.deque()
        self.written = []
        self.flushes = 0
        self.resets = 0
        self.close_calls = 0
        self.closed = False
        self.read_sizes = []
        self.write_error = None
        self.short_write = None
        self.error_when_empty = None     # raised by read() once rx has run dry
        self.ignore_size = False
        self.lock = threading.Lock()

    @property
    def in_waiting(self):
        return len(self.rx[0]) if self.rx else 0

    def feed(self, *chunks):
        self.rx.extend(chunks)

    def write(self, data):
        if self.closed:
            raise OSError("write to closed port")
        if self.write_error is not None:
            raise self.write_error
        with self.lock:
            self.written.append(bytes(data))
            self.rx.extend(self.replies.get(bytes(data).decode().rstrip("\n"), ()))
        return len(data) if self.short_write is None else self.short_write

    def flush(self):
        self.flushes += 1

    def reset_input_buffer(self):
        self.resets += 1
        self.rx.clear()

    def read(self, size):
        if self.closed:
            raise OSError("read from closed port")
        self.read_sizes.append(size)
        with self.lock:
            if self.rx:
                chunk = self.rx.popleft()
                if not self.ignore_size and len(chunk) > size:
                    chunk, rest = chunk[:size], chunk[size:]
                    self.rx.appendleft(rest)
                return chunk
        if self.error_when_empty is not None:
            raise self.error_when_empty
        if self.clock is not None:
            self.clock.advance(0.1)
        return b""

    def close(self):
        self.close_calls += 1
        self.closed = True


class BlockingPort(ScriptedPort):
    """read() really blocks (up to `timeout`), and cancel_read() wakes it."""

    def __init__(self, timeout=0.05):
        super().__init__()
        self.timeout = timeout
        self.waiting = threading.Event()
        self._wake = threading.Event()

    def read(self, size):
        if self.rx or self.closed:
            return super().read(size)
        self.waiting.set()
        self._wake.wait(self.timeout)
        self._wake.clear()
        return super().read(size)

    def cancel_read(self):
        self._wake.set()


class BlockingPortNoCancel(BlockingPort):
    cancel_read = None      # an older/other port without cancel_read: only the read timeout wakes it


def bytes_in_chunks(data, size):
    return [data[i:i + size] for i in range(0, len(data), size)]


def transport(port=None, clock=None, **config):
    clock = clock or ManualClock()
    port = port or ScriptedPort(clock)
    return UartTransport(port, UartConfig(**config), clock=clock), port, clock


class OpenCloseTests(unittest.TestCase):
    def test_open_uses_exact_serial_settings(self):
        port = ScriptedPort()
        factory = mock.Mock(return_value=port)
        uart = open_uart("/dev/serial/by-id/example", serial_factory=factory)
        kwargs = factory.call_args.kwargs
        self.assertEqual(kwargs["port"], "/dev/serial/by-id/example")
        self.assertEqual((kwargs["baudrate"], kwargs["bytesize"], kwargs["parity"], kwargs["stopbits"]),
                         (115200, 8, "N", 1))
        self.assertGreater(kwargs["timeout"], 0)          # finite read timeout
        self.assertGreater(kwargs["write_timeout"], 0)    # finite write timeout
        self.assertEqual((kwargs["xonxoff"], kwargs["rtscts"], kwargs["dsrdtr"]), (False, False, False))
        self.assertFalse(uart.closed)
        uart.close()
        self.assertTrue(port.closed)
        self.assertTrue(uart.closed)

    def test_baud_comes_from_config_not_a_constant(self):
        factory = mock.Mock(return_value=ScriptedPort())
        open_uart("/dev/x", UartConfig(baudrate=9600), serial_factory=factory)
        self.assertEqual(factory.call_args.kwargs["baudrate"], 9600)

    def test_open_failures_become_uart_open_error(self):
        for error in (OSError("No such file or directory"), PermissionError("denied"), ValueError("bad baud")):
            with self.assertRaises(UartOpenError) as caught:
                open_uart("/dev/ttyACM9", serial_factory=mock.Mock(side_effect=error))
            self.assertIn("/dev/ttyACM9", str(caught.exception))
        with self.assertRaises(UartOpenError):
            open_uart("")

    def test_real_pyserial_failure_is_wrapped(self):
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest("pyserial not installed")
        with self.assertRaises(UartOpenError):
            open_uart("/dev/ttyACM_does_not_exist_for_tests")

    def test_context_manager_closes(self):
        port = ScriptedPort()
        with UartTransport(port) as uart:
            self.assertFalse(uart.closed)
        self.assertTrue(port.closed)

    def test_repeated_shutdown_and_close_are_harmless(self):
        uart, port, _ = transport()
        uart.shutdown()
        uart.shutdown()
        uart.close()
        self.assertEqual(port.close_calls, 1)
        self.assertEqual(port.written, [])               # shutdown never writes

    def test_close_errors_are_swallowed(self):
        uart, port, _ = transport()
        port.close = mock.Mock(side_effect=OSError("already gone"))
        uart.shutdown()
        self.assertTrue(uart.closed)

    def test_config_defaults_and_validation(self):
        config = UartConfig()
        self.assertEqual((config.baudrate, config.bytesize, config.parity, config.stopbits), (115200, 8, "N", 1))
        self.assertEqual(config.max_command_chars, 31)
        self.assertEqual(config.max_line_chars, 254)
        for bad in ({"baudrate": 0}, {"read_timeout_s": 0}, {"max_line_chars": -1}):
            with self.assertRaises(ValueError):
                UartConfig(**bad)


class SendTests(unittest.TestCase):
    def test_exact_bytes_with_one_final_lf(self):
        uart, port, _ = transport()
        uart.send_line("MOV,FL,90")
        self.assertEqual(port.written, [b"MOV,FL,90\n"])
        self.assertEqual(port.flushes, 1)

    def test_rejects_anything_but_one_ascii_line_and_writes_nothing(self):
        uart, port, _ = transport()
        for bad in ("MOV,F,3\n", "MOV,F,3\r", "MOV,F,3\r\n", "MO\nV", "MOV,F,é", "MOV,F,30–",
                    "", "TAB\there", "NUL\x00", "x" * (STM_MAX_COMMAND_CHARS + 1), None, b"ACK"):
            with self.assertRaises(UartCommandError, msg=repr(bad)):
                uart.send_line(bad)
        self.assertEqual(port.written, [])
        self.assertFalse(uart.closed)
        self.assertIsNone(uart.fault)                    # a bad command is not a link failure

    def test_longest_stm_command_is_accepted(self):
        uart, port, _ = transport()
        uart.send_line("x" * STM_MAX_COMMAND_CHARS)
        self.assertEqual(len(port.written[0]), STM_MAX_COMMAND_CHARS + 1)

    def test_no_id_or_extra_text_is_added(self):
        uart, port, _ = transport()
        uart.send_line("IMU,GET")
        self.assertEqual(port.written, [b"IMU,GET\n"])

    def test_write_failure_is_reported_and_not_retried(self):
        uart, port, _ = transport()
        port.write_error = OSError("device reports readiness to write but write failed")
        with self.assertRaises(UartWriteError):
            uart.send_line("MOV,F,30")
        port.write_error = None
        self.assertEqual(port.written, [])
        self.assertIsInstance(uart.fault, UartWriteError)
        with self.assertRaises(UartDisconnectedError):   # sticky: no further I/O after a failed write
            uart.send_line("MOV,F,30")
        self.assertEqual(port.written, [])

    def test_short_write_is_a_failure_and_not_retried(self):
        uart, port, _ = transport()
        port.short_write = 4
        with self.assertRaises(UartWriteError) as caught:
            uart.send_line("MOV,F,30")
        self.assertIn("short write", str(caught.exception))
        self.assertEqual(len(port.written), 1)           # one attempt only
        with self.assertRaises(UartDisconnectedError):
            uart.send_line("MOV,F,30")
        self.assertEqual(len(port.written), 1)

    def test_send_after_shutdown_raises_and_writes_nothing(self):
        uart, port, _ = transport()
        uart.shutdown()
        with self.assertRaises(UartShutdownError):
            uart.send_line("MOV,F,30")
        self.assertEqual(port.written, [])


class ReceiveTests(unittest.TestCase):
    def test_reply_fragmented_one_byte_at_a_time(self):
        uart, port, _ = transport()
        port.feed(*bytes_in_chunks(b"ACK\n", 1))
        self.assertEqual(uart.read_line(5), "ACK")

    def test_reply_split_across_arbitrary_chunks(self):
        uart, port, _ = transport()
        port.feed(b"MM,EV:ST", b"ART,D:F,T:0,", b"V:30", b"\nAC", b"K", b"\n")
        self.assertEqual(uart.read_line(5), "MM,EV:START,D:F,T:0,V:30")
        self.assertEqual(uart.read_line(5), "ACK")

    def test_several_lines_in_one_read(self):
        uart, port, _ = transport()
        port.feed(b"MM,a\nAK,b\nTB,c\nACK\n")
        self.assertEqual([uart.read_line(5) for _ in range(4)], ["MM,a", "AK,b", "TB,c", "ACK"])
        self.assertEqual(len(port.read_sizes), 1)        # one port read produced all four

    def test_crlf_and_lone_cr(self):
        uart, port, _ = transport()
        port.feed(b"ACK\r\n", b"\r\n", b"\n", b"PROFILE,OK\r", b"\n")
        self.assertEqual(uart.read_line(5), "ACK")
        self.assertEqual(uart.read_line(5), "PROFILE,OK")    # empty lines skipped
        port.feed(b"AC\rK\n", b"ACK\n")
        with self.assertRaises(UartFramingError):            # CR in the middle is not a terminator
            uart.read_line(5)
        self.assertEqual(uart.read_line(5), "ACK")

    def test_timeout_returns_none_and_keeps_a_partial_line(self):
        uart, port, clock = transport()
        port.feed(b"AC")
        self.assertIsNone(uart.read_line(0.5))
        self.assertGreaterEqual(clock.now, 0.5)
        port.feed(b"K\n")
        self.assertEqual(uart.read_line(5), "ACK")

    def test_zero_timeout_still_polls_once(self):
        uart, port, _ = transport()
        port.feed(b"ACK\n")
        self.assertEqual(uart.read_line(0), "ACK")

    def test_non_ascii_and_control_bytes_are_dropped_with_a_framing_error(self):
        uart, port, _ = transport()
        port.feed(b"AC\xffK\n", b"A\x00CK\n", b"A\tCK\n", b"ACK\n")
        for _ in range(3):
            with self.assertRaises(UartFramingError):
                uart.read_line(5)
        self.assertEqual(uart.read_line(5), "ACK")
        self.assertEqual(uart.malformed_lines, 3)

    def test_longest_legitimate_line_is_accepted_and_one_more_is_not(self):
        uart, port, _ = transport()
        longest = "M" * STM_MAX_RX_LINE_CHARS
        port.feed((longest + "\n").encode(), (longest + "\r\n").encode())
        self.assertEqual(uart.read_line(5), longest)
        self.assertEqual(uart.read_line(5), longest)
        for ending in (b"\n", b"\r\n"):
            port.feed(b"M" * (STM_MAX_RX_LINE_CHARS + 1) + ending, b"ACK\n")
            with self.assertRaises(UartFramingError):
                uart.read_line(5)
            self.assertEqual(uart.read_line(5), "ACK")      # resynchronised at the next line

    def test_bound_matches_the_firmware_buffer(self):
        # app_comm.c: movementSummaryResponse[256]; snprintf must return < 256 including the LF
        self.assertEqual(STM_MAX_RX_LINE_CHARS, 256 - 1 - 1)
        worst_mm = ("MM,D:FL,T:4294967295,ST:RECOVERY,TH:-2147483648,H:-2147483648,HE:-2147483648,"
                    "GZ:-2147483648,SP:4294967295,LT:-2147483648,LP:-2147483648,LE:-2147483648,"
                    "LO:-2147483648,RT:-2147483648,RP:-2147483648,RE:-2147483648,RO:-2147483648")
        self.assertLessEqual(len(worst_mm), STM_MAX_RX_LINE_CHARS)    # 16 fields at their widest

    def test_overlong_stream_without_newline_stays_bounded_and_recovers(self):
        uart, port, _ = transport()
        port.feed(*[b"A" * 100] * 300)                       # 30 kB with no LF
        errors, peak = 0, 0
        for _ in range(300):
            try:
                uart.read_line(0)
            except UartFramingError:
                errors += 1
            peak = max(peak, len(uart._buffer))
        self.assertEqual(errors, 1)                          # one error for the whole overlong line
        self.assertLessEqual(peak, STM_MAX_RX_LINE_CHARS + 1)
        self.assertGreater(uart.discarded_bytes, 29000)
        port.feed(b"tail\nACK\n")                            # the LF ends the discard
        self.assertEqual(uart.read_line(5), "ACK")
        self.assertEqual(uart.overlong_lines, 1)

    def test_one_huge_read_is_still_bounded(self):
        uart, port, _ = transport()
        port.ignore_size = True
        port.feed(b"B" * 200_000 + b"\nACK\n")
        with self.assertRaises(UartFramingError):
            uart.read_line(5)
        self.assertEqual(uart.read_line(5), "ACK")
        self.assertLessEqual(len(uart._buffer), STM_MAX_RX_LINE_CHARS + 1)

    def test_read_requests_are_bounded_by_the_chunk_size(self):
        uart, port, _ = transport(read_chunk_bytes=16)
        port.feed(b"x" * 40 + b"\n")
        self.assertEqual(uart.read_line(5), "x" * 40)
        self.assertGreater(len(port.read_sizes), 1)
        self.assertTrue(all(size <= 16 for size in port.read_sizes))

    def test_disconnect_while_waiting(self):
        uart, port, _ = transport()
        port.error_when_empty = OSError("device reports readiness to read but returned no data")
        with self.assertRaises(UartDisconnectedError):
            uart.read_line(5)
        self.assertIsInstance(uart.fault, UartDisconnectedError)
        with self.assertRaises(UartDisconnectedError):       # sticky
            uart.read_line(5)
        with self.assertRaises(UartDisconnectedError):
            uart.send_line("IMU,GET")
        self.assertEqual(port.written, [])

    def test_lines_received_before_a_disconnect_are_still_delivered(self):
        uart, port, _ = transport()
        port.feed(b"MM,a\nACK\n")
        port.error_when_empty = OSError("gone")
        self.assertEqual(uart.read_line(5), "MM,a")
        self.assertEqual(uart.read_line(5), "ACK")
        with self.assertRaises(UartDisconnectedError):
            uart.read_line(5)

    def test_input_reset_failure_is_a_disconnect(self):
        uart, port, _ = transport()
        port.reset_input_buffer = mock.Mock(side_effect=OSError("gone"))
        with self.assertRaises(UartDisconnectedError):
            uart.discard_input()

    def test_discard_input_drops_buffered_and_pending_data(self):
        uart, port, _ = transport()
        port.feed(b"stale1\nstale2\nPART")
        self.assertEqual(uart.read_line(0), "stale1")        # stale2 is complete, PART is half a line
        dropped = uart.discard_input()
        self.assertEqual(dropped, len("stale2\n") + len("PART"))
        self.assertEqual(port.resets, 1)
        self.assertIsNone(uart.read_line(0.2))
        port.feed(b"ACK\n")
        self.assertEqual(uart.read_line(5), "ACK")


class ShutdownTests(unittest.TestCase):
    def reader(self, uart):
        outcome = {}

        def run():
            try:
                outcome["value"] = uart.read_line(30)
            except Exception as exc:       # noqa: BLE001 - the test inspects it
                outcome["error"] = exc

        thread = threading.Thread(target=run)
        thread.start()
        return thread, outcome

    def check_unblocks(self, port):
        uart = UartTransport(port, UartConfig(read_timeout_s=0.05))
        thread, outcome = self.reader(uart)
        self.assertTrue(port.waiting.wait(2))
        started = time.monotonic()
        uart.shutdown()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertLess(time.monotonic() - started, 2)
        self.assertIsInstance(outcome.get("error"), UartShutdownError)
        self.assertTrue(port.closed)
        self.assertEqual(port.written, [])
        uart.shutdown()                                      # again: harmless
        self.assertEqual(port.close_calls, 1)

    def test_shutdown_unblocks_a_pending_read_via_cancel_read(self):
        self.check_unblocks(BlockingPort(timeout=10))        # only cancel_read can wake this in time

    def test_shutdown_unblocks_a_pending_read_without_cancel_read(self):
        self.check_unblocks(BlockingPortNoCancel(timeout=0.05))

    def test_no_reader_thread_is_left_behind(self):
        before = {t for t in threading.enumerate()}
        uart, port, _ = transport()
        port.feed(b"ACK\n")
        uart.read_line(1)
        uart.shutdown()
        self.assertEqual({t for t in threading.enumerate()} - before, set())

    def test_cancel_read_errors_do_not_stop_the_close(self):
        uart, port, _ = transport()
        port.cancel_read = mock.Mock(side_effect=RuntimeError("unsupported"))
        uart.shutdown()
        self.assertTrue(port.closed)


class TransactionTests(unittest.TestCase):
    def test_transaction_lock_is_exclusive_across_threads_and_reentrant(self):
        uart, _, _ = transport()
        inside, overlaps = [], []

        def worker(name):
            with uart.transaction():
                with uart.transaction():                     # re-entrant for one thread
                    inside.append(name)
                    if len(inside) > 1:
                        overlaps.append(list(inside))
                    time.sleep(0.02)
                    inside.remove(name)

        threads = [threading.Thread(target=worker, args=(n,)) for n in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(5)
        self.assertEqual(overlaps, [])


if __name__ == "__main__":
    unittest.main()
