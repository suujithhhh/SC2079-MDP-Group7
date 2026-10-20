#!/usr/bin/env python3
"""Interactive STM console for the Pi: type a command, see every reply line.

For bench work such as speed-profile calibration, over SSH, with no Android,
planner or map. It is the Pi-side equivalent of the PC's uart_test.py, built on
uart.py so it uses the same port settings (115200 8N1), line framing and
failure handling as the mission program.

    ~/mdp/.venv/bin/python stm_console.py --stm /dev/serial/by-id/<name>
    stm> mov,f,30
    stm> spprof get
    stm> stopctrl 120 1800 1

Input is upper-cased, and spaces become commas ("spprof get" -> "SPPROF,GET").
Command names and ranges are in RTOSV/Core/Src/vision_rx.c (type `help`).
One command is in flight at a time and nothing is ever resent. `STOP` brakes a move
in progress; the STM answers it with ACK,STOPPED, which also ends a MOV that was
stopped (by STOP, or by the ultrasonic parking protection, USPARK).

Automatic USPARK stops: when ACK,STOPPED follows a `USPARK,STOP,...` cause line, the
STM then sends the measurement trace (`USPARK,TRACE,...`, bench builds only). The
console keeps the same UART transaction open and keeps collecting, printing and
logging every line until `USPARK,TRACE,EV:DONE,` (at most 10 s after ACK,STOPPED, then a
warning), and sends nothing meanwhile. A manual STOP, or a stop with no USPARK,STOP
cause line, ends at ACK,STOPPED as before.

Ctrl+C: during a MOV it sends ONE emergency STOP and waits for ACK,STOPPED; if that
fails or you press Ctrl+C again, cut motor power (the console never resends STOP, or
any command, by itself). During any other command Ctrl+C only stops the waiting: the
STM may still be executing it, and the next command may answer ERR,BUSY. At the idle
`stm>` prompt Ctrl+C exits.
"""

from __future__ import annotations

import argparse
import re
import time
from typing import Callable

from stm_link import mov_timeout_s, summarise_motion
from uart import UartError, UartFramingError, UartTransport, open_uart

TELEMETRY_PREFIXES = ("MM,", "AK,", "TB,", "MD,", "MS,", "HS,", "RTG,", "MP,", "SPC,", "STP,", "SFF,",
                      "TL,", "TR,", "TS,")
FIRST_REPLY_WAIT_S = 3.0    # non-MOV: give up waiting for a first reply after this
IDLE_S = 0.6                # non-MOV: the reply is complete after this much silence
STOP_WAIT_S = 3.0           # STOP: ACK,STOPPED normally follows within ~150 ms (PROTOCOL.md 3.3)
ACK_LINE = "ACK"
STOPPED_LINE = "ACK,STOPPED"    # exact token; also begins with "ACK" but is not movement success
USPARK_STOP_PREFIX = "USPARK,STOP,"            # cause line of an automatic parking stop
TRACE_PREFIX = "USPARK,TRACE,"                 # measurement trace lines (bench builds); never hidden
TRACE_DONE_PREFIX = "USPARK,TRACE,EV:DONE,"    # last line of a trace
TRACE_DRAIN_S = 10.0        # after ACK,STOPPED of an automatic stop: wait at most this long for EV:DONE
                            # (a firmware with the shadow predictor sends its block first: about 6 s in all)

HELP = """\
Commands (one per line; upper/lower case and spaces are fine):
  MOV,F|B,<cm>  MOV,FL|FR|BL|BR,<deg>       waits for ACK / ACK,STOPPED / ERR (with a timeout)
  STOP                                      brake now; ends at ACK,STOPPED. A MOV it interrupts
                                            also ends at ACK,STOPPED: do not assume its endpoint
  USPARK,1   ultrasonic parking protection ON (idle only: USPARK,OK / ERR,BUSY / ERR,US_FAULT)
  USPARK,0   OFF (any time: USPARK,OK)      USPARK,GET   state (USPARK,E:...,ST:...)
             while ON, a forward MOV that nears an obstacle is stopped automatically; the
             cause line USPARK,STOP,R:... precedes ACK,STOPPED. Off after every STM reset.
             After such an automatic stop the console keeps collecting the USPARK,TRACE,...
             lines through EV:DONE (<= 10 s after ACK,STOPPED) and sends nothing meanwhile.
  PROFILE,PRECISION|STANDARD|TASK2TEST|GET
  SPCTRL,<target>,<kp>,<ki>,<en>   SPCTRL,GET     speed control
  SPPROF,<decelStart>,<approach>,<en>  SPPROF,GET  speed profile
  STOPCTRL,<lead>,<entrySpeed>,<en>   STOPCTRL,GET final stop gate
  SFF,<F|B>,<start>,<full>,<maxUs>,<en>   SFF,GET
  MOVCAL,<slope>,<offset>,<en>   MOVCAL,GET
  DPROF / WSYNC / HCTRL / RTGUARD / PRESTEER / TURNSPEED / ACKERMANN, ... ,GET
  ENC,GET  ENC,RST  IMU,GET  IMU,ZERO  SERVO,GET  PID,GET ...
Other commands: any line the STM accepts (max 31 characters) is sent as typed.
Console only:  help | telemetry on|off | quit
Ctrl+C:        during a MOV it sends ONE emergency STOP and waits for ACK,STOPPED (if that fails,
               or you press Ctrl+C again, cut motor power). During other commands it only stops
               the waiting. At the idle prompt it exits. The console never resends STOP.
"""


def normalise(text: str) -> str:
    """'mov, f 30' -> 'MOV,F,30'."""
    text = re.sub(r"\s*,\s*", ",", text.strip().upper())
    return re.sub(r"\s+", ",", text)


def is_telemetry(line: str) -> bool:
    return line.startswith(TELEMETRY_PREFIXES)


def run_command(
    uart: UartTransport,
    command: str,
    out: Callable[[str], None] = print,
    show_telemetry: bool = True,
    clock: Callable[[], float] = time.monotonic,
    raw: Callable[[str], None] | None = None,
) -> list[str] | None:
    """Send one command and print the lines that come back. Returns every line
    received (None if nothing was sent). MOV ends at the exact line `ACK`,
    `ACK,STOPPED` (stopped: not success, endpoint unknown) or `ERR,...`; STOP
    ends at the exact `ACK,STOPPED` (a plain `ACK` is the interrupted move's
    own completion and is only shown); any other command is finished by silence
    (so multi-line replies like PID,GET are shown whole). Matching is exact.
    If that `ACK,STOPPED` follows a `USPARK,STOP,` cause line (an automatic parking
    stop) the transaction stays open and every further line (the USPARK,TRACE
    measurement trace) is shown, logged and returned until `USPARK,TRACE,EV:DONE,`,
    at most TRACE_DRAIN_S after the `ACK,STOPPED`, then a warning; nothing is sent
    while draining. Nothing is ever resent. Raises UartError if the link fails."""
    is_move = command.startswith("MOV,")
    is_stop = command == "STOP"
    timeout_s = None
    if is_move:
        try:
            timeout_s = mov_timeout_s(command)
        except ValueError as exc:
            out(f"not sent: {exc}")
            return None
    elif is_stop:
        timeout_s = STOP_WAIT_S
    lines: list[str] = []
    hidden: list[str] = []     # received but not printed (telemetry switched off)
    stopped = False
    reported = False           # the stop report has already been printed (trace drain)
    drain_until: float | None = None    # set while collecting the trace after an automatic stop
    trace_lines = 0
    trace_done = False

    def finish_report() -> None:
        if is_move:
            summary = summarise_motion(lines)
            if summary:
                out(f"          {summary}")
        if stopped:
            report_stop(lines, hidden, command, out)

    with uart.transaction():
        stale = uart.discard_input()
        if stale:
            out(f"(discarded {stale} stale bytes)")
        uart.send_line(command)
        if raw:
            raw(f"<- {command}")
        started = last = clock()
        while True:
            now = clock()
            if drain_until is not None:
                remaining = drain_until - now
            elif timeout_s is not None:
                remaining = started + timeout_s - now
            else:
                remaining = (IDLE_S if lines else FIRST_REPLY_WAIT_S) - (now - last)
            if remaining <= 0:
                if drain_until is not None:
                    out(f"WARNING: USPARK trace incomplete: no {TRACE_DONE_PREFIX} within {TRACE_DRAIN_S:.1f} s "
                        f"of ACK,STOPPED ({trace_lines} trace lines received). The trace may have been lost; "
                        "nothing was resent.")
                elif is_stop:
                    out(f"TIMEOUT after {timeout_s:.1f} s with no ACK,STOPPED; the robot may still be moving. "
                        "STOP is NOT resent: type STOP again if you want to.")
                elif timeout_s is not None:
                    out(f"TIMEOUT after {timeout_s:.1f} s with no ACK/ACK,STOPPED/ERR; the robot may still be "
                        "moving or may have moved. The command is NOT resent.")
                elif not lines:
                    out("(no reply)")
                break
            try:
                line = uart.read_line(remaining)
            except UartFramingError as exc:
                out(f"(garbled line ignored: {exc})")
                continue
            if line is None:
                continue
            last = clock()
            lines.append(line)
            if raw:
                raw(f"-> {line}")
            if line.startswith(TRACE_PREFIX):
                trace_lines += 1
            if show_telemetry or line.startswith(TRACE_PREFIX) or not is_telemetry(line):
                out(f"{last - started:7.2f}s  {line}")
            else:
                hidden.append(line)
            if drain_until is not None:
                # Collecting the trace: only EV:DONE ends it (an ERR or ACK here is not ours to act on).
                if line.startswith(TRACE_DONE_PREFIX):
                    trace_done = True
                    break
                continue
            if line == STOPPED_LINE and (is_move or is_stop):
                stopped = True
                if any(seen.startswith(USPARK_STOP_PREFIX) for seen in lines):
                    # Automatic parking stop: the STM sends the trace AFTER this reply. Report the stop now,
                    # then keep the transaction open (sending nothing) until EV:DONE or the bound.
                    drain_until = last + TRACE_DRAIN_S
                    finish_report()
                    reported = True
                    out(f"          collecting the USPARK trace until EV:DONE (at most {TRACE_DRAIN_S:.1f} s); "
                        "nothing is sent meanwhile")
                    continue
                break
            if line.startswith("ERR,"):
                break
            if line == ACK_LINE and not is_stop:
                break
    if trace_done:
        out(f"          USPARK trace complete ({trace_lines} lines, through EV:DONE)")
    if not reported:
        finish_report()
    return lines


def report_stop(lines: list[str], hidden: list[str], command: str, out: Callable[[str], None]) -> None:
    """After `ACK,STOPPED`: say that the movement was stopped, show its cause, and
    make sure the stop sequence is visible even with telemetry switched off."""
    faults = [line for line in lines if line.startswith("USPARK,FAULT,")]
    causes = [line for line in lines if line.startswith("USPARK,STOP,")]
    ends = [line for line in hidden if line.startswith("MM,EV:END")]
    out("")
    if command == "STOP":
        out("STOPPED (ACK,STOPPED): STOP acknowledged. If a movement was running it was braked before "
            "finishing; its planned endpoint must NOT be assumed.")
    else:
        out(f"MOVEMENT STOPPED (ACK,STOPPED): {command} did not complete. The robot was braked part-way; "
            "its planned endpoint must NOT be assumed (the pose is unknown).")
    for line in ends:                     # hidden telemetry: still show how the movement ended
        out(f"          {line}")
    for line in faults:
        out(f"          ultrasonic fault: {line}")
    if causes:
        out(f"          automatic ultrasonic parking stop: {causes[0]}")
    else:
        out("          no USPARK,STOP cause line was received (a manual STOP, or USPARK was not involved)")


def emergency_stop(
    uart: UartTransport,
    show_telemetry: bool = True,
    raw: Callable[[str], None] | None = None,
    out: Callable[[str], None] = print,
    clock: Callable[[], float] = time.monotonic,
) -> bool:
    """Ctrl+C during a MOV: send exactly ONE `STOP` through the normal run_command()
    STOP path and wait for the exact `ACK,STOPPED`. Never resends; if the STOP is not
    acknowledged, the link fails, or Ctrl+C is pressed again, say that motor power
    should be cut. Returns True only when `ACK,STOPPED` was received."""
    cut_power = ("The robot may still be moving: CUT MOTOR POWER now. "
                 "STOP was sent once and is NOT resent.")
    try:
        out("\nCtrl+C during a MOV: sending an emergency STOP now (once; it is never resent). "
            "Waiting for ACK,STOPPED...")
        lines = run_command(uart, "STOP", out=out, show_telemetry=show_telemetry, clock=clock, raw=raw)
    except KeyboardInterrupt:
        out("\nInterrupted again while waiting for ACK,STOPPED. " + cut_power)
        return False
    except UartError as exc:
        out(f"UART error while sending the emergency STOP: {exc}")
        out(cut_power)
        return False
    if lines is not None and STOPPED_LINE in lines:
        return True
    out("No ACK,STOPPED was received. " + cut_power)
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stm", default="/dev/ttyACM0",
                        help="STM serial device; prefer the stable /dev/serial/by-id/... path")
    parser.add_argument("--no-telemetry", action="store_true", help="start with MM/MD/SPC/... lines hidden")
    parser.add_argument("--log", help="append every line sent and received to this file")
    args = parser.parse_args()
    try:
        import readline  # noqa: F401  (arrow-key history, where available)
    except ImportError:
        pass
    try:
        uart = open_uart(args.stm)
    except UartError as exc:
        raise SystemExit(str(exc))
    log = open(args.log, "a") if args.log else None

    def raw(text: str) -> None:
        if log:
            log.write(f"{time.strftime('%H:%M:%S')} {text}\n")
            log.flush()

    show = not args.no_telemetry
    print(f"Connected to {args.stm} (115200 8N1). `help` lists commands, `quit` exits. "
          "`stop` brakes a move in progress (it answers ACK,STOPPED).")
    try:
        while True:
            try:
                text = input("stm> ")
            except EOFError:
                break
            command = normalise(text)
            if not command:
                continue
            if command in ("QUIT", "EXIT", "Q"):
                break
            if command == "HELP":
                print(HELP)
                continue
            if command.startswith("TELEMETRY"):
                show = not command.endswith("OFF")
                print(f"telemetry lines {'shown' if show else 'hidden'}")
                continue
            try:
                run_command(uart, command, show_telemetry=show, raw=raw)
            except KeyboardInterrupt:
                if command.startswith("MOV,"):
                    emergency_stop(uart, show_telemetry=show, raw=raw)
                    if uart.closed or uart.fault:
                        print("The link is down; restart after checking the cable.")
                        break
                else:
                    print("\nStopped waiting. The STM may still be executing the command (type `stop` to brake "
                          "it); nothing was resent.")
            except UartError as exc:
                print(f"UART error: {exc}")
                if uart.closed or uart.fault:
                    print("The link is down; restart after checking the cable.")
                    break
    except KeyboardInterrupt:
        print()
    finally:
        uart.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
