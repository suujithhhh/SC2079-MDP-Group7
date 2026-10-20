"""Front ultrasonic distance from the STM: the `USD,GET` query (PROTOCOL.md §4.8) when the firmware answers it,
else the `US,` bench stream (PROTOCOL.md §6.2.2 bench lines).

Firmware with `APP_ULTRASONIC_BENCH_LOG=1` (e.g. the team's `rstovv2` build) sends, about every 200 ms and
without being asked (source: app_comm.c `AppComm_TrySendUltrasonicLog`):

    US,ST:<state>,R:<result>,PW:<us|NA>,RAW:<mm|NA>,FLT:<mm|NA>,V:<0|1>,F:<0|1>,AGE:<ms|NA>,SEQ:<n>,CYC:<n>,
       OK:<n>,NR:<n>,NF:<n>,NEAR:<n>,FAR:<n>,EDG:<n>,LATE:<n>,ES:<n>,STL:<n>,SRS:<n>,SPU:<n>,SCP:<n>,FM:<hex>

FLT is the median of the last 3 valid echoes, in mm from the transducer faces (bench calibration in
app_ultrasonic.h); F:1 means it is fresh (a valid echo within 200 ms); AGE is the time since that echo.
A bench line has no request, so the Pi only listens: about 1 s for the US_SAMPLE_LINES lines of one reading.

Firmware with `USD,GET` (app_comm.c `AppComm_TrySendUsdGet`) answers one query in one telemetry pass with the
newest publication (the sensor publishes about every 61 ms), so one reading takes a few hundred ms:

    USD,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,D:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>

Both become `UsLine`s. A line is trusted only with no fault, a fresh numeric distance (FLT with F:1, or
ST:OK with a number) and AGE <= US_MAX_AGE_MS. One reading is the median of several trusted lines with
distinct SEQ / S that agree within US_AGREE_CM; anything less is "no reading" (distance None, `problem`
says why), never a guess.

The ultrasonic sees the nearest surface in its cone (about +-15 deg), straight ahead only.
"""

from __future__ import annotations

import re
import statistics
from dataclasses import dataclass, field

import config

US_PATTERN = re.compile(
    r"US,ST:(?P<st>[A-Z?]+),R:(?P<r>[A-Z?]+),PW:(?:\d+|NA),RAW:(?P<raw>\d+|NA),FLT:(?P<flt>\d+|NA),"
    r"V:(?P<v>[01]),F:(?P<f>[01]),AGE:(?P<age>\d+|NA),SEQ:(?P<seq>\d+),CYC:\d+,"
    r"OK:\d+,NR:\d+,NF:\d+,NEAR:\d+,FAR:\d+,EDG:\d+,LATE:\d+,ES:\d+,STL:\d+,SRS:\d+,SPU:\d+,SCP:\d+,"
    r"FM:(?P<fm>[0-9A-F]{2,4})")


@dataclass(frozen=True)
class UsLine:
    state: str
    result: str
    raw_mm: int | None
    filtered_mm: int | None
    valid: bool
    fresh: bool
    age_ms: int | None
    seq: int
    fault_mask: int
    received_at_s: float = 0.0

    def trusted(self) -> str | None:
        """None when this line may be used, else why not."""
        if self.fault_mask or self.state == "FAULT":
            return f"fault FM:{self.fault_mask:02X}"
        if not self.fresh or self.filtered_mm is None:
            return f"no fresh echo (R:{self.result})"
        if self.age_ms is None or self.age_ms > config.US_MAX_AGE_MS:
            return f"AGE {self.age_ms} ms"
        return None


def parse_us(text: str, received_at_s: float = 0.0) -> UsLine | None:
    """One `US,` bench line, or None if it does not match the grammar exactly."""
    match = US_PATTERN.fullmatch(text)
    if match is None:
        return None
    number = lambda key: None if match[key] == "NA" else int(match[key])   # noqa: E731
    return UsLine(match["st"], match["r"], number("raw"), number("flt"), match["v"] == "1", match["f"] == "1",
                  number("age"), int(match["seq"]), int(match["fm"], 16), received_at_s)


# PROTOCOL.md §4.8, the frozen strict grammar (full match).
USD_PATTERN = re.compile(
    r"USD,S:(?P<s>0|[1-9][0-9]{0,9}),ST:(?P<st>OK|WARM|STALE|ERR|FAULT),D:(?P<d>NEAR|FAR|NA|[1-9][0-9]{1,3}),"
    r"AGE:(?P<age>NA|0|[1-9][0-9]{0,4}),F:(?P<f>[0-9A-F]{2,})")
USD_NOT_FRESH = 0x40            # F bit: the newest publication is not a fresh good one (§4.5 / §4.8)


def parse_usd(text: str, received_at_s: float = 0.0) -> UsLine | None:
    """One `USD,GET` reply as a UsLine, or None if it breaks the grammar or the ST / D / F rules of §4.8 (OK needs
    a distance or NEAR / FAR and F:00; WARM needs NA and F:00; the others NA and the 40 bit). A distance is
    filtered_mm, fresh only for ST:OK with a number; F without the 40 bit is the fault mask."""
    match = USD_PATTERN.fullmatch(text)
    if match is None:
        return None
    state, d, flags = match["st"], match["d"], int(match["f"], 16)
    numeric = d.isdigit()
    if state == "OK":
        if d == "NA" or flags:
            return None
    elif d != "NA" or (state == "WARM" and flags) or (state != "WARM" and not flags & USD_NOT_FRESH):
        return None
    age = None if match["age"] == "NA" else int(match["age"])
    return UsLine(state, d if not numeric else "OK", None, int(d) if numeric else None, numeric, numeric,
                  age, int(match["s"]), flags & ~USD_NOT_FRESH, received_at_s)


@dataclass
class FrontReading:
    """distance_cm: ultrasonic face -> nearest surface ahead, or None (then `problem` says why).
    used: trusted lines that agreed; seen: `US,` lines read; values_cm: every trusted value."""

    distance_cm: float | None
    used: int = 0
    seen: int = 0
    values_cm: list[float] = field(default_factory=list)
    problem: str | None = None

    @property
    def ok(self) -> bool:
        return self.distance_cm is not None


def summarise(lines: list[UsLine], seen: int | None = None) -> FrontReading:
    """Median of the trusted lines (distinct SEQ) that lie within US_AGREE_CM of their median; at least
    US_MIN_AGREEING of them or no reading."""
    seen = len(lines) if seen is None else seen
    if seen == 0:
        return FrontReading(None, 0, 0, [], "no US, lines (firmware without the ultrasonic bench log?)")
    by_seq: dict[int, UsLine] = {}
    reasons: list[str] = []
    for line in lines:
        why = line.trusted()
        if why is None:
            by_seq.setdefault(line.seq, line)
        else:
            reasons.append(why)
    values = [line.filtered_mm / 10.0 for line in by_seq.values()]
    if not values:
        return FrontReading(None, 0, seen, [], "no trusted line: " + "; ".join(sorted(set(reasons))))
    middle = statistics.median(values)
    agreeing = [v for v in values if abs(v - middle) <= config.US_AGREE_CM]
    if len(agreeing) < config.US_MIN_AGREEING:
        return FrontReading(None, len(agreeing), seen, values,
                            f"only {len(agreeing)} of {len(values)} trusted values agree within "
                            f"{config.US_AGREE_CM:g} cm (need {config.US_MIN_AGREEING})")
    return FrontReading(statistics.median(agreeing), len(agreeing), seen, values)
