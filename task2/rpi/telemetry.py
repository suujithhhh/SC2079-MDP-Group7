"""Structured run log: one JSON object per line in <run dir>/events.jsonl,
plus a readable line on the console."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable


class RunLog:
    def __init__(self, path: Path | None, echo: Callable[[str], None] | None = print,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._file = open(path, "a") if path else None
        self._echo = echo
        self._clock = clock
        self._t0 = clock()
        self.events: list[dict] = []

    def event(self, kind: str, text: str = "", **data) -> None:
        record = {"t": round(self._clock() - self._t0, 3), "kind": kind, **({"text": text} if text else {}), **data}
        self.events.append(record)
        if self._file:
            self._file.write(json.dumps(record, default=str) + "\n")
            self._file.flush()
        if self._echo:
            extra = " ".join(f"{k}={v}" for k, v in data.items())
            self._echo(f"[{record['t']:7.2f}] {kind:<10} {text} {extra}".rstrip())

    def state(self, name: str) -> None:
        self.event("state", name)

    def close(self) -> None:
        if self._file:
            self._file.close()
